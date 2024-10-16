#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Dec  2 14:43:43 2021

@author: thomas
"""

from cmath import cos
import numpy as np
import math
import rospy
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Point, PoseWithCovariance, PoseWithCovarianceStamped, Pose2D
from can_zoe.msg import Kinematics
import pymap3d as pm
from scipy.stats import norm


def sub_mod_pi2(x):
    while np.abs(x)>np.pi:
        x=x-2*np.pi
    return x


def yaw_from_quaternion(x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return yaw_z # in radians

def euler_to_quaternion(roll, pitch, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [qx, qy, qz, qw]

# Variable à l'état initial
P = np.matrix([[3., 0., 0.],
		    [0., 3., 0.],
		    [0., 0., 1.]])
Q = np.matrix([[0.1, 0],
		    [0, 0.001]])

U = np.matrix([[0.001, 0.001, 0.001],[0.001, 0.001, 0.001],[0.001, 0.001, 0.001]])

I = np.eye(3)
vit_long = 0
t = t_old = Te = 0
pose_init = None
gnss = None
count = 0

# Estimation fait à partir des estimations de l'IMU et de l'odometrie
def IMU_callback(msg):
    global x, y, vit_long, X, t, t_old, Te, P, Q, pose_init

    if pose_init!=None:
        t = (msg.header.stamp.secs * 10 ** 9 + msg.header.stamp.nsecs) * 10 ** (-9)
        if t_old == 0:
            Te = 0
        else:
            Te = (t - t_old)
        w = msg.angular_velocity.z

        A = np.matrix([[1, 0, -vit_long * Te *np.sin(X[2,0])],
                        [0, 1, vit_long * Te * np.cos(X[2,0])],
                        [0, 0, 1]])
        B = np.matrix([[Te *np.cos(X[2,0]), 0],
                    [Te *np.sin(X[2,0]), 0],
                    [0, Te]])


        X[0] += vit_long * Te * np.cos(X[2,0])
        X[1] += vit_long * Te * np.sin(X[2,0])
        X[2] += Te * w
        X[2,0] = sub_mod_pi2(X[2, 0])
        P = np.dot(A,np.dot(P,A.T)) + np.dot(B, np.dot(Q,B.T)) + U


        cov = np.zeros(36)
        cov[0] = P[0,0]
        cov[1] = P[0,1]
        cov[5] = P[0,2]
        cov[6] = P[1, 0]
        cov[7] = P[1, 1]
        cov[11] = P[1,2]
        cov[30] = P[2, 0]
        cov[31] = P[2, 1]
        cov[35] = P[2,2]

        qua = euler_to_quaternion(0, 0, X[2, 0])
        output = PoseWithCovarianceStamped()
        output.pose.pose.position.x = X[0,0]
        output.pose.pose.position.y = X[1,0]
        output.pose.pose.position.z = 0.0
        output.pose.pose.orientation.x = qua[0]
        output.pose.pose.orientation.y = qua[1]
        output.pose.pose.orientation.z = qua[2]
        output.pose.pose.orientation.w = qua[3]
        output.pose.covariance = cov
        output.header.frame_id = "map"
        output.header.stamp = msg.header.stamp
        pub_kalman.publish(output)
        t_old = t

# On stocke la vitesse longitudinale de l'odométrie qui sera utillisé dans le callback de l'IMU
def ODO_callback(msg):
    global vit_long, yaw_rate
    vit_long = msg.longitudinal_speed
    yaw_rate = msg.yaw_rate

# Mise à jour quand on reçoit une donnée GNSS
def GNSS_callback(msg):
    global X, P, I, gnss, pose_init
    lat = msg.latitude
    long = msg.longitude
    alt = msg.altitude
    p = pm.geodetic2enu(lat, long, alt, 49.401322  , 2.797066, 73.5217) #transformations en coordonnées ENU

    if pose_init == None: # Notre pose initiale est donnée par le septentrio avec theta fixé
            X = np.zeros( (3, 1) )
            X[0] = p[0]
            X[1] = p[1]
            X[2] = 0.3
            pose_init = 1

    y = np.matrix([[p[0] - 0.6 * np.cos(X[2,0])], [p[1]] - 0.6 * np.sin(X[2,0])])

    R = 2 * np.matrix(msg.position_covariance)
    R.shape = (3, 3)
    R = np.matrix(R[0:2, 0:2])
    C = np.matrix([[1., 0., 0.],[0., 1., 0.]])

    K = np.dot(P, np.dot(C.T, np.linalg.inv((np.dot(C,np.dot(P, C.T))) + R)))
    if pose_init != None:
       X = X + K @ (y - C @ X)
       P = (I - K @ C) @ P @ np.transpose(I - K @ C) + K @ R @ np.transpose(K)

    point = Point()
    point.x = y[0]
    point.y = y[1]
    pub_gnss.publish(point)

def lidar_callback(msg):
    global X, P, I, gnss, pose_init
    yaw = yaw_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    yaw = sub_mod_pi2(yaw)
    dt = t - (msg.header.stamp.secs * 10 ** 9 + msg.header.stamp.nsecs) * 10 ** (-9)

    y = np.matrix([[msg.pose.pose.position.x + dt*vit_long*np.cos(yaw)], [msg.pose.pose.position.y + dt*vit_long*np.sin(yaw)], [yaw + dt*yaw_rate]])
    if abs(y[2] - X[2]) > 0.15:
        return
    R = np.matrix([[3 + 8*abs(yaw_rate), 0, 0], [0, 3 + 8*abs(yaw_rate), 0], [0, 0, 1 + 5*abs(yaw_rate)]])

    C = np.matrix([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]])

    K = np.dot(P, np.dot(C.T, np.linalg.inv((np.dot(C,np.dot(P, C.T))) + R)))
    if pose_init != None:
       X = X + K @ (y - C @ X)
       X[2,0] = sub_mod_pi2(X[2, 0])
       P = (I - K @ C) @ P @ np.transpose(I - K @ C) + K @ R @ np.transpose(K)

def rtk_callback(msg):
    global X, pose_init, gnss
    yaw =  yaw_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
    point = Pose2D()
    point.x = msg.pose.pose.position.x
    point.y = msg.pose.pose.position.y
    point.theta = yaw

    pub_rtk.publish(point)

def rtk_bruit_callback(msg):
    global X, P, I, gnss, pose_init, count
    yaw =  yaw_from_quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)

    if count > 10:
        count = 0
        ox = msg.pose.covariance[0]
        oy = msg.pose.covariance[7]
        otheta = msg.pose.covariance[35]
        ex = norm.rvs(0, np.sqrt(ox))
        ey = norm.rvs(0, np.sqrt(oy))
        etheta = norm.rvs(0, np.sqrt(otheta))

        y = np.matrix([[msg.pose.pose.position.x + 8*ex], [msg.pose.pose.position.y + 8*ey], [yaw + 8*etheta]])

        R = np.matrix([[msg.pose.covariance[0], msg.pose.covariance[1], msg.pose.covariance[5]], [msg.pose.covariance[6], msg.pose.covariance[7], msg.pose.covariance[11]], [msg.pose.covariance[30], msg.pose.covariance[31], msg.pose.covariance[35]]])
        
        
        R = 70 * R
        C = np.matrix([[1., 0., 0.],[0., 1., 0.],[0., 0., 1.]])

        K = np.dot(P, np.dot(C.T, np.linalg.inv((np.dot(C,np.dot(P, C.T))) + R)))
        if pose_init != None:
           X = X + K @ (y - C @ X)
           X[2,0] = sub_mod_pi2(X[2, 0])
           P = (I - K @ C) @ P @ np.transpose(I - K @ C) + K @ R @ np.transpose(K)
    else:
        count += 1
        return





if __name__=='__main__':
    rospy.init_node('Kalmanr') # Registers the node with name 'Kalmanr'
    pub_kalman = rospy.Publisher('kalman', PoseWithCovarianceStamped, queue_size=10)
    pub_rtk = rospy.Publisher('rtk', Pose2D, queue_size=10)
    pub_gnss = rospy.Publisher('gnss', Point, queue_size=10)
    sub_imu = rospy.Subscriber('/span/imu/data', Imu, IMU_callback) # Subscribes to topic 'number'
    sub_rtk = rospy.Subscriber('/span/enuposeCovs', PoseWithCovarianceStamped, rtk_callback)
    sub_odo = rospy.Subscriber('/vehicle/kinematics', Kinematics, ODO_callback)
    sub_GNSS = rospy.Subscriber('/septentrio/navsatfix', NavSatFix, GNSS_callback)
    sub_lidar = rospy.Subscriber('lidar_pose', PoseWithCovarianceStamped, lidar_callback)
    #sub_rtk_bruit = rospy.Subscriber('/span/enuposeCovs', PoseWithCovarianceStamped, rtk_bruit_callback)

    rospy.spin()
