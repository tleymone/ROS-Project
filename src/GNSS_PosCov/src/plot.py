#!/usr/bin/env python3

import rospy
from matplotlib import pyplot as plt
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Point, PoseWithCovariance, PoseWithCovarianceStamped
import numpy as np
import pymap3d as pm
import tf
import time
import math
import copy
#import seaborn as sns
#from matplotlib.patches import Ellipse
#import matplotlib.pyplot as plt
#import matplotlib.transforms as transforms

distance = [0.66914, -0.00135, 1.5011]
class DrawError(object):
    x_o = []
    y_o = []
    offset = [0,0,0]
    #fig = plt.gca()
    pub_Cov = []
    pub_Cov_o = []
    pub_Cov_s = []
    pub_NavSatFix = []
    x = 0
    def __init__(self):
        self.pub_Cov = rospy.Publisher('posCov_m', PoseWithCovarianceStamped, queue_size=10)
        self.pub_Cov_o = rospy.Publisher('posCov_o', PoseWithCovarianceStamped, queue_size=10)
        self.pub_Cov_s = rospy.Publisher('posCov_s', PoseWithCovarianceStamped, queue_size=10)
        self.pub_NavSatFix = rospy.Publisher('navSatFix_m', NavSatFix, queue_size=10)
        self.x=0
        pass
    
    def GNSS_callback(self, data):
        self.x+=1
        if(self.x==5):
            #plt.cla()
            self.x=0
        lat = data.latitude
        long = data.longitude
        alt = data.altitude  
        p = pm.geodetic2enu(lat, long, alt, 49.401322,2.797066, 73.5217)
        posCov = PoseWithCovarianceStamped()
        posCov.header.frame_id="world"
        self.x_o.append(p[0])
        self.y_o.append(p[1])
        posCov.pose.pose.position.x = p[0]
        posCov.pose.pose.position.y = p[1]
        posCov.pose.pose.position.z = 0.0
        posCov.pose.pose.orientation.x = 0.0
        posCov.pose.pose.orientation.y = 0.0
        posCov.pose.pose.orientation.z = 0.0
        posCov.pose.pose.orientation.w = 0.0
        
        c = np.zeros((3,3,))
        i, j = 0, 0
        for cov in data.position_covariance:
        	c[j][i] = cov
        	i += 1
        	if(i==3):
        		i = 0
        		j += 1
        lam,vet = np.linalg.eig(c)
        #test1 = np.matrix(vet).T*c*vet
        #test2 = np.matrix(vet.T).I*test1*np.matrix(vet).I
        #print(c,test1,vet,test2)
        lam,vet = np.linalg.eigh(c)
        lam = lam[::-1]
        c1 = np.zeros((3,3,))
        for i in range(3):
        	vet[i,:]=vet[i,:][::-1]
        	c1[i,i]=lam[i]
        c1[0,0] = 0
        test2 = np.matrix(vet.T).I*c1*np.matrix(vet).I
        #plt.ion()
        #line, =de.fig.plot(self.x_o, self.y_o, color="b", marker = 'x',label="/gnss_point")
        #line.set_data(p[0],p[1])
        #plt.show()
        i = 0
        data.position_covariance = [0,0,0,0,0,0,0,0,0]
        data.position_covariance[0]=test2[0,0]
        data.position_covariance[1]=test2[0,1]
        data.position_covariance[3]=test2[1,0]
        data.position_covariance[4]=test2[1,1]
        posCov.pose.covariance = np.zeros(36)
        #pearson = test2[0, 1]/np.sqrt(test2[0, 0] * test2[1, 1])
        #ell_radius_x = np.sqrt(1 + pearson)
        #ell_radius_y = np.sqrt(1 - pearson)
        #ellipse = Ellipse((0, 0), width=ell_radius_x * 2, height=ell_radius_y * 2,
        #              edgecolor='r', fc='None', lw=2)
        #scale_x = np.sqrt(test2[0, 0]) * 3
        #scale_y = np.sqrt(test2[1, 1]) * 3
        #ellipse = Ellipse(xy=(0, 0), width=ell_radius_x, height=ell_radius_y, 
        #               edgecolor='r', fc='None', lw=1)
        #transf = transforms.Affine2D() \
        #       .rotate_deg(45) \
        #       .scale(scale_x, scale_y)\
        #       .translate(p[0], p[1])
        #ellipse.set_transform(transf+self.fig.transData)
        #self.fig.add_patch(ellipse)
        #pearson = c[0, 1]/np.sqrt(c[0, 0] * c[1, 1])
        #ell_radius_x = np.sqrt(1 + pearson)
        #ell_radius_y = np.sqrt(1 - pearson)
        #scale_x = np.sqrt(c[0, 0]) * 3
        #scale_y = np.sqrt(c[1, 1]) * 3
        #ellipse2 = Ellipse(xy=(0, 0), width=ell_radius_x, height=ell_radius_y, 
        #               edgecolor='b', fc='None', lw=1)
        #transf = transforms.Affine2D() \
        #       .rotate_deg(45) \
        #       .scale(scale_x, scale_y)\
        #       .translate(p[0], p[1])
        #ellipse2.set_transform(transf+self.fig.transData)
        #self.fig.add_patch(ellipse2)
        posCov.pose.covariance[0] = c[0,0]
        posCov.pose.covariance[1] = c[0,1]
        posCov.pose.covariance[2] = c[0,2]
        posCov.pose.covariance[6] = c[1,0]
        posCov.pose.covariance[7] = c[1,1]
        posCov.pose.covariance[8] = c[1,2]
        posCov.pose.covariance[12] = c[2,0]
        posCov.pose.covariance[13] = c[2,1]
        posCov.pose.covariance[14] = c[2,2]
        posCov_o = copy.deepcopy(posCov)
        posCov.pose.covariance[2] = 0
        posCov.pose.covariance[8] = 0
        posCov.pose.covariance[12] = 0
        posCov.pose.covariance[13] = 0
        posCov.pose.covariance[14] = 0
        posCov_s = copy.deepcopy(posCov)
        posCov.pose.pose.position.x -= self.offset[0]
        posCov.pose.pose.position.y -= self.offset[1]
        posCov.pose.covariance[0] = test2[0,0]
        posCov.pose.covariance[1] = test2[0,1]
        posCov.pose.covariance[6] = test2[1,0]
        posCov.pose.covariance[7] = test2[1,1]
        #print(posCov_s.pose.pose.position,posCov.pose.pose.position)
        self.pub_Cov.publish(posCov)
        self.pub_NavSatFix.publish(data)
        self.pub_Cov_o.publish(posCov_o)
        self.pub_Cov_s.publish(posCov_s)
        
    def pose_callback(self, data):
    	quaternion = (data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z, data.pose.pose.orientation.w)
    	angle = tf.transformations.euler_from_quaternion(quaternion)
    	self.offset[0]=distance[0]*math.cos(angle[2])+distance[1]*math.sin(angle[2])
    	self.offset[1]=distance[0]*math.sin(angle[2])-distance[1]*math.cos(angle[2])
    	#print(angle[2])
	
	



if __name__ == "__main__":
    rospy.init_node("gnss")
    #plt.figure(1)
    #plt.title("display")
    de = DrawError()
    sub_GNSS = rospy.Subscriber('/septentrio/navsatfix', NavSatFix, de.GNSS_callback)
    sub_Pose = rospy.Subscriber('/kalman', PoseWithCovarianceStamped, de.pose_callback)
    #de.fig.plot(de.x_o, de.y_o, color="b", marker = 'x',label="/gnss_point")

    #plt.legend()
    #plt.show()
    rospy.spin()
