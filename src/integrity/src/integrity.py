#!/usr/bin/env python3
from math import cos, sin
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Int8
import numpy as np
from nav_control.msg import NavError

TURN_BORDER_DISTANCE = 1.8
STRAIGHT_BORDER_DISTANCE = 1.4
CAR_WIDTH = 1.787
STOP = 0
SLOW_DOWN = 1
MAX_SPEED = 2

class Integrity :
    def __init__(self):
        self.e = None
        self.R = None
        self.o = 2
        self.link_border_distance = STRAIGHT_BORDER_DISTANCE
        self.max_from_link = self.link_border_distance - (CAR_WIDTH/2)
        self.control_state = STOP
        self.timer = None
        
        rospy.init_node('integrity', anonymous=True)
        
        rospy.Subscriber("/nav/integrity", NavError, self.NavCallback)
        rospy.Subscriber("/kalman", PoseWithCovarianceStamped, self.PoseCallback)
        #rospy.Subscriber("/span/enuposeCovs", PoseWithCovarianceStamped, self.PoseCallback)
        self.control_state_publisher = rospy.Publisher("/nav/control_state", Int8, queue_size=10)
        self.changeControlState(STOP)
        self.control_state_publisher.publish(self.control_state)
        rospy.spin() # Keeps the node waiting until ROS stops

    def NavCallback(self, msg): # Function called when a message is received
        #print('Received %f e:', msg.e, " p: ", msg.p, " c: " ,msg.c) # Prints in either log or console
        self.e = np.abs(msg.e)
        self.R = np.matrix([[cos(msg.p), -sin(msg.p)],
		                    [sin(msg.p), cos(msg.p)]])
        if msg.is_turn == True:
            self.link_border_distance = TURN_BORDER_DISTANCE
        else:
            self.link_border_distance = STRAIGHT_BORDER_DISTANCE
        self.max_from_link = self.link_border_distance - (CAR_WIDTH/2)
        
    def timerCallback(self, event):
        print("Slow Down to STOP")
        self.changeControlState(STOP)
        self.control_state_publisher.publish(self.control_state)
    
    def changeControlState(self, state):
        self.control_state = state        

    def PoseCallback(self, msg):
        if (self.e != None):
            P = np.matrix([[msg.pose.covariance[0], msg.pose.covariance[1]],
                            [msg.pose.covariance[6], msg.pose.covariance[7]]])
            print(P)
            Pf = self.R @ P @ self.R.T
            estimated_max_from_link = self.e + self.o*np.sqrt(Pf[1,1])
            print(self.e)
            print(self.o*np.sqrt(Pf[1,1]))
            print("Margin : {}".format(self.max_from_link - estimated_max_from_link))
            if estimated_max_from_link < self.max_from_link :
                print("INTEGRE")
                if(self.control_state == STOP):
                    self.changeControlState(MAX_SPEED)
                elif(self.control_state == SLOW_DOWN):
                    self.changeControlState(MAX_SPEED)
                    self.timer.shutdown()
            else:
                print("NON INTEGRE")
                if(self.control_state == MAX_SPEED):
                    self.changeControlState(SLOW_DOWN)
                    self.timer = rospy.Timer(rospy.Duration(0.5), self.timerCallback, True)

            self.control_state_publisher.publish(self.control_state)
            
            
        
if __name__=='__main__':
    mmt = Integrity()
