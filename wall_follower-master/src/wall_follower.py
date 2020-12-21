#!/usr/bin/env python2

import numpy as np

import rospy
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Point

class WallFollower:
    # Import ROS parameters from the "params.yaml" file.
    # Access these variables in class functions with self:
    # i.e. self.CONSTANT
    SCAN_TOPIC = "/scan" #rospy.get_param("wall_follower/scan_topic")
    DRIVE_TOPIC = "/vesc/ackermann_cmd_mux/input/navigation" #rospy.get_param("wall_follower/drive_topic")
    SIDE = -1 #rospy.get_param("wall_follower/side")
    VELOCITY = 0.8 #rospy.get_param("wall_follower/velocity")
    DESIRED_DISTANCE = 0.5 #rospy.get_param("wall_follower/desired_distance")    
    
    def __init__(self):
        # Initialize your publishers and
        # subscribers here
        self.laser_sub = rospy.Subscriber(self.SCAN_TOPIC, LaserScan, self.cb)
        self.drive_pub = rospy.Publisher(self.DRIVE_TOPIC, AckermannDriveStamped, queue_size=10)
        self.drive_msg = AckermannDriveStamped()
        self.drive_msg.drive.speed = self.VELOCITY
        self.dist_d = self.DESIRED_DISTANCE
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.rate = rospy.Rate(20)
        self.L = .325 # Length of car, from front to back wheel axis
        #L_lidar_back = # Length of car, from LIDAR to back wheel axis
        
        self.K_dd = 2.**-2.5
    

    # TODO:
    # Write your callback functions here.
    def cb(self, laser_msg):
        offset = (2./3. - 1./2.)*np.pi
        angle_max = laser_msg.angle_max - offset
        angle_min = laser_msg.angle_min + offset
        
        frac_angles_used = 2/4.
        ranges = np.array(laser_msg.ranges) #Convert to numpy array
        number_scans = int(frac_angles_used*len(ranges))

        if self.SIDE == 1:
            angle_max = angle_max
            angle_min = angle_max - (number_scans - 1)*laser_msg.angle_increment
            r = ranges[len(ranges) - number_scans:]
        
        elif self.SIDE == -1:
            angle_min = angle_min
            angle_max = angle_min + (number_scans - 1)*laser_msg.angle_increment
            r = ranges[:number_scans]

        theta = np.arange(angle_min, angle_max + 1e-10, laser_msg.angle_increment)
        
        range_min = laser_msg.range_min
        multiplier = 2
        while len(r[r <= multiplier * self.DESIRED_DISTANCE]) == 0:
            multiplier += .5
        
        
        range_max = self.DESIRED_DISTANCE * multiplier
        
        theta = theta[r >= range_min]
        r = r[r >= range_min]
        theta = theta[r <= range_max]
        r = r[r <= range_max]        
        
        # Convert to cartesian
        x_vals = np.multiply(r, np.cos(theta))
        y_vals = np.multiply(r, np.sin(theta))

        #rs = np.array(laser_msg.ranges[self.min_ind:self.max_ind])
        # convert to rectangular coords to pass into least squares
        #A = np.vstack([np.array([rs[i]*np.cos(ths[i]) for i in range(len(rs))]), np.ones(self.max_ind-self.min_ind)]).T
        A = np.vstack([x_vals, np.ones(len(x_vals))]).T
        #y = np.array([rs[i]*np.sin(ths[i]) for i in range(len(rs))])
        # get line approximating wall
        m, b = np.linalg.lstsq(A, y_vals)[0]
        thw = -self.SIDE*np.arctan(m)

        b_prime = b - self.SIDE*self.DESIRED_DISTANCE/np.cos(-thw)

        l_d = self.VELOCITY*self.K_dd
        K_dd = self.K_dd
        
        p = -self.L
        A = m**2 + 1
        B = m*b_prime - p
        C = -(l_d**2) + (p**2) + (b_prime**2)
    
        while (B**2 - 4*A*C) < 0:
            K_dd += .1
            l_d = self.VELOCITY*K_dd
            C = -(l_d**2) + (p**2) + (b_prime**2)

        if True:
            t_x = (-B + np.sqrt(B**2 - 4*A*C))/(2*A)
            t_y = m*t_x + b_prime
            
            
            alpha = np.arctan2(t_y, t_x + self.L)
            
            delta = np.arctan2(2*self.L*np.sin(alpha), l_d)
        
        rospy.loginfo(str(delta))
        self.drive_msg.drive.steering_angle = delta
        self.drive_pub.publish(self.drive_msg)

if __name__ == "__main__":
    rospy.init_node('wall_follower')
    wall_follower = WallFollower()
    rospy.spin()    
