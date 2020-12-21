#!/usr/bin/env python
import rospy
from lab4.msg import cone_location, parking_error
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np

class ParkingController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        rospy.Subscriber("/relative_cone", cone_location, 
            self.relative_cone_callback)    
        self.drive_pub = rospy.Publisher("/drive", 
            AckermannDriveStamped, queue_size=10)
        self.error_pub = rospy.Publisher("/parking_error",
            parking_error, queue_size=10)

        self.relative_x = 0
        self.relative_y = 0
        
        self.STATE = "PARK"
        
        ########### OTHER REQUIRED PARAMETERS ###########
        self.L = .325 # Length of car in meters
        self.L_LB = self.L # Length from LIDAR to back axle
        
        
    def relative_cone_callback(self, msg):
        self.relative_x = msg.x_pos
        self.relative_y = msg.y_pos
        drive_cmd = AckermannDriveStamped()

        # Play with this number too
        parking_distance = 2 #meters
        angle_bound = np.pi/8. #radians
        dist_bound = .05 #meters
        backup_multiplier = 1.5 #Distance to back up to
        
        K_v = 2
        K_d = 2
        
        dist = np.sqrt((self.relative_x**2) + (self.relative_y**2))
        
        rospy.loginfo(self.STATE)
        
        if self.STATE == "PARK":
            velocity = max(min(1, (K_v*(dist - parking_distance))), -1)
            
            L_1 = max(K_d*velocity, self.L)
            
            front_or_back = np.sign(self.relative_x)
            
            m = self.relative_y/self.relative_x
            
            angle_to_cone = np.arctan2(self.relative_y, self.relative_x)
            
            A = (1 + m**2)
            B = 2*self.L
            C = (self.L**2 - L_1**2)
            
            t_x = (-B + front_or_back * np.sqrt(B**2 - 4*A*C))/(2*A)
            t_y = m*t_x
            theta = np.arctan2(t_y, t_x + self.L)
            
            delta = np.arctan2(2*self.L*np.sin(theta), L_1)
            
            drive_cmd.drive.speed = velocity
            drive_cmd.drive.steering_angle = delta*np.sign(velocity)
            
            
            
            if (abs(dist - parking_distance) < dist_bound) and abs(angle_to_cone) > angle_bound:
                self.STATE = "BACK UP"
            
            self.drive_pub.publish(drive_cmd)
            self.error_publisher()
        elif self.STATE == "BACK UP":
            velocity = max(min(1, (K_v*(dist - parking_distance*backup_multiplier))),-1)
            
            L_1 = max(K_d*velocity, self.L)
            
            front_or_back = np.sign(self.relative_x)
            
            m = self.relative_y/self.relative_x
            
            A = (1 + m**2)
            B = 2*self.L
            C = (self.L**2 - L_1**2)
            
            t_x = (-B - front_or_back * np.sqrt(B**2 - 4*A*C))/(2*A)
            t_y = m*t_x
            theta = np.arctan2(t_y, t_x + self.L)
            
            delta = np.arctan2(2*self.L*np.sin(theta), L_1)
            
            drive_cmd.drive.speed = velocity
            rospy.loginfo(velocity)
            drive_cmd.drive.steering_angle = delta
            
            if (abs(dist - parking_distance*backup_multiplier) < dist_bound):
                self.STATE = "PARK"
            
            self.drive_pub.publish(drive_cmd)
            self.error_publisher()

            
            
            
            
        
    def error_publisher(self):
        """
        Publish the error between the car and the cone. We will view this
        with rqt_plot to plot the success of the controller
        """
        
        # Might need to subtract desired distance from error???
        
        error_msg = parking_error()
        
        error_msg.x_error = self.relative_x
        error_msg.y_error = self.relative_y
        error_msg.distance_error = np.sqrt((self.relative_x**2) + (self.relative_y**2))
        
        self.error_pub.publish(error_msg)

if __name__ == '__main__':
    try:
        rospy.init_node('ParkingController', anonymous=True)
        ParkingController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
