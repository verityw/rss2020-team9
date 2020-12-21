#!/usr/bin/env python2

import rospy
from rospy.rostime import Time
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from std_msgs.msg import Header


class Safety:
    SCAN_TOPIC = rospy.get_param("safety/scan_topic")
    NAVIGATION_TOPIC = rospy.get_param("safety/navigation_topic")
    SAFETY_TOPIC = rospy.get_param("safety/safety_topic")
    ANGLE_RANGE = np.pi / 3 # angle range to monitor for obstacles
    N_SECTIONS = 50 # should be large
    CLOSE_THRESHOLD = 0.2 # crashing distance in meters
    FWD_CLOSE = 0.3 # seconds to brake

    def __init__(self):
        self.safety_pub = rospy.Publisher(
            self.SAFETY_TOPIC,
            AckermannDriveStamped,
            queue_size=10,
        )
        self.last_steering_angle = 0
        self.last_velocity = 0
        self.listen()

    def listen(self):
        rospy.Subscriber(
            self.SCAN_TOPIC,
            LaserScan,
            self.safety,
        )
        rospy.Subscriber(
            self.NAVIGATION_TOPIC,
            AckermannDriveStamped,
            self.update_drive_command,
        )

    def safety(self, scan_data):
        '''
        Look forward
        '''
        def extract_range_index(angle):
            '''
            in: angle should be between angle_min, angle_max
            (implicitly: scan_data)
            out: index
            '''
            index = int(round((angle - scan_data.angle_min) / scan_data.angle_increment))
            if index < 0:
                return 0 # usually doesn't happen
            elif index >= len(scan_data.ranges):
                return len(scan_data.ranges) - 1 # usually doesn't happen
            else:
                return index

        ranges = np.array(scan_data.ranges)
        # assume scan centered at 0
        min_index = extract_range_index(-self.ANGLE_RANGE)
        max_index = extract_range_index(self.ANGLE_RANGE)
        num_indices_per_section = (max_index - min_index) / self.N_SECTIONS # type int
	angle_per_section = (2.0*self.ANGLE_RANGE)/self.N_SECTIONS
	
        '''
        # wrong code
        split = np.split(ranges, num_indices_per_section)
        if np.any(np.mean(split, axis=1) < self.CLOSE_THRESHOLD):
            self.publish_stop_message()
        '''
        for i in range(self.N_SECTIONS):
            start_i = i * num_indices_per_section + min_index
	    angle = -self.ANGLE_RANGE + i*angle_per_section
            ave_dist = np.mean(ranges[start_i: start_i + num_indices_per_section])
            if ave_dist < self.CLOSE_THRESHOLD + self.FWD_CLOSE  * np.cos(angle) * self.last_velocity:
                self.publish_stop_message()
                return # don't publish more than one message

    def update_drive_command(self, data):
        self.last_steering_angle = data.drive.steering_angle
        self.last_velocity = data.drive.speed
        
    def publish_stop_message(self):
        header = Header(
            stamp=Time.now(),
            frame_id="base_link",
        )
        drive = AckermannDrive(
            steering_angle=self.last_steering_angle,
            steering_angle_velocity=0,
            speed=0,  # STOP!
            acceleration=0,
            jerk=0,
        )
        msg = AckermannDriveStamped(
            header=header,
            drive=drive,
        )
        rospy.logwarn("STOP: safety controller ordered a stop!")
        self.safety_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node('safety')
    safety = Safety()
    rospy.spin()
