#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import imutils
import numpy as np
import pdb
from lab4.msg import cone_location
from computer_vision import color_segmentation
from visualization_msgs.msg import Marker

class ConeDetector():
    """
    Takes in camera feed to detect cone and figure out its relative position to the camera/bot.
    """
    def __init__(self):
        
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/relative_cone", cone_location, queue_size=10)
        self.message_x = None
        self.message_y = None
        self.message_frame = "base_link"
        self.marker_pub = rospy.Publisher("/cone_marker",
                                          Marker, queue_size=10)
        rospy.Subscriber("/zed/zed_node/rgb/image_rect_color", Image, self.detection_callback)

        self.image_pub = rospy.Publisher("/cone_image",Image, queue_size=10)
        
    def detection_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)
        
        ### Do segmentation stuff. ###

        # crop = color_segmentation.lookahead(cv_image)
        crop = cv_image

        def image_pub(cv_image):
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))

        (min_x, min_y), (max_x, max_y) = color_segmentation.cd_color_segmentation(crop, None, image_pub=image_pub)

        ave = lambda x, y: (x + y) / 2
        # bottom_of_cone = [ave(min_x, max_x), max_y, 1]
        pixel_vector = bottom_of_cone

        middle_of_cone = [ave(min_x, max_x), ave(min_y, max_y), 1]
        pixel_vector = middle_of_cone # hack because we're getting the reflection as well as the cone
        
        ##############################
        
        position_vector = self.pixel_to_coord(pixel_vector)
        
        cone_location_msg = cone_location()
        
        cone_location_msg.x_pos, cone_location_msg.y_pos = position_vector[0], position_vector[1]
        self.message_x, self.message_y = cone_location_msg.x_pos, cone_location_msg.y_pos
        self.pub.publish(cone_location_msg)
        self.draw_marker()

        # now drawing inside of color_segmentation
        # cv2.rectangle(cv_image, (min_x, min_y), (max_x, max_y), (0,0,255), 2)
        # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
     
    def pixel_to_coord(self, v):
        """
        Takes in v, a 3 x 1 numpy array of the form [u, v, 1] where u and v are pixel coordinates.
        Returns [x, y, 1] --> Homogeneous array of corresponding location in world
        """
        H = np.array([[ -4.18127974e-05,   1.30031812e-03,  -6.24127179e-01,],
                      [  1.19511722e-03,  -2.56099802e-04,  -3.64493354e-01,],
                      [  1.12374426e-04,  -6.02413608e-03,   1.00000000e+00]])
        return np.dot(H,v)/np.dot(H,v)[-1]
        
    def draw_marker(self):
        """
        Publish a marker to represent the cone in rviz
        """
        marker = Marker()
        marker.header.frame_id = self.message_frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = self.message_x
        marker.pose.position.y = self.message_y
        self.marker_pub.publish(marker)
                
        
        
if __name__ == '__main__':
    try:
        rospy.init_node('ConeDetector', anonymous=True)
        ConeDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
