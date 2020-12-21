#!/usr/bin/env python

import numpy as np
import time
from threading import RLock

import rospy
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped, PoseWithCovarianceStamped, PointStamped, Point
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from nav_msgs.srv import GetMap
import tf
import tf.transformations as transformations



class ObstacleDetection(object):
    '''
    Node that detects and plots obstacles on a new OccupancyGrid map.
    '''
    def __init__(self):
        rospy.loginfo(" initialized!")
        
        # Get parameters
        self.particle_filter_frame = "/hood_lidar"
        self.base_link_frame = "base_link_gt"
        
        # Additional tunable hyperparameter
        self.viable_angle_range = np.pi/6. # Amount away from theta = 0 that will be considered
        self.in_range = 50. # Number of meters away for a obstacle to be considered
        
        # Subscribers, publishers, listeners, and broadcasters
        self.listener = tf.TransformListener()
        self.obs_pub = rospy.Publisher("/obstacles", OccupancyGrid, queue_size=1)
        self.road_map = None
        
        if self.road_map is None:
            self.road_sub = rospy.Subscriber("/map", OccupancyGrid, self.road_cb, queue_size=1)    
        self.laser_sub = rospy.Subscriber(rospy.get_param("~scan_topic", "/tesse/hood_lidar/scan"), LaserScan, self.lidar_cb, queue_size=1)
        
        # For testing
        self.vis_pub = rospy.Publisher("/particles", Marker, queue_size = 1)
        
            
        
    def lidar_cb(self, msg):
        rospy.loginfo("starting lidar cb")
        if self.road_map is None:
            return

        rospy.loginfo("got map")
        ranges = np.array(msg.ranges)
        num_ranges = len(ranges)
        angles = (np.ones(num_ranges) * msg.angle_min) + (np.arange(num_ranges) * msg.angle_increment)
        
        # Discard extraneous data (< range_min or > range_max)
        angles = angles[ranges > msg.range_min]
        ranges = ranges[ranges > msg.range_min]
        angles = angles[ranges < msg.range_max]
        ranges = ranges[ranges < msg.range_max]
        
        # Remove data points not in the viable range in front of the car, ie theta E (-viable_angle_range, viable_angle_range)
        ranges = ranges[angles > -self.viable_angle_range]
        angles = angles[angles > -self.viable_angle_range]
        ranges = ranges[angles < self.viable_angle_range]
        angles = angles[angles < self.viable_angle_range]
        
        # Remove data points outside self.in_range
        angles = angles[ranges < self.in_range]
        ranges = ranges[ranges < self.in_range]
        
        # Convert to cartesian, LIDAR-frame positions
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        new_num_ranges = len(x)
        arr = np.vstack((x, y))
        
        # Find transform from LIDAR frame to map frame
        try:
            (trans,rot) = self.listener.lookupTransform(msg.header.frame_id, self.road_map.frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rate.sleep()
            return
            
        # Convert translation + quaternion to homogeneous transformation matrix
        T_lidar_map = transformations.quaternion_matrix(rot)
        T_lidar_map = T_lidar_map.T
        TEMP = transformations.quaternion_matrix(rot)
        TEMP = np.linalg.inv(TEMP[:3, :3])
        T_lidar_map[:3, 3] = np.matmul(TEMP, -np.array(trans).T)
        #T_lidar_map = T_lidar_map.T
        
        # Convert LIDAR points from LIDAR frame to map frame
        arr_LIDAR = np.vstack((x, y, np.zeros(new_num_ranges), np.ones(new_num_ranges)))
        arr_MAP = np.matmul(T_lidar_map, arr_LIDAR)
        print(T_lidar_map)
        arr_MAP = arr_MAP[:2]
        
        x_map = arr_MAP[0]
        y_map = arr_MAP[1]
        
        # Find which points are on the roads
        on_road = self.road_map.check_free(arr_MAP)
        
        x_obs = x_map[on_road]
        y_obs = y_map[on_road]
        """        
        # Find occupancy grid matrix with detected obstacles filled in
        new_occ_matrix = self.road_map.add_obstructions(np.vstack((x_obs, y_obs)))
        
        
        # Creates and publishes OccupancyGrid message with obstacles to /obstacles
        obs_msg = OccupancyGrid()
        
        # now = rospy.get_time()
        # obs_msg.header.stamp = now
        obs_msg.header.frame_id = self.road_map.frame
        
        obs_msg.info = self.road_map.msg.info
        # obs_msg.info.map_load_time = rospy.Time(0) # Probably unnecessary?
        
        obs_msg.data = np.ravel(new_occ_matrix)
        rospy.loginfo("publishing")
        
        self.obs_pub.publish(obs_msg)
        
        print("test")
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.POINTS
        marker.action = marker.ADD
        
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        for i in range(len(x_obs)):
            TEMP = Point()
            TEMP.x = x_obs[i]
            TEMP.y = y_obs[i]
            TEMP.z = .05
            marker.points.append(TEMP)
        
            TEMP = ColorRGBA()
            TEMP.r = 1
            TEMP.g = 0
            TEMP.b = 0
            TEMP.a = 1
            
            marker.colors.append(TEMP)
        
        self.vis_pub.publish(marker)
        
        
        
        
        
    
    def road_cb(self, msg):
        if self.road_map is not None:
            return
        else:
            # Make self.road_map a map of the roads
            self.road_map = Map(msg)
            return
        





class Map:
    def __init__(self, msg):
        """
        Converts occupancy grid message to an actual map.
        
        Args:
            msg: OccupancyGrid, converted to a more usable map.
        """
        self.msg = msg
        
        self.frame = msg.header.frame_id
        self.width, self.height, self.resolution = msg.info.width, msg.info.height, msg.info.resolution # width and height are integer numbers of cells, and resolution is in m/cell
        self.occ_matrix = np.reshape(np.array(msg.data), (self.height, self.width))
        self.origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.origin_angle = np.arctan2(msg.info.origin.orientation.z, msg.info.origin.orientation.w)*2.
        self.rot = np.array([[np.cos(self.origin_angle), -np.sin(self.origin_angle)],
                             [np.sin(self.origin_angle), np.cos(self.origin_angle)]])
        self.rot_inv = np.array([[np.cos(-self.origin_angle), -np.sin(-self.origin_angle)],
                                 [np.sin(-self.origin_angle), np.cos(-self.origin_angle)]])
        
        # Final all (m, n) pairs of cell coordinates that have value 0 (i.e. free space)
        m_cells, n_cells = np.where(self.occ_matrix == 0)
        self.free_cells = np.vstack((m_cells, n_cells)).T # f x 2 array           
        
    def cell_to_cartesian(self, arr):
        """
        
        """
        cart_coords = np.dot(self.rot, arr[::-1] * self.resolution) + self.origin
        return cart_coords
        
    def cartesian_to_cell(self, arr):
        """
        
        """
        cell_coords = (np.dot(self.rot_inv, (arr.T - self.origin).T))/self.resolution
        cell_coords = cell_coords[:][::-1]
        return cell_coords.astype("int")        
        
    def check_free(self, arr):
        """
        Check to see if a given cartesian coordinate is in free space.
        
        Args:
            arr: np.array of cartesian coordinate pairs, representing where to check. Can either do np.array([x,y]) or np.array([[x1, x2, x3 ...], [y1, y2, y3 ...]])
        
        Returns:
            np.array of bools
        """
        cell_location = self.cartesian_to_cell(arr)
        cell = self.occ_matrix[cell_location[0], cell_location[1]]
        return (cell == 0)

    def add_obstructions(self, arr):
        """
        Adds obstructed points to the occupancy matrix.
        
        Args:
            arr: np.array of cartesian coordinate pairs, representing obstruction locations. Can either do np.array([x,y]) or np.array([[x1, x2, x3 ...], [y1, y2, y3 ...]])
            
        Returns:
            np.array that is identical to self.occ_matrix except with the above points' corresponding pixels filled in as "occupied" (i.e. pixel value = 100)
        """
        #res = np.copy(self.occ_matrix)
        res = np.zeros((self.height, self.width))
        indices = self.cartesian_to_cell(arr)
        res[indices[0], indices[1]] = 100
        return res
        
        
        
        
        
        
        
        
        


if __name__ == "__main__":
    rospy.init_node("obstacle_detection")
    od = ObstacleDetection()
    rospy.spin()
