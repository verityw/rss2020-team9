#!/usr/bin/env python

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Pose, PoseStamped, PoseArray, Quaternion, PolygonStamped, PoseWithCovarianceStamped, PointStamped, Point
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from nav_msgs.srv import GetMap
from scipy.ndimage.filters import minimum_filter1d, maximum_filter1d, median_filter
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from threading import RLock
from visualization_msgs.msg import Marker
import numpy as np
import rospy
import tf
import tf.transformations as transformations
import time
import utils

# Limiting factor seems to be bad localization 
MAX_SPEED =8 
SLOWDOWN_SPEED = 8
# long straight shots that go fast 
MAX_KDD = 1.72 
# for ??? bc lookahead = speed*kdd
MIN_KDD = 1.72
ANGLE_SCALE_MAX = 0.06
ANGLE_SCALE_MIN = 0.06
SMALL_SEG = 0.272 #6m

SHARP_TURN = np.radians(20.0)
SMOOTHING_FACTOR = 1
# how many meters further than lookahead pt to consider obstacles from
# might have to be less than 1.71m , probably.. to ensure nonnegative ranges + pip working
# DO NO T CHANGE
MARGINAL_REACT_DIST = 1.71

HOOD_LIDAR_FROM_BASE_LINK_X = 1.71 # manually obtained from inspecting rviz

ANGLE_DILATION = np.pi/16.0
STEERING_ANGLE = np.radians(20.0)

class ObstacleDetectionPurePursuit(object):
    '''
    Node that detects obstacles and uses them to adjust target points during pure pursuit.
    '''
    def __init__(self):
        ################################
        # Begin ObstacleDetection init #
        ################################
        
        # Additional tunable hyperparameter
        self.viable_angle_range = np.pi/6. # Amount away from theta = 0 that will be considered
        self.in_range = 50. # Number of meters away for a obstacle to be considered
        
        self.laser_sub = rospy.Subscriber(rospy.get_param("~scan_topic", "/tesse/hood_lidar/scan"), LaserScan, self.lidar_cb, queue_size=1)

        # For testing
        self.vis_pub = rospy.Publisher("/particles", Marker, queue_size = 1)


        self.lidar_pts = None
        ############################
        # # Begin PurePursuit init #
        ############################
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.kdd              = MAX_KDD
        self.speed            = MAX_SPEED # FILL IN #
        self.lookahead        = self.kdd*self.speed
        # if wrap is false, self.trajectory.np_points will have an extra pt, but self.trajectory.points won't

        self.wrap             = False # FILL IN #
        self.stop             = False
        self.wheelbase_length = 3.0#0.325 FILL IN #
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/tesse/drive", AckermannDriveStamped, queue_size=1)
        # change value to self.odom_topic later
        self.pose_sub = rospy.Subscriber(self.odom_topic, Odometry, self.pursuit_callback, queue_size=1)
        self.num_pts = None
        self.prev_seg = None

        # initial pose waiting
        self.initialpose = True
        self.init_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.init_cb, queue_size=1)
        rospy.loginfo("Pure pursuit Waiting 10 seconds for initial pose")
        time.sleep(10.0)

        #rviz crap
        self.do_rviz = True 
        self.nearest_pub = rospy.Publisher("/target_pt", Marker, queue_size=1)
        self.closest_pub = rospy.Publisher("/close_pt", Marker, queue_size=1)

        

    def init_cb(self, msg):
        self.initialpose = True

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)
        # make numpy array of trajectory points (nx2) so that hopefully we can make things go fast
        self.trajectory.make_np_array()

        if not self.wrap:
            last_pt = self.trajectory.np_points[-1]
            second_last_pt = self.trajectory.np_points[-2]
            last_seg = last_pt - second_last_pt
            extra = last_pt + last_seg
            self.trajectory.np_points = np.append(self.trajectory.np_points, [extra], axis=0)
        

        
        # perhaps make list of squared distances bw points in trajectory
        # this is needed for the closest line segment thingy and is independent of the position of the robot 
        # maybe efficiency is improved by only calculating this once per time that the trajectory is given
        def next_pt(i):
            return (i+1)%len(self.trajectory.np_points)

        self.num_pts = len(self.trajectory.np_points)

        # n entries, one for each line segment + connect the last to the first point
        self.trajectory_dists = np.array([np.linalg.norm(self.trajectory.np_points[next_pt(i)]-self.trajectory.np_points[i])**2 
                                          for i in range(self.num_pts)])

        # also vectors bw points can be calculated once only
        if self.wrap:
            self.wv = np.array([self.trajectory.np_points[next_pt(i)]-self.trajectory.np_points[i] for i in range(self.num_pts)])
        else:
            self.wv = np.array([self.trajectory.np_points[next_pt(i)]-self.trajectory.np_points[i] for i in range(self.num_pts-1)])
            


    def get_closest_seg(self, pose_np):
        """
        given 1x2 numpy pose array, find closest segment from nx2 trajectory.np_points
        return 2 points from line segment (?) 
        may need to be optimized

        let the point the robot is at be p, and for each line segment the start and end points be v and w respectively
        TODO: keep track of previous segment and don't look for segment very far ahead
        """
        if self.wrap:
            #last pt leads to first 
            last_pt = self.num_pts
        else:
            last_pt = self.num_pts-1
        
        if self.prev_seg is not None:
            lookahead_seg = min(self.prev_seg+5, last_pt)
            t_d = self.trajectory_dists[self.prev_seg:lookahead_seg]
            wv = self.wv[self.prev_seg:lookahead_seg]
            pv = pose_np - self.trajectory.np_points[self.prev_seg:lookahead_seg]
            t = np.maximum(0, np.minimum(1, (pv*wv).sum(axis=-1)/t_d))
            proj = self.trajectory.np_points[self.prev_seg:lookahead_seg] + wv*t[:, np.newaxis]
            # return index of minimum distance, and closest pt 
            minseg = np.argmin(np.linalg.norm(pose_np-proj, axis=1))
            return self.prev_seg+minseg, proj[minseg]
            

        pv = pose_np - self.trajectory.np_points[:last_pt] # self.num_pts x 2

        # projecting p onto line bw vw, constraining b/w 0 and 1 to keep within line segment
        t = np.maximum(0, np.minimum(1, (pv*self.wv[:last_pt]).sum(axis=-1)/self.trajectory_dists[:last_pt]))
        proj = self.trajectory.np_points[:last_pt] + self.wv[:last_pt]*t[:, np.newaxis]
        # return index of minimum distance, and closest pt 
        minseg = np.argmin(np.linalg.norm(pose_np-proj, axis=1))
        return minseg, proj[minseg]

    def adjust_kdd(self, A, B, p1, q):
        self.kdd += 0.1
        self.lookahead = self.kdd*self.speed
        disc = B**2 - 4*A*(np.dot(p1,p1) + np.dot(q,q) - np.dot(2*p1,q) - self.lookahead**2)
        return disc

    def pursuit_callback(self, msg):
        """
        given pose:
            1) find closest line segment from trajectory
            2) pure pursuit
            3) ???
            4) profit (set angle of ackermann message and publish)
        """
        if not self.initialpose:
            return
        if not self.num_pts:
            return
        #get pose
        np_pose_now = np.array([[msg.pose.pose.position.x, msg.pose.pose.position.y]])
        theta_now = np.arctan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)*2
        # now we have x,y,theta in world frame
        seg, close_pt = self.get_closest_seg(np_pose_now)
        self.prev_seg = seg
        if self.do_rviz:
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.POINTS
            marker.action = marker.ADD
            
            marker.points = []
            marker.colors = []

            marker.scale.x = 0.5
            marker.scale.y = 0.5

            target_viz = Point()
            target_viz.x = close_pt[0]
            target_viz.y = close_pt[1]
            target_viz.z = .05
            marker.colors.append(ColorRGBA(r=0, g=0, b=1, a=1))
            marker.points.append(target_viz)

            self.closest_pub.publish(marker)
    
        if not self.wrap and seg == self.num_pts-2:
            self.speed = 0
            self.stop = True
    
        ## FINDING TARGET POINT ##
        ## circle intersecting line segment stuff using quadratics ##
        # everything so far in the world frame
        def next_seg(seg):
            return (seg+1)%self.num_pts

        q = np_pose_now.reshape((2,))
        # initial line seg to check bw
        # p1 = pt.reshape((2,)) # dont check behind car
        p1 = self.trajectory.np_points[seg]
        p2 = self.trajectory.np_points[next_seg(seg)]
        
        self.kdd = MAX_KDD
        self.speed = MAX_SPEED
        scale = ANGLE_SCALE_MAX

        while not self.stop:
            seg_len = np.sqrt(self.trajectory_dists[seg]) 
            #if seg_len >= 2:
            #    self.kdd = 1.5
            if seg_len <= MAX_SPEED*SMALL_SEG:
                self.kdd = MIN_KDD
                self.speed = SLOWDOWN_SPEED
                scale = ANGLE_SCALE_MIN

            self.lookahead = self.speed*self.kdd

            vec = p2-p1
            A = np.dot(vec, vec)
            B = 2*(np.dot(vec, p1 - q))
            C = np.dot(p1, p1) + np.dot(q,q) - np.dot(2*p1,q) - self.lookahead**2
            disc = B**2 - 4*A*C
            
            while disc <= 0:
                # no intersection
                # extend lookahead distance
                disc = self.adjust_kdd(A, B, p1, q)
                #rospy.loginfo("Point: " + str(p1) + str(p2))
                #rospy.loginfo(str(disc))
            t1 = (-B + np.sqrt(disc))/(2*A)
            t2 = (-B - np.sqrt(disc))/(2*A)

            #rospy.loginfo("Found expanded disc")
            
            while t1 <= 0:
                # if segment extended enough backwards it would touch, so we shouldn't move to
                # next segment, increase lookahead again
                disc = self.adjust_kdd(A, B, p1, q)
                t1 = (-B + np.sqrt(disc))/(2* A)
                t2 = (-B - np.sqrt(disc))/(2*A)
                
            if t1 >= 1:
                # also no intersection, but bc segment is not long enough
                # here we should probably do the moving on to next line segment
                # start point of next segment is endpoint of this segment
                
                seg = (seg+1)%self.num_pts 
                p1 = p2
                p2 = self.trajectory.np_points[next_seg(seg)]
                continue 

            #else:
            # we have intersection! yay
            target_pos = p1 + t1*vec # position on line segment if 0<=t1<=1
            target_theta = np.arctan2(p1[1]-p2[1], p1[0]-p2[0]) # direction of line segment
            break
        if not self.stop:
            
            # convert to robot frame
            target_coord = get_robot_frame_coord((target_pos, target_theta), (np_pose_now, theta_now))

            target_coord = self.adjust_target_coord_for_obstacles(target_coord, STEERING_ANGLE)
            if self.do_rviz:
                marker = Marker()
                marker.header.frame_id = "/base_link"
                marker.type = marker.POINTS
                marker.action = marker.ADD
                
                marker.points = []
                marker.colors = []

                marker.scale.x = 0.5
                marker.scale.y = 0.5

                target_viz = Point()
                target_viz.x = target_coord[0]
                target_viz.y = target_coord[1]
                target_viz.z = .05
                marker.colors.append(ColorRGBA(r=1, g=0, b=0, a=1))
                marker.points.append(target_viz)

                self.nearest_pub.publish(marker)
            rospy.loginfo("TARGET: " + str(target_coord))
            target_th = np.arctan2(target_coord[1],target_coord[0])

            if abs(target_th) > SHARP_TURN:
                self.kdd = MIN_KDD
                self.speed = SLOWDOWN_SPEED
                scale = ANGLE_SCALE_MIN
        
            # find turning radius using method in that paper linked in piazza 
            R_track = self.lookahead**2/(2*abs(target_coord[1]))
        
            # now get desired turning angle!
            delta = np.sign(target_coord[1])*np.arctan2(self.wheelbase_length, R_track)
       
        if self.stop:
            delta = 0

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = msg.header.stamp
        drive_msg.drive.speed = self.speed 
        drive_msg.drive.steering_angle = -1*scale*delta #NEEDS TO BE SET
        self.drive_pub.publish(drive_msg) 

    def adjust_target_coord_for_obstacles(self, target_coord, steering_angle):
        """
        figure out if there are obstacles in between car and target_coord, adjust coord to avoid crashes
        """
        if not self.point_in_polygon(target_coord):
            rospy.loginfo("adjust for obstacle")
            # target point is past obstacles, need to reroute
            target_th = np.arctan2(target_coord[1], target_coord[0])
            target_dist = np.linalg.norm(target_coord)
            remaining_rs = self.rs#[np.where(steering_angle+target_th > self.ths > target_th-steering_angle)]        
            remaining_ths = self.ths#[np.where(steering_angle+target_th > self.ths > target_th-steering_angle)]
            out = np.vstack((self.rs, self.ths))

            # priority 1: avoid obstacles
            # priority 2: keep target pt
            
            remaining_rs = remaining_rs[np.where(remaining_rs > target_dist)]
            remaining_ths = remaining_ths[np.where(remaining_rs > target_dist)]
            if len(remaining_ths) == 0:
                return target_coord
            off_target_th = np.abs(remaining_ths-target_th)
            best_th = remaining_ths[np.argmin(off_target_th)]
            
            target_coord = [target_dist*np.cos(best_th), target_dist*np.sin(best_th)]
        return target_coord

    def point_in_polygon(self, pt):
        """
        adapted to python from PNPOLY
        returns true if point inside self.lidar_pts
        #TODO: vectorize 
        j = len(self.lidar_pts[1]-1)
        c = False
        for i in range(len(self.lidar_pts[1])):
            v = self.lidar_pts.T
            if (v[i][1] > pt[1]) is not (v[j][1] > pt[1]):
                if pt[0] < ((v[j][0] - v[i][0]) * (pt[1] - v[i][1]) / (v[j][1] - v[i][1]) + v[i][0]):
                    c = not c
            j = i
        return c
        """
        scan = self.lidar_pts.T
        scan = np.append(scan, [[0,0]], axis=0)
        scan = np.append(scan, scan[0:1], axis=0)
        current = scan[1:]
        prev = scan[:-1]
        ys_diff = np.not_equal(current[:,1] > pt[1], prev[:,1] > pt[1])
        xs_diff = pt[0] < (prev[:,0]-current[:,0])*(pt[1]-current[:,1]) / (prev[:,1]-current[:,1])+current[:,0]
        both_in = np.logical_and(ys_diff, xs_diff)
        odd_in = np.sum(both_in)%2
        return bool(odd_in)


        
    def lidar_cb(self, msg):
        ranges = np.array(msg.ranges)
        num_ranges = len(ranges)
        angles = (np.ones(num_ranges) * msg.angle_min) + (np.arange(num_ranges) * msg.angle_increment)

        # Before cleaning angles (guarantee that all ranges are contiguous)
        ranges = self.smooth_outliers(ranges)
        ranges = self.dilate_ranges(ranges, msg.angle_increment)
        angles, ranges = self.clean_angles_range(msg, angles, ranges)
        self.rs = ranges
        self.ths = angles

        # We want to just report all lidar data, so the following is commented out
        
        # Convert to cartesian, LIDAR-frame positions
        x = ranges * np.cos(angles)
        y = ranges * np.sin(angles)
        
        # Find transform from LIDAR frame to base_link frame
        # Assume it's aligned on the car in the y-direction
        x = x + HOOD_LIDAR_FROM_BASE_LINK_X-MARGINAL_REACT_DIST # example: 3 m lidar reading -> 4 m from base link
        
        lidar_pts = np.vstack((x, y))

        self.lidar_pts = lidar_pts
        return

    def smooth_outliers(self, ranges):
        return median_filter(ranges, size=SMOOTHING_FACTOR)
    
    def dilate_ranges(self, ranges, angle_increment):
        # unvectorized
        # i_dilation = int(round(self.angle_dilation / angle_increment))
        # for i in range(len(ranges)):
        #     dilated[i] = np.min(ranges[i-i_dilation:i+i_dilation])
        i_dilation = int(round(ANGLE_DILATION / angle_increment)) # one-sided
        i_dilation = i_dilation * 2 + 1 # two-sided
        dilated = minimum_filter1d(ranges, size=i_dilation)
        return dilated

    def dilate_coords(self, coords):
        dilated = np.zeros(coords.shape)
        


    def clean_angles_range(self, msg, angles, ranges):
        # Discard extraneous data (< range_min or > range_max or > in_range)
        assert msg.range_max > self.in_range
        angles = angles[self.in_range > ranges]
        ranges = ranges[self.in_range > ranges]
        angles = angles[ranges > msg.range_min]
        ranges = ranges[ranges > msg.range_min]
        
        # Remove data points not in the viable range in front of the car, ie theta E (-viable_angle_range, viable_angle_range)
        ranges = ranges[angles > -self.viable_angle_range]
        angles = angles[angles > -self.viable_angle_range]
        ranges = ranges[self.viable_angle_range > angles]
        angles = angles[self.viable_angle_range > angles]
        return angles, ranges


    def marker_stuff(self, x_obs, y_obs):
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
        
        


def get_robot_frame_coord(target_pose, current_pose):
    """
    get in robot frame from map frame
    """
    target_pos, target_theta = target_pose
    np_pose_now, theta_now = current_pose

    diff = target_pos-np_pose_now
    rot = np.array([[np.cos(theta_now), -np.sin(theta_now)],
                    [np.sin(theta_now), np.cos(theta_now)]])
    # target pos in frame of robot
    return np.dot(rot.T, diff.T)        


if __name__ == "__main__":
    rospy.init_node("obstacle_detection")
    od = ObstacleDetectionPurePursuit()
    rospy.spin()
