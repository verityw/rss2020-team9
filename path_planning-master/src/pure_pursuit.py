#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

# Limiting factor seems to be bad localization 
MAX_SPEED = 22.0 
SLOWDOWN_SPEED = 12.0
# long straight shots that go fast 
MAX_KDD = 1.1 
# for corners bc lookahead = speed*kdd
MIN_KDD = 1.25
ANGLE_SCALE_MAX = 0.032
ANGLE_SCALE_MIN = 0.055
SMALL_SEG = 0.272 #6m

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
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
        rospy.loginfo("pure pursuit waiting for initialpose")

        #rviz crap
        self.do_rviz = True 
        self.nearest_pub = rospy.Publisher("/target_pt", Marker, queue_size=1)

    def init_cb(self, msg):
        rospy.loginfo("pure pursuit got initial pose!")
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
        

        self.num_pts = len(self.trajectory.np_points)
        
        # perhaps make list of squared distances bw points in trajectory
        # this is needed for the closest line segment thingy and is independent of the position of the robot 
        # maybe efficiency is improved by only calculating this once per time that the trajectory is given
        def next_pt(i):
            return (i+1)%self.num_pts

        # n entries, one for each line segment + connect the last to the first point
        self.trajectory_dists = np.array([np.linalg.norm(self.trajectory.np_points[next_pt(i)]-self.trajectory.np_points[i])**2 
                                          for i in range(self.num_pts)])

        # also vectors bw points can be calculated once only
        self.wv = np.array([self.trajectory.np_points[next_pt(i)]-self.trajectory.np_points[i] for i in range(self.num_pts)])


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
            #last point got extended
            last_pt = self.num_pts
        
        if self.prev_seg is not None:
            t_d = self.trajectory_dists[self.prev_seg:self.prev_seg+5]
            wv = self.wv[self.prev_seg:self.prev_seg+5]
            pv = pose_np - self.trajectory.np_points[self.prev_seg:self.prev_seg+5]
            t = np.maximum(0, np.minimum(1, (pv*wv).sum(axis=-1)/t_d))
            proj = self.trajectory.np_points[self.prev_seg:self.prev_seg+5] + wv*t[:, np.newaxis]
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
        seg, pt = self.get_closest_seg(np_pose_now)
        self.prev_seg = seg
    
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

            #rospy.loginfo("Found expanded disc")
            
            while t1 <= 0:
                # if segment extended enough backwards it would touch, so we shouldn't move to
                # next segment, increase lookahead again
                disc = self.adjust_kdd(A, B, p1, q)
                t1 = (-B + np.sqrt(disc))/(2* A)
                
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
                target_viz.x = target_pos[0]
                target_viz.y = target_pos[1]
                target_viz.z = .05
                marker.colors.append(ColorRGBA(r=1, g=0, b=0, a=1))
                marker.points.append(target_viz)

                self.nearest_pub.publish(marker)
            break
        if not self.stop:
            seg_len = np.sqrt(self.trajectory_dists[seg]) 
            #if seg_len >= 2:
            #    self.kdd = 1.5
            if seg_len <= MAX_SPEED*SMALL_SEG:
                self.kdd = MIN_KDD
                self.speed = SLOWDOWN_SPEED
                scale = ANGLE_SCALE_MIN
            
            # convert to robot frame
            target_coord = get_robot_frame_coord((target_pos, target_theta), (np_pose_now, theta_now))
        
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
    
def get_robot_frame_coord(target_pose, current_pose):
    target_pos, target_theta = target_pose
    np_pose_now, theta_now = current_pose

    diff = target_pos-np_pose_now
    rot = np.array([[np.cos(theta_now), -np.sin(theta_now)],
                    [np.sin(theta_now), np.cos(theta_now)]])
    # target pos in frame of robot
    return np.dot(rot.T, diff.T)        

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
