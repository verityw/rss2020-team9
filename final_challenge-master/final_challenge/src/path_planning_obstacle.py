#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from std_msgs.msg import ColorRGBA

import skimage.morphology as morph

from rtree import index

VERBOSE = False

class Node:
    def __init__(self, value, parent = None, cost = 0, depth = 0):
        '''
        Attrs:
        - value (np.array(float)): array representing cartesian coords of the node in the map frame
        - parent (Node): parent (may have been rewired)
        - children (list(Node)): child nodes
        - cost (float): heuristic -- the first-discovered cumulative path distance from start
        - depth (int): heuristic -- the depth (# nodes) of the first-discovered path from start
                       depth = 0 for starting node
        '''
        self.value = value # Numpy array representing configuration -- np.array([x, y]). We can change to accept np.array([x, y, theta]) later on
        self.parent = parent # Parent node
        # TODO: for more efficiency, could be a set
        self.children = [] # List of children nodes
        self.cost = cost        # Cost of reaching node from start
        self.depth = depth

class RRTStar:
    def __init__(self, start): #may need additional stuff here
        '''Representation for an RRT* search.

        root (Node): starting node for the search
        size (int): number of nodes in the search
        max_size (int): parameter -- maximum size of the search, above which we break the loop
        nodes (list(Node)): the nodes in the search.  len(nodes) == size
        idx (rtree.index.Index):  stores bounding boxes for every node

        For each i, nodes[i] is a Node with bounding box self.idx.bounds(i).
        '''
        self.root = start
        self.size = 1
        self.max_size = 10000
        self.nodes = [start]
        # TODO: could optimize by using the obj= parameter of Index()
        self.idx = index.Index()
        self.idx.insert(0, self.make_bounding_box(start.value))
    
    def make_bounding_box(self, arr, delta = .01):
        # creates a bounding box from (x, y), (xmin, ymin, xmax, ymax)
        return (arr[0] - delta, arr[1] - delta, arr[0] + delta, arr[1] + delta)
    
    def cost(self, start_value, end_value):
        """
        Calculate cost to connect start value to end value. Cost function can vary. 
        For [x, y], can use L2 distance
        For [x, y, theta], can use length of dubins curve connecting the two points
        
        For now, we will be using np.array([x, y]) and cost function L2 Euclidean distance.
        
        Args:
            start_value: np.array, starting configuration
            end_value: np.array, ending configuration
            
        Returns:
            cost: float, cost connecting start to end value.
        """
        cost = np.linalg.norm(start_value - end_value)
        return cost
        
    def add_config(self, parent_node, child_value):
        """
        Creates and adds a new child node to an existing parent node.

        Caller should check against self.max_size before calling this function.
        
        Args:
            parent_node: Node, preexisting parent node
            child_value: np.array, configuration of new child node
        Return:
            the new child node added to the RRT*
        """
        edge_cost = self.cost(parent_node.value, child_value)
        child_node = Node(
            child_value,
            parent=parent_node,
            cost=parent_node.cost + edge_cost,
            depth=parent_node.depth + 1
        )
        parent_node.children.append(child_node)
        self.nodes.append(child_node)
        # self.tree = KDTree(np.vstack((self.tree.data, np.array([child_value[0], child_value[1]]))))
        coords = child_value[:2] # child_value only has 2 coords (x, y) right now, but it may have theta in the future.
        self.idx.insert(self.size, self.make_bounding_box(coords))
        self.size += 1
        
        return child_node
   
    def nearest(self, value):
        """
        Finds the node that is closest (defined by the one with the least cost) to a given configuration.
        
        Args:
            value: np.array, configuration of considered node
        
        Returns:
            closest: Node, closest existing node to value
        """
        coords = value[:2] # value only has 2 coords (x, y) right now, but it may have theta in the future
        hits = self.idx.nearest(self.make_bounding_box(coords), 1, objects=False)
        for hit in hits:
            # take the first index in the event of any ties
            return self.nodes[hit]
        
        
        
        #assert that value is valid here
        """def recur(node, depth=0):
            closest, distance = node, self.cost(node.value, value)
            if depth < self.max_size:
                for child in node.children:
                    (child_closest, child_distance) = recur(child, depth+1)
                    if child_distance < distance:
                        closest = child_closest
                        distance = child_distance 
            return closest, distance
        return recur(self.root)[0]"""

    def nearest_in_n_sphere(self, value, r):
        """
        Finds the nodes in an n-dim (hyper)sphere in configuration space of radius r centered at the specified configuration.
        
        Args:
            value: np.array, representing robot's configuration
            r: float, radius of n-dimensional sphere to look within
        
        Returns:
            List of nodes in radius r of the value
        """
        return self.nearest_in_bounding_box(value, r)
        
        # This seems right
        # return self.binary_search_find_nearest_neighbors_in_radius(value, r)
        
        # This seems wrong
        # return self.recur_find_nearest_n_neighbor(value, r)

    def nearest_in_bounding_box(self, value, r):
        # Slightly more points than normal, possibly
        bounding_box = (value[0] - r, value[1] - r, value[0] + r, value[1] + r)
        hits = self.idx.intersection(bounding_box, objects=False)
        return [self.nodes[i] for i in hits]
        
    def binary_search_find_nearest_neighbors_in_radius(self, value, r):
        # this one seems all right
        k = 2
        max_found = False
        while not max_found:
            ind = list(self.idx.nearest(self.make_bounding_box(coords), k))
            k = k*2
            if len(ind) != 0:
                if VERBOSE:
                    print("woah")
                TEMP = self.nodes[ind[-1]]
                if abs(self.cost(TEMP.value, value)) > r:
                    max_found = True
                    if VERBOSE:
                        print("oof")
        TEMP = []
        for i in ind:
           TEMP.append(self.nodes[i])
        return TEMP
        
    def recur_find_nearest_n_neighbor(self, value, r):
        # seems wrong
        # assert that value is valid here
        # Recursively checks to see if each node is within the correct distance.
        def recur(node_list, node, depth=0):
            if self.cost(node.value, value) <= r:
                node_list.append(node)
            if depth < self.max_size:
                for child in node.children:
                    recur(node_list, child, depth+1)
            return
        node_list = []
        recur(node_list, self.root)
        return node_list

    def adoption(self, node, new_parent):
        """
        Changes the parent of a node in the tree graph.
        
        Args:
            node: Node, the adopted child.
            new_parent: Node, the adoptive parent.
            
        Returns:
            none
        """
        node.parent.children.remove(node) # Child protective services be like
        new_parent.children.append(node)  # Signing of adoption papers
        node.parent = new_parent
        node.depth = new_parent.depth + 1
        node.cost = node.parent.cost + self.cost(new_parent.value, node.value)
        # No need to recursively update heuristic values for children,
        # since it's just an heuristic
        #
        # def recur(considered_node):
        #     # TODO: really big unnecessary performance hit
        #     for children in considered_node.children:
        #         children.cost = considered_node.cost + self.cost(considered_node.value, children.value)
        #         children.depth = considered_node.depth + 1
        #         recur(children)
        #     return
        # recur(node)
    
    def radius(self, gamma, d):
        """
        Calculates radius FINISH THIS DESCRIPTION LATER WHEN INTERNET ISN'T BORKED
        
        Args:
            gamma: float, planning constant
            d: int, dimension of search space
            
        Returns:
            r: float, radius of "nearest" sphere
        """
        r = gamma*(np.log10(self.size + 1)/(self.size + 1))**(1./float(d))
        return r
        
class Map:
    def __init__(self, msg, dilate=-1):
        """
        Converts occupancy grid message to an actual map.
        
        Args:
            msg: OccupancyGrid, converted to a more usable map.
            dilate: int, how much to dilate filled-in pixels by. -1 means no dilation.
        """
        self.width, self.height, self.resolution = msg.info.width, msg.info.height, msg.info.resolution # width and height are integer numbers of cells, and resolution is in m/cell
        self.occ_matrix = np.reshape(np.array(msg.data), (self.height, self.width))
        self.origin = np.array([msg.info.origin.position.x, msg.info.origin.position.y])
        self.origin_angle = np.arctan2(msg.info.origin.orientation.z, msg.info.origin.orientation.w)*2. #- np.pi/2
        if VERBOSE:
            print(self.origin_angle)
        self.rot = np.array([[np.cos(self.origin_angle), -np.sin(self.origin_angle)],
                             [np.sin(self.origin_angle), np.cos(self.origin_angle)]])
        self.rot_inv = np.array([[np.cos(-self.origin_angle), -np.sin(-self.origin_angle)],
                                 [np.sin(-self.origin_angle), np.cos(-self.origin_angle)]])
        
        # Final all (m, n) pairs of cell coordinates that have value 0 (i.e. free space)
        m_cells, n_cells = np.where(self.occ_matrix == 0)
        self.free_cells = np.vstack((m_cells, n_cells)).T # f x 2 array

        if dilate != -1:
            #self.occ_matrix = np.maximum(np.minimum(morph.dilation(self.occ_matrix, morph.disk(dilate)) + self.occ_matrix, 100), -1)
            self.occ_matrix = morph.dilation(np.absolute(self.occ_matrix), morph.disk(dilate))
           
        
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
        
    def sample_free_space(self):
        """
        Samples a random free space location on the map, returning its coordinates.
        
        Args:
            None
            
        Returns:
            free_space_pose: np.array, gives a pose in free space np.array([x, y])
        """
        # Choose a random cell from this array
        index = np.random.choice(len(self.free_cells))
        coords = self.free_cells[index]
        m, n = coords[0], coords[1]
        
        # Convert to x, y location.
        cart_arr = self.cell_to_cartesian(coords)
        
        return cart_arr #np.array([x, y]) # Can alter to x, y, theta if necessary
        
    def check_free(self, arr):
        """
        Check to see if a given cartesian coordinate is in free space.
        
        Args:
            arr: np.array of cartesian coordinate pairs, representing where to check. Can either do np.array([x,y]) or np.array([[x1, x2, x3 ...], [y1, y2, y3 ...]])
        
        Returns:
            bool, True if empty, False if not. True if all coordinates in provided array are empty, False if not.
        """
        cell_location = self.cartesian_to_cell(arr)
        cell = self.occ_matrix[cell_location[0], cell_location[1]]
        return cell == 0

    def generate_line_path_unlimited(self, start, end, res=1000):
        return self.generate_line_path(start, end, eta=np.inf, res=1000)
    
    def generate_line_path(self, start, end, eta=np.inf, res=1000):
        """
        Generates a straight line path from start to end
        
        Args:
            start: np.array, starting coordinates np.array([x_start, y_start])
            end: np.array, starting coordinates np.array([x_end, y_end])
            eta: float, maximum length of path. Truncates path if cannot reach end. Default is inf, so no
            res: int, number of discrete positions along path.
        
        Returns:
            path: np.array of coordinates, representing discrete steps on the path or None, if no path exists            
        """
        direction_vector = end - start
        dv = direction_vector/res
        path = start

        # Unvectorized:
        # for i in range(res):
        #     partial_path = (i+1)*dv
        #     if not self.check_free(start + partial_path):
        #         return None
        #     elif np.linalg.norm(partial_path) >= eta:
        #         return path
        #     path = np.vstack((path, start + partial_path))
        # return path
        range = np.arange(1, res)
        partial_paths = np.vstack((range, range)).T*dv
        coords_along_partial_path = start + partial_paths
        exceeding_eta_indices = np.where(np.linalg.norm(partial_paths, axis=1) >= eta)
        if len(exceeding_eta_indices[0]) == 0:
            # none exceeding eta
            path = coords_along_partial_path
        else:
            last_index = np.min(exceeding_eta_indices)
            path = coords_along_partial_path[:last_index]
        if not np.all(self.check_free(path.T)):
            return None
        return path


class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/obstacles", OccupancyGrid, self.obstacle_map_cb)
        self.trajectory = LineTrajectory("/final_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_cb, queue_size=10)
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/final", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)
        self.rest_of_trajectory = LineTrajectory("/rest_of_trajectory")
        self.planning_dist = 50.0
        
        # FOR TESTING #
        self.vis_pub = rospy.Publisher("/particles", Marker, queue_size = 1)
        self.nearest_pub = rospy.Publisher("/nearest", Marker, queue_size = 1)
        
        self.start_pose = None
        self.goal_pose = None
        
        #self.max_iterations = 1000 # Max size of RRT* tree before breaking.
        self.dim = 2 # Configuration space dimensionality ([x, y] -> 2, [x, y, theta] -> 3)
        self.gamma = 10 # I don't actually know what this is called lol, it's used in rrt_star.radius()
        self.eta = .75
        self.goal_bias = 0.15
        self.map = None
        
        # FOR TESTING PURPOSES ONLY -- set to true if you want to see the tree grow
        self.enable_vis = True

    def obstacle_map_cb(self, msg):
        """
        Convert from OccupancyGrid to actual array.
        """
        self.map = Map(msg, dilate=5)
        if True:
            rospy.loginfo("Obstacle map received! Origin: " + str(msg.info.origin))

        self.pick_intermediate_goal(self.map)
        if True:
            rospy.loginfo("start pose: " + str(self.start_pose))
            rospy.loginfo("end pose: " + str(self.end_pose))
        
        self.start_time = rospy.get_time()
        self.time_thresh = 0.5  
        self.plan_path(self.start_pose, self.end_pose, self.map)
        self.extend_trajectory(self.rest_of_trajectory)
        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()

    def extend_trajectory(self):
        """
        adds rest of trajectory after intermediate points to the planned obstacle avoiding trajectory
        """
        traj_poses = self.trajectory.toPoseArray()
        rest_poses = self.rest_of_trajectory.toPoseArray()
        self.trajectory.fromPoseArray(traj_poses.poses + rest_poses.poses)

    def pick_intermediate_goal(self):
        """
        precondition: self.trajectory is populated w/new obstacle-agnostic trajectory
        takes in current planned path, and obstacle map (self.map)
        finds first segment within planning_dist that is blocked by obstacle + replans to endpoint of segment
        return false if we can just use current traj
        otherwise returns true&saves goal to self.end_pose, save rest of previous path to something else
        """
        first_i, _ = self.get_closest_seg(self.start_pose)
        if first_i + 1 >= self.num_pts:
            raise Exception("something bad happened")

        if self.map.generate_line_path_unlimited(self.start_pose, self.trajectory.np_points[first_i+1]):
            safe_pt = first_i+1
            while self.trajectory.distance_along_trajectory(safe_pt) - self.trajectory.distance_along_trajectory(first_i) < self.planning_dist:
                if safe_pt + 1 >= self.num_pts:
                    return False
                if not self.map.generate_line_path_unlimited(self.trajectory.np_points[safe_pt], self.trajectory.np_points[safe_pt+1]):
                    self.end_pose = self.trajectory.np_points(safe_pt+1)
                    self.rest_of_trajectory.clear()
                    poseArr = self.trajectory.toPoseArray()
                    self.rest_of_trajectory.fromPoseArray(poseArr.poses[safe_pt+2:])
                    return True
                    
                safe_pt += 1
        else:
            self.end_pose = self.trajectory.np_points(first_i+1)
            self.rest_of_trajectory.clear()
            poseArr = self.trajectory.toPoseArray()
            self.rest_of_trajectory.fromPoseArray(poseArr.poses[first_i+2:])
            return True
        
        

    def trajectory_cb(self, msg):
        if self.trajectory.empty():
            self.trajectory.fromPoseArray(msg)

        self.trajectory.make_np_array()
        #TODO: extend last pt??
        self.num_pts = len(self.trajectory.np_points)
        
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
        let the point the robot is at be p, and for each line segment the start and end points be v and w respect~
        """
        pv = pose_np - self.trajectory.np_points[:last_pt] # self.num_pts x 2

        # projecting p onto line bw vw, constraining b/w 0 and 1 to keep within line segment
        t = np.maximum(0, np.minimum(1, (pv*self.wv[:last_pt]).sum(axis=-1)/self.trajectory_dists[:last_pt]))
        proj = self.trajectory.np_points[:last_pt] + self.wv[:last_pt]*t[:, np.newaxis]
        # return index of minimum distance, and closest pt
        minseg = np.argmin(np.linalg.norm(pose_np-proj, axis=1))
        return minseg, proj[minseg]

    def odom_cb(self, msg):
        """
        
        """
        self.start_pose = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])


    def goal_cb(self, msg):
        self.end_pose = np.array([msg.pose.position.x, msg.pose.position.y])

        if VERBOSE:
            rospy.loginfo("start pose: " + str(self.start_pose))
            rospy.loginfo("end pose: " + str(self.end_pose))
        
        self.start_time = rospy.get_time()
        self.time_thresh = 117.  
        self.plan_path(self.start_pose, self.end_pose, self.map)

    def branched_from_existing_path(self, final_node, depth_underestimate=0):
        '''
        depth_underestimate: bias to add to depth.  Because we don't
           recursively update it, depth will steadily get less and less
           accurate as the path is denser than expected due to rewirings.

        Return a random free space that was near the existing
        path up to the final_node

        '''
        def find_ancestor(node, n):
            if n <= 0 or node.parent is None:
                return node
            else:
                return find_ancestor(node.parent, n-1)

        # back_track_depth = np.random.choice(int(final_node.depth + depth_underestimate))
        back_track_depth = np.random.choice(int(final_node.depth))
        x_temp = find_ancestor(final_node, back_track_depth)

        if x_temp.parent is None:
            # No point in exploring directly from the origin
            return self.map.sample_free_space()

        extend = 1
        frac = .1*extend

        while extend > 0:
            rand = np.random.uniform()*2*np.pi
            free_space_near_path = x_temp.value + extend*np.array([np.cos(rand), np.sin(rand)])
            if self.map.check_free(free_space_near_path):
                return free_space_near_path
            else:
                extend -= frac

        # unlucky -- just give a random free point
        return self.map.sample_free_space()

    def find_nearby_connectable(self, x_nearest, x_new):
        radius = self.rrt_star.radius(self.gamma, self.dim)
        X_nearby = self.rrt_star.nearest_in_n_sphere(x_new, 1)#min(radius, self.eta)) # Find all nodes near x_new
        
        # Find all X_near nodes that can actually be connected to x_new
        X_nearby_connectable = [x_nearest] # List of candidate parent nodes for x_new
        for node in X_nearby:
            TEMP_PATH = self.map.generate_line_path_unlimited(node.value, x_new)
            if (TEMP_PATH is not None) and (node != x_nearest): # If the path exists between node and x_new, append node to list of candidate parents. (x_nearest starts in the list, so it's gonna be excluded here.) 
                X_nearby_connectable.append(node)

        return X_nearby_connectable

    def find_best_parent(self, X_nearby_connectable, x_new):
        cost_min = np.inf
        node_min = None
                
        # Find which candidate parent node will impart the minimum cost to the x_new node when connected
        for node in X_nearby_connectable:
            TEMP_COST = node.cost + self.rrt_star.cost(node.value, x_new)
            if TEMP_COST < cost_min:
                cost_min = TEMP_COST
                node_min = node
        return cost_min, node_min

    def rewire(self, cost_min, node_new, X_nearby_connectable):
        # Rewire to minimize costs
        for node in X_nearby_connectable:
            potential_cost = cost_min + self.rrt_star.cost(node_new.value, node.value)
            if potential_cost < node.cost:
                self.rrt_star.adoption(node, node_new)
    
    def plan_path(self, start_point, end_point, map_obj):
        """
        RRT*. Tries to find a collision free path from start to goal.
        """
        # STUFF FOR TESTING 
        self.trajectory.clear()
        if self.enable_vis:
            marker = Marker()
            marker.header.frame_id = "/map"
            marker.type = marker.POINTS
            marker.action = marker.ADD
            
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            self.vis_pub.publish(marker)
        
        exploration_bias = 1.0 - self.goal_bias
        final_node = None
        num_existing_path_points_added = 0
        
        self.rrt_star = RRTStar(Node(start_point))
        self.max_iterations = self.rrt_star.max_size
        while self.rrt_star.size <= self.max_iterations:
            p = np.random.uniform()
            if p < exploration_bias:
                
                x_rand = self.map.sample_free_space()
            else:
                if final_node is None:
                    x_rand = end_point
                else:
                    x_rand = self.branched_from_existing_path(
                        final_node,
                        depth_underestimate=num_existing_path_points_added
                    )
                    num_existing_path_points_added += 1

            x_nearest = self.rrt_star.nearest(x_rand)  # Find the nearest node to x_rand

            path = self.map.generate_line_path(x_nearest.value, x_rand, eta=self.eta)
            if path is not None: # no obstacles between x_nearest and x_rand
                x_new = path[-1]
                X_nearby_connectable = self.find_nearby_connectable(x_nearest, x_new)

                cost_min, node_min = self.find_best_parent(X_nearby_connectable, x_new)

                X_nearby_connectable.remove(node_min) # Remove x_new's parent node from the list of nearby nodes so it is not considered for rewiring
                
                # Create the new node at x_new!
                node_new = self.rrt_star.add_config(node_min, x_new)
                
                if self.enable_vis:
                    # FOR TESTING ONLY #
                    # Code to publish marker for new node
                    ###########################################################################################
                    TEMP = Point()
                    TEMP.x = x_new[0]
                    TEMP.y = x_new[1]
                    TEMP.z = .05
                    marker.points.append(TEMP)
                    
                    TEMP = ColorRGBA()
                    TEMP.r = 1
                    TEMP.g = 0
                    TEMP.b = 0
                    TEMP.a = 1
                    
                    marker.colors.append(TEMP)
                    
                    self.vis_pub.publish(marker)
                    ###########################################################################################

                self.rewire(cost_min, node_new, X_nearby_connectable)
                
                if np.allclose(node_new.value, end_point, .05, 0) and (final_node is None):#np.array_equal(node_new.value, end_point):
                    final_node = node_new
                    # reduce exploration bias so that we reinforce the existing path
                    exploration_bias = .5
                    if VERBOSE:
                        print("Path found!!!!")
                        print(final_node.cost)
                if rospy.get_time() - self.start_time > self.time_thresh:
                    if VERBOSE:
                        print(self.rrt_star.size)
                    break

    
        if final_node is not None:
            if self.enable_vis:
                marker = Marker()
                marker.header.frame_id = "/map"
                marker.type = marker.POINTS
                marker.action = marker.ADD
                
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.points = []
                marker.colors = []
            def recur(node):
                if self.enable_vis:
                    TEMP = Point()
                    TEMP.x = node.value[0]
                    TEMP.y = node.value[1]
                    TEMP.z = .05
                    marker.points.append(TEMP)
                    
                    TEMP = ColorRGBA()
                    TEMP.r = 1
                    TEMP.g = 0
                    TEMP.b = 0
                    TEMP.a = 1
                    
                    marker.colors.append(TEMP)
                
                
                self.trajectory.points.append([node.value[0], node.value[1]])
                parent = node.parent
                if parent is not None:
                    recur(parent)
            recur(final_node)
            self.trajectory.points.reverse()
            if self.enable_vis:
                self.vis_pub.publish(marker)
            if VERBOSE:
                print (final_node.depth)
        else:
            # TODO:give best guess- find closest node to goal
            if VERBOSE:
                print("No path found! Please try again.")
            
        
        


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
