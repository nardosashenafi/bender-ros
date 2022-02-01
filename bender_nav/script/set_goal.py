#! /usr/bin/python3

__author__ = 'NardosAshenafi'
from email.mime import base
import roslib
# roslib.load_manifest('bender_nav')
import rospy
import actionlib
import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import MapMetaData, OccupancyGrid
import numpy as np
import math
from map_msgs.msg import OccupancyGridUpdate
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit 

class Costmap:
    def __init__(self):
        self.vf             = None
        self.counter        = 0
        self.q_g_lm         = None
        self.q_g_r          = None
        self.o_lm_map       = None
        self.o_r_map        = None
        self.num_steps      = 1
        self.w              = 0
        self.h              = 0
        self.R              = 0.0
        self.resolution     = 0.0
        self.cost_arr       = None
        self.circle_ind        = None

    def define_parameters(self, data):
        #load costmap data
        cost                = data.data
        print("update cost size = ", len(cost))
        print("sum of cost = ", np.sum(cost))

        self.w              = data.info.width
        self.h              = data.info.height
        self.resolution      = 0.1

        self.cost_arr       = np.zeros((self.h, self.w))
        self.vf             = np.zeros((self.h, self.w))
        # self.R              = np.sqrt(len(cost))/10.0/2.0 - 0.5
        self.R              = 4.0
        
        #change tuple cost to 2D matrix
        cost_ind                        = 0

        for i in range(self.h):      
            for j in range(self.w):
                self.cost_arr[i][j]     = cost[cost_ind]
                self.vf[i][j]           = abs(cost[cost_ind])       #unknown costs are -1. Abs makes them less favorable than 0 costs.
                cost_ind                += 1 

        # print("cost sum = ", np.sum(self.cost_arr))

    def create_vf(self):

        for k in range(20):         #TODO: should be saving vf and using the old vf on every point
            for i in range(1, self.h-2):
                for j in range(1, self.w-2):
                    self.vf[i][j] = 1.0/5.0*(self.vf[i][j] + self.vf[i+1][j] + self.vf[i-1][j] + self.vf[i][j-1] + self.vf[i][j+1])

        # print("value function generated!")

    def grids_between_lanes(self, forward_step):

        #find the grid on the lanes by finding the maximum cost. Use those grids to identify a goal between lanes
        col_forward_step_cost   = [self.cost_arr[i][forward_step] for i in range(self.h)]       
        lanes_index             = np.argwhere(col_forward_step_cost == np.max(col_forward_step_cost))   #identify lanes
        search_grid             = [lanes_index[0][0], lanes_index[-1][0]]

        return search_grid 

    def circle_around_robot(self):
    
        #the x values are sampled finer around x=R, because the formula of circle has more spread around x=R
        xr_coarse       = np.arange(0.0, self.R-2*self.resolution, self.resolution)
        xr_finer        = np.arange(self.R-2*self.resolution, self.R, 0.01)
        xr              = np.concatenate((xr_coarse, xr_finer))

        #The transpose of xr is necessary to arrange the circle indices in a counterclockwise manner.
        #otherwise it will be hard to find a goal between lanes indices
        xr_transposed   = xr[::-1]
        points_on_circle   = np.zeros((2, 4*len(xr)))
      

        #(xr, yr) = (0.0, 0.0) is the location of the robot
        ind = 0
        # shiftx = np.sqrt((self.w*self.resolution)**2 + (self.h*self.resolution)**2)/np.sqrt(2.0)
        shiftx = self.w*self.resolution/2.0
        shifty = self.h*self.resolution/2.0
        #SAVED IN COUNTERCLOCKWISE ROTATION FROM (X, Y) = (0, 1)
        #second quadrant
        for i in range(len(xr)):

            points_on_circle[0, i+ind]           = shiftx - xr[i]
            points_on_circle[1, i+ind]           = shifty + np.sqrt(self.R**2 - xr[i]**2)
        
        ind += len(xr)

        #third quadrant
        for i in range(len(xr)):    
            points_on_circle[0, i+ind]         = shiftx - xr_transposed[i]
            points_on_circle[1, i+ind]         = shifty - np.sqrt(self.R**2 - xr_transposed[i]**2)

        ind += len(xr)

        #fourth quadrant
        for i in range(len(xr)):
            points_on_circle[0, i+ind]         = shiftx + xr[i]
            points_on_circle[1, i+ind]         = shifty - np.sqrt(self.R**2 - xr[i]**2) 

        ind += len(xr)

        #first quadrant
        for i in range(len(xr)):
            points_on_circle[0, i+ind]         = shiftx + xr_transposed[i]
            points_on_circle[1, i+ind]         = shifty + np.sqrt(self.R**2 - xr_transposed[i]**2)    

        # print(points_on_circle)
        ind += len(xr)
        self.circle_ind = np.vectorize(math.floor)(points_on_circle*1.0/self.resolution)

    def circle_cost(self): 

        self.circle_around_robot()
        cost_on_circle             = np.zeros((self.circle_ind.shape[1]))

        #find the cost of each grid on the circle
        for i in range(self.circle_ind.shape[1]):
            cost_on_circle[i]   = self.cost_arr[self.circle_ind[1][i]][self.circle_ind[0][i]]

        return cost_on_circle

    def circle_cost_rng(self, ind1, ind2): 

        cost_in_range   = []

        for i in range(ind1, ind2):
            cost_in_range.append(self.cost_arr[self.circle_ind[1][i]][self.circle_ind[0][i]])

        return cost_in_range

    def circle_vf(self, ind1, ind2):

        # self.circle_around_robot()
        vf_on_circle  = []

        for i in range(ind1, ind2):
            vf_on_circle.append(self.vf[self.circle_ind[1][i]][self.circle_ind[0][i]])

        return vf_on_circle

    def identify_lanes(self):
        #given agent heading, this functions finds the two lanes. To do so, we first create histogram of all the columns and row
        #to find the base of each lane. Then we use a 5x5 sliding window to move from the base of the lane to the top edge.
        #We save and return each lane separately

        # collapsed_col = self.vf[1:np.size(self.vf, 0)-2,np.size(self.vf, 0)-1]
        collapsed_col = self.vf[1:np.size(self.vf, 0)-2,0]

        for i in range(0, math.floor(np.size(self.vf, 1)/2)-1, 1): #check half of the costmap grid (around curves histogram will be shifted at base)
            collapsed_col = 1.0/1.0*(collapsed_col + self.vf[0:-3,i+1] + self.vf[2:-1,i+1] + self.vf[1:-2,i+1])

        # for i in range(np.size(self.vf, 1)-1, math.floor(np.size(self.vf, 1)/2), -1): #check half of the costmap grid (around curves histogram will be shifted at base)
        #     collapsed_col = 1.0/4.0*(collapsed_col + self.vf[0:-3,i-1] + self.vf[2:-1,i-1] + self.vf[1:-2,i-1])

        collapsed_row = self.vf[0,1:np.size(self.vf, 0)-2]
        # for j in range(1, np.size(self.vf, 0)-1):
        for j in range(0, math.floor(np.size(self.vf, 0)/2)-1): #check half of the costmap grid (around curves histogram will be shifted at base)
            collapsed_row = 1.0/1.0*(collapsed_row + self.vf[j+1, 0:-3] + self.vf[j+1, 2:-1] + self.vf[j+1, 1:-2])

        # base_lanes_col = np.zeros(2)
        # base_lanes_row = np.zeros(2)
        print(collapsed_col)
        sort_col = np.sort(collapsed_col)
        sort_row = np.sort(collapsed_row)
        max_col_ind = np.argwhere(collapsed_col >= sort_col[-10])
        max_row_ind = np.argwhere(collapsed_row >= sort_row[-10])

        sort_index_col = np.sort(max_col_ind)
        sort_index_row = np.sort(max_row_ind)

        base_lanes_col = [[sort_index_col[0], 0], [sort_index_col[-1], 0]]
        base_lanes_row = [[0, sort_index_row[0]], [0, sort_index_row[-1]]]

        #TODO: test the index sorting

        win_size = 5
        #pick row or column; may pick one of each if the agent is on a curve
        if np.sum(sort_col[-2]) >= np.sum(sort_row[-2]):
            base        = base_lanes_col
            search_dirc = range(np.size(self.vf, 1)-win_size, win_size, -win_size)
            def sliding_window(arr, i, j) :  arr[j-win_size:j+win_size,i-win_size:i]

        elif np.sum(sort_col[-2]) < np.sum(sort_row[-2]):
            base        = base_lanes_row
            search_dirc = range(win_size, np.size(self.vf, 1)-win_size, win_size)
            def sliding_window(arr, i, j) :  arr[j:j+win_size,i-win_size:i+win_size]

        print("base = ", base)

        #create the sliding window

        lane1 = []
        lane2 = []
        lane1.append(base[0])
        lane2.append(base[1])

        for i in search_dirc:
            #check moving base plus 5x5 sliding window; pick one index for every 5x5 window to curve fit
            lane1_point = lane1[-1] + np.argmax(sliding_window(self.vf, lane1[-1][0], lane1[-1][1]))
            lane2_point = lane2[-1] + np.argmax(sliding_window(self.vf, lane2[-1][0], lane2[-1][1]))
            lane1.append(lane1_point)
            lane2.append(lane2_point)

        return lane1, lane2

    def lane_fit(self, x, a0, a1, a2, a3):
        return a0 + a1*x + a2*x**2 + a3*x**3                

    def approximate_poly_lane(self):
        #approximate polynomial fit for each lane
        lane1, lane2 = self.identify_lanes()
        x1 = lane1[:,0]
        y1 = lane1[:,1]
        x2 = lane2[:,0]
        y2 = lane2[:,1]

        #TODO: draw circle in rviz
        #TODO: if the lanes are not visible (not enough data points for optimizer), handle exception

        popt1, cov1 = curve_fit(self.lane_fit, x1, y1)
        a01, a11, a21, a31 = popt1 
        popt2, cov2 = curve_fit(self.lane_fit, x2, y2)
        a02, a12, a22, a32 = popt2 
        return  a01, a11, a21, a31, a02, a12, a22, a32
        
    def goal_on_circle_wrt_lm(self):

        cost_on_circle             = self.circle_cost()

        # a0, a1, a2, a3          = self.approximate_poly_lane(self)
        # lanes_index             = self.lane_fit(x, a0, a1, a2, a3)
        lanes_index             = np.argwhere(cost_on_circle == np.max(cost_on_circle))   #identify lanes
        search_grid             = [lanes_index[0][0], lanes_index[1][0]]
        print("lanes_index = ", lanes_index)
        ind = 1
        #make sure two consective indices are not selected
        while (search_grid[1] - search_grid[0]) < 2:
            ind += 1
            search_grid[1] = lanes_index[ind][0]
        print("search_grid = ", search_grid)
        # print("circle_ind[search_grid] = ", self.circle_ind[0][search_grid[0]:search_grid[1]], self.circle_ind[1][search_grid[0]:search_grid[1]])

        #select minimum value function
        vf_on_circle_in_lanes      = self.circle_vf(search_grid[0], search_grid[1])
        
        min_ind                 = search_grid[0] + np.argmin(vf_on_circle_in_lanes)
        # print("min_ind = ", min_ind)
        min_cost_ind            = [self.circle_ind[0, min_ind], self.circle_ind[1, min_ind]]
        min_cost                = np.min(vf_on_circle_in_lanes)
       
        self.q_g_lm     = tuple(np.multiply((min_cost_ind[0], min_cost_ind[1] , 0.0, 1.0/self.resolution), self.resolution))
        print("q_g_lm = ", self.q_g_lm)
        self.make_plots(search_grid, lanes_index, min_cost_ind)

    def make_plots(self):

        if self.counter == 1: 
            # plt.plot([self.cost_arr[i][forward_step] for i in range(0, self.h)])
            # plt.plot([self.vf[i][forward_step] for i in range(0, self.h)])
            x       = range(0, self.w)
            y       = range(0, self.h)
            X, Y    = np.meshgrid(x, y)
            plt.pcolormesh(X, Y, self.vf)
            # #plot circle using self.circle_ind
            # plt.scatter(self.circle_ind[0], self.circle_ind[1])
            # plt.scatter(self.circle_ind[0][lanes_index], self.circle_ind[1][lanes_index])
            # plt.scatter(self.circle_ind[0][search_grid[0]:search_grid[1]], self.circle_ind[1][search_grid[0]:search_grid[1]])
            # plt.scatter(min_cost_ind[0], min_cost_ind[1])
            # vf_circle      = self.circle_vf(search_grid[0], search_grid[1])
            # cost_in_rng             = self.circle_cost_rng(search_grid[0], search_grid[1])
            # plt.plot(vf_circle)
            # plt.plot(cost_in_rng)
            #PLOT LANES
            lane1, lane2 = self.identify_lanes()
            plt.scatter(lane1[0], lane1[1])
            plt.scatter(lane2[0], lane2[1])
            plt.show()
        self.counter += 1

    def min_vf_goal(self, forward_step):

        search_grid             = self.grids_between_lanes(forward_step)
        goal_set_col            = [self.vf[i][forward_step] for i in range(search_grid[0], search_grid[1])]

        #select minimum value function
        min_cost_ind            = search_grid[0] + np.argmin(goal_set_col)
        min_cost                = np.min(goal_set_col)
        # self.min_cost_goal   = tuple(map(operator.sub, (5.0, 5.0, 0.0), tuple(np.multiply((forward_step, min_cost_ind, 0.0), 0.1))))
        self.q_g_r          = tuple(np.multiply((self.w/2.0 - forward_step, self.h/2.0 - min_cost_ind, 0.0), self.resolution))

    def transform_lm_map(self):

        transformer = tf.TransformerROS(True, rospy.Duration(5.0))
        trans = [self.o_lm_map.position.x, self.o_lm_map.position.y, self.o_lm_map.position.z]
        rot = [self.o_lm_map.orientation.x, self.o_lm_map.orientation.y, self.o_lm_map.orientation.z, self.o_lm_map.orientation.w]
        return transformer.fromTranslationRotation(trans, rot)

    def transform_r_map(self, listener):

        trans, rot = current_position(listener)
        self.o_r_map = create_pose_stamp(trans, 'map')
        self.o_r_map.pose.orientation.x, self.o_r_map.pose.orientation.y, self.o_r_map.pose.orientation.z, self.o_r_map.pose.orientation.w = rot
        
        transformer = tf.TransformerROS(True, rospy.Duration(5.0))
        return transformer.fromTranslationRotation(trans, rot)

    def goal_wrt_r(self, T_r_map, T_lm_map):

        self.q_g_r  = np.dot(np.dot(np.linalg.inv(T_r_map),(T_lm_map)), self.q_g_lm)
        print("q_g_r  = ", self.q_g_r)


    # def costmap_update_callback(self, data):
        
    #     self.define_parameters(data)
    #     self.create_vf()
    #     # self.make_plots()
    #     # forward_step        = 5      
    #     # self.min_vf_goal(forward_step)
    #     self.goal_on_circle_wrt_lm()
    #     listener = self.goal_wrt_r()

    #     print("goal is = ", self.q_g_r )
    #     result = setGoalClient(self.q_g_r, listener )

    #     if result:
    #         rospy.loginfo("Goal execution done!")
    #         self.num_steps += 1

    def costmap_callback(self, data):

        self.o_lm_map = data.info.origin
        print("o_lm origin = ", self.o_lm_map)
        self.define_parameters(data)
        self.create_vf()
        self.make_plots()

        # current_pose_listener  = tf.TransformListener()

        # T_r_map     = self.transform_r_map(current_pose_listener)
        # T_lm_map    = self.transform_lm_map()
        # self.goal_on_circle_wrt_lm()
        # self.goal_wrt_r(T_r_map, T_lm_map)

        # print("goal is = ", self.q_g_r )
        # result = setGoalClient(self.q_g_r, current_pose_listener )

        # if result:
        #     rospy.loginfo("Goal execution done!")
        #     self.num_steps += 1
        # print("Localmap location updated. This does not mean localcostmap/costmap_updates is updated")


    def costmap_subscriber(self):

        rospy.Subscriber("/bender_nav/local_costmap/costmap", OccupancyGrid, self.costmap_callback)
        # rospy.Subscriber("/bender_nav/local_costmap/costmap_updates", OccupancyGridUpdate, self.costmap_update_callback)

def create_pose_stamp(delta_base, frame):
    p                   = PoseStamped()
    p.header.frame_id   = frame
    p.header.stamp      = rospy.Time.now()

    p.pose.position.x   = delta_base[0]
    p.pose.position.y   = delta_base[1]
    p.pose.position.z   = delta_base[2]
    
    return p

def transform_goal_to_map(delta_map):

    goal = MoveBaseGoal()

    goal.target_pose.header.frame_id    = 'map'
    goal.target_pose.header.stamp       = rospy.Time.now()

    goal.target_pose.pose.position.x    = delta_map.pose.position.x 
    goal.target_pose.pose.position.y    = delta_map.pose.position.y 
    goal.target_pose.pose.position.z    = delta_map.pose.position.z 

    # goal.target_pose.pose.position.x = delta_map[0]
    # goal.target_pose.pose.position.y = delta_map[1]
    # goal.target_pose.pose.position.z = delta_map[2]

    #pass goal with normalized quaternion
    #TODO: sign of the (xr-min_cost) depends on the heading of the robot wrt the local map indices.
    #Replace the negative sign with dot product between the robot's heading and local map indexing
    goal.target_pose.pose.orientation = Quaternion(*(tf.transformations.quaternion_from_euler(0,0,3.1415, axes='sxyz')))

    return goal


def setGoalClient(delta_base, current_pose_listener):
    
    #receive current position
    rospy.sleep(1)
    cur_positon, cur_rot = current_position(current_pose_listener)
    
    #define desired position as PoseStamped message 
    p = create_pose_stamp(delta_base, 'base_footprint')
    
    #transform delta_base into map frame
    # rate = rospy.Rate(10.0)
    now = rospy.Time.now()
    current_pose_listener.waitForTransform("/base_footprint", "/map", now, rospy.Duration(1.0))
    delta_map = current_pose_listener.transformPose("/map", p)
    #pass delta_map through SimpleActionClient
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    client.wait_for_server()
    goal = transform_goal_to_map(delta_map)

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        # rospy.signal_shutdowm("Action server not available!")
    else:
        results =  client.get_result()
        current_position(current_pose_listener)
        return results


def delta_pose():
    return (-0.5, 0.0, 0.0)

def current_position(current_pose):
    current_pose.waitForTransform("/map", "/base_footprint", rospy.Time(), rospy.Duration(5.0))
    now = rospy.Time.now()
    
    success = False
    while not (success):
        try:
            current_pose.waitForTransform("/map", "/base_footprint", now, rospy.Duration(5.0))
            (trans, rot) = current_pose.lookupTransform("/map", "/base_footprint", now)
            success = True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print("In exception")
        rospy.sleep(0.5)
    print(trans)
    return trans, rot

if __name__ == '__main__':
    try:
        rospy.init_node('set_goal_client')
        # result = setGoalClient((-0.5, 0.0, 0.0))

        # if result:
        #     rospy.loginfo("Goal execution done!")

        cm = Costmap()

        print("Ready to send goal")
        cm.costmap_subscriber()
        rospy.sleep(10)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


##
# listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(10.0))
# trans, rot = listener.lookupTransform("/map","/base_link", rospy.Time(0))
##
# client.send_goal(goal)