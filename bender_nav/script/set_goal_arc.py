#! /usr/bin/python3

__author__ = 'NardosAshenafi'

from nav_base import *
from lane_fit import *
# roslib.load_manifest('bender_nav')
import actionlib

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from nav_msgs.msg import MapMetaData, OccupancyGrid

from map_msgs.msg import OccupancyGridUpdate
import matplotlib.pyplot as plt

CIRCLE_RESOLUTION = 1/5
#TODO: subscribe to heading

class Costmap():
    def __init__(self, current_pose_listener):
        self.vf                     = None
        self.counter                = 0
        self.q_g_r                  = None
        self.o_r_map                = None
        self.num_steps              = 1
        self.w                      = 0
        self.h                      = 0
        self.R                      = 0.0
        self.resolution             = 0.05
        self.cost_arr               = None
        self.circle_ind             = None
        self.heading                = None  #subscribe from state estimator to get the initial heading
        self.agent_prevpose_map     = create_pose_stamp([0.0,0.0,0.0], 'map')
        self.current_pose_listener  = current_pose_listener

    def define_parameters(self, data):
        #load costmap data
        cost                = data.data
        print("update cost size = ", len(cost))
        print("sum of cost = ", np.sum(cost))

        self.w              = data.info.width
        self.h              = data.info.height
        # self.resolution      = 0.1

        self.cost_arr       = np.zeros((self.h, self.w))
        self.vf             = np.zeros((self.h, self.w))
        # self.R              = np.sqrt(len(cost))/10.0/2.0 - 0.5
        # self.R              = 4.0
        self.R              = 4.0
        
        #change tuple cost to 2D matrix
        cost_ind                        = 0

        for i in range(self.h):      
            for j in range(self.w):
                self.cost_arr[i,j]     = cost[cost_ind]
                self.vf[i,j]           = abs(cost[cost_ind])       #unknown costs are -1. Abs makes them less favorable than 0 costs.
                cost_ind                += 1 

        # print("cost sum = ", np.sum(self.cost_arr))

    def create_vf(self):

        for k in range(20):         #TODO: should be saving vf and using the old vf on every point
            for i in range(1, self.h-2):
                for j in range(1, self.w-2):
                    self.vf[i,j] = 1.0/5.0*(self.vf[i,j] + self.vf[i+1,j] + self.vf[i-1,j] + self.vf[i,j-1] + self.vf[i,j+1])

        # print("value function generated!")

    def grids_between_lanes(self, forward_step):

        #find the grid on the lanes by finding the maximum cost. Use those grids to identify a goal between lanes
        col_forward_step_cost   = [self.cost_arr[i,forward_step] for i in range(self.h)]       
        lanes_index             = np.argwhere(col_forward_step_cost == np.max(col_forward_step_cost))   #identify lanes
        search_grid             = [lanes_index[0][0], lanes_index[-1][0]]

        return search_grid 

    def circle_around_robot(self):
    
        #the x values are sampled finer around x=R, because the formula of circle has more spread around x=R
        res             = self.resolution*CIRCLE_RESOLUTION
        xr_coarse       = np.arange(0.0, self.R-2*res, res)
        xr_finer        = np.arange(self.R-2*res, self.R, 0.001)
        xr              = np.concatenate((xr_coarse, xr_finer))

        #The transpose of xr is necessary to arrange the circle indices in a counterclockwise manner.
        #otherwise it will be hard to find a goal between lanes indices
        xr_transposed   = xr[::-1]
        points_on_circle   = []
      

        #(xr, yr) = (0.0, 0.0) is the location of the robot
        ind = 0
        # shiftx = np.sqrt((self.w*self.resolution)**2 + (self.h*self.resolution)**2)/np.sqrt(2.0)
        shiftx = self.w*self.resolution/2.0
        shifty = self.h*self.resolution/2.0
        # shiftx = self.w*res/2.0
        # shifty = self.h*res/2.0
        #SAVED IN COUNTERCLOCKWISE ROTATION FROM (X, Y) = (0, 1)
        #second quadrant
        for i in range(len(xr)):

            points_on_circle.append([shiftx - xr[i], shifty + np.sqrt(self.R**2 - xr[i]**2)])
        
        # ind += len(xr)

        #third quadrant
        for i in range(len(xr)):    
            points_on_circle.append([shiftx - xr_transposed[i], shifty - np.sqrt(self.R**2 - xr_transposed[i]**2)])

        # ind += len(xr)

        #fourth quadrant
        for i in range(len(xr)):
            points_on_circle.append([shiftx + xr[i], shifty - np.sqrt(self.R**2 - xr[i]**2)])

        # ind += len(xr)

        #first quadrant
        for i in range(len(xr)):
            points_on_circle.append([shiftx + xr_transposed[i], shifty + np.sqrt(self.R**2 - xr_transposed[i]**2)])

        # print(points_on_circle)
        # ind += len(xr)
        self.circle_ind = list(map(self.grids_to_index, points_on_circle))

    def grid_to_index(self, p):
        return math.floor(p*1.0/(self.resolution))

    def grids_to_index(self, ps):
        return list(map(self.grid_to_index, ps))

    def circle_cost(self): 

        self.circle_around_robot()
        cost_on_circle             = np.zeros(len(self.circle_ind))

        #find the cost of each grid on the circle
        for i in range(self.circle_ind.shape[1]):
            cost_on_circle[i]   = self.cost_arr[self.circle_ind[i][0],self.circle_ind[i][1]]

        return cost_on_circle

    def circle_cost_rng(self, ind1, ind2): 

        cost_in_range   = []

        for i in range(ind1, ind2):
            cost_in_range.append(self.cost_arr[self.circle_ind[i][0],self.circle_ind[i][1]])

        return cost_in_range

    def circle_vf(self, ind1, ind2):

        # self.circle_around_robot()
        vf_on_circle  = []

        for i in range(ind1, ind2):
            vf_on_circle.append(self.vf[self.circle_ind[i][0],self.circle_ind[i][1]])

        return vf_on_circle
       
    # def goal_on_circle_wrt_lm(self):

    #     cost_on_circle             = self.circle_cost()

    #     # a0, a1, a2, a3          = self.approximate_poly_lane(self)
    #     # lanes_index             = self.lane_fit(x, a0, a1, a2, a3)
    #     lanes_index             = np.argwhere(cost_on_circle == np.max(cost_on_circle))   #identify lanes
    #     search_grid             = [lanes_index[0][0], lanes_index[1][0]]
    #     print("lanes_index = ", lanes_index)
    #     ind = 1
    #     #make sure two consective indices are not selected
    #     while (search_grid[1] - search_grid[0]) < 2:
    #         ind += 1
    #         search_grid[1] = lanes_index[ind][0]
    #     print("search_grid = ", search_grid)
    #     # print("circle_ind[search_grid] = ", self.circle_ind[0][search_grid[0]:search_grid[1]], self.circle_ind[1][search_grid[0]:search_grid[1]])

    #     #select minimum value function
    #     vf_on_circle_in_lanes      = self.circle_vf(search_grid[0], search_grid[1])
        
    #     min_ind                 = search_grid[0] + np.argmin(vf_on_circle_in_lanes)
    #     # print("min_ind = ", min_ind)
    #     min_cost_ind            = [self.circle_ind[0, min_ind], self.circle_ind[1, min_ind]]
    #     min_cost                = np.min(vf_on_circle_in_lanes)
       
    #     self.q_g_lm     = tuple(np.multiply((min_cost_ind[0], min_cost_ind[1] , 0.0, 1.0/self.resolution), self.resolution))
    #     print("q_g_lm = ", self.q_g_lm)
    #     self.make_plots(search_grid, lanes_index, min_cost_ind)

    def heading_map(self):
        ac          = self.o_r_map.pose.position
        ap          = self.agent_prevpose_map.pose.position
        heading     = np.add([ac.x, ac.y, ac.z], [-ap.x ,-ap.y ,-ap.z])
        print("previous pose = ", ap)
        print("Heading = ", heading)
        return heading

    def heading_lm(self, T_lm_map):
        h_wrt_map = self.heading_map()
        h_wrt_lm = map_to_lm(h_wrt_map, T_lm_map)
        return h_wrt_lm[0:2]

    def neighbor_indices(self, e):
        delta  = [[0,0], [0, 1], [1, 0], [1, 1], [-1, 1], 
                  [0, -1], [-1, 0], [-1, -1], [1, -1]]

        return list(map(lambda x: list(np.add(e, x)), delta))

    def find_intersection_or_neighboring(self, lane_index, circle_ind):
        
        return [list(e) for e in circle_ind for a in self.neighbor_indices(e) if a in lane_index]

    def lanes_on_circle(self, lane1_index, lane2_index):

        self.circle_around_robot()
        # circle_ind_set_tup  = set([ tuple(a) for a in self.circle_ind])
        # lane1_index_tup     = [ tuple(a) for a in lane1_index]
        # lane2_index_tup     = [ tuple(a) for a in lane2_index]
        # lane1_on_circle_tup = circle_ind_set_tup.intersection(lane1_index_tup)
        # lane2_on_circle_tup = circle_ind_set_tup.intersection(lane2_index_tup)
        # lane1_on_circle     = [list(a) for a in lane1_on_circle_tup]
        # lane2_on_circle     = [list(a) for a in lane2_on_circle_tup]
        
        lane1_on_circle = self.find_intersection_or_neighboring(lane1_index, self.circle_ind)
        lane2_on_circle = self.find_intersection_or_neighboring(lane2_index, self.circle_ind)

        print("lane1 on circle = ", lane1_on_circle)
        print("lane2 on circle = ", lane2_on_circle)
        # print("circle_ind = ", self.circle_ind)

        return lane1_on_circle, lane2_on_circle 

    def pick_arc(self, T_lm_map, lane1_index, lane2_index):

        heading      = self.heading_lm(T_lm_map)
        lane1_on_circle, lane2_on_circle  = self.lanes_on_circle(lane1_index, lane2_index)
        search_grid_on_circle = [[0.0,0.0], [0.0,0.0]]
        
        ac          = self.o_r_map.pose.position
        agent_pose_map  = [ac.x, ac.y, ac.z]
        agent_pose_lm   = map_to_lm(agent_pose_map, T_lm_map)
        print("agent pose lm = ", agent_pose_lm)
        print("resolution = ", self.resolution)

        #pick the arc that is along the heading
        for l in lane1_on_circle:
            l1_wrt_agentpos = np.subtract(agent_pose_lm[0:2], np.dot(l[::-1], self.resolution))
            if np.sign(np.dot(l1_wrt_agentpos, heading)) > 0:
                search_grid_on_circle[0] = self.circle_ind.index(l)
                print("lane1 selection = ", l)
                # print("shifted lane1 indices = ", l1_wrt_agentpos)
                break


        for l in lane2_on_circle:
            l2_wrt_agentpos = np.subtract(agent_pose_lm[0:2], np.dot(l[::-1], self.resolution))
            if np.sign(np.dot(l2_wrt_agentpos, heading)) > 0:
                search_grid_on_circle[1] = self.circle_ind.index(l)
                print("lane 2 selection = ", l)
                # print("shifted lane2 indices = ", l2_wrt_agentpos)
                break

        print("search_grid_on_circle = ", search_grid_on_circle)
        self.make_plots(lane1_index,lane2_index, lane1_on_circle, lane2_on_circle, search_grid_on_circle)

        return np.sort(search_grid_on_circle) 


    def goal_on_arc_bt_lanes(self, search_grid_on_circle):
        
        vf_on_circle_in_lanes_ccw   = self.circle_vf(search_grid_on_circle[0], search_grid_on_circle[1])
        vf_on_circle_in_lanes_cw   = np.concatenate(self.circle_vf(0, search_grid_on_circle[0]), 
                                        self.circle_vf(search_grid_on_circle[1], -1))

        #To pick an arc that spans in the CW or CCW direction, simply pick the shortest. That is within lanes.
        vf_on_circle_in_lanes = len(vf_on_circle_in_lanes_ccw) < vf_on_circle_in_lanes_cw if vf_on_circle_in_lanes_ccw else vf_on_circle_in_lanes_cw
        min_ind                 = search_grid_on_circle[0] + np.argmin(vf_on_circle_in_lanes)
        # print("min_ind = ", min_ind)
        min_cost_ind            = self.circle_ind[min_ind]
        min_cost                = np.min(vf_on_circle_in_lanes)

        return min_cost_ind 

    def goal_on_circle_wrt_lm(self, T_lm_map):

        a01, a11, a21, a31, a02, a12, a22, a32  = approximate_poly_lane(self.vf, self.cost_arr)
        x                   = np.array(range(0, self.w))
        lane1_index         = lane_fit(x, a01, a11, a21, a31)
        lane2_index         = lane_fit(x, a02, a12, a22, a32)

        search_grid         = self.pick_arc(T_lm_map, lane1_index, lane2_index)

        #select minimum value function
        min_cost_ind        = self.goal_on_arc_bt_lanes(search_grid) 
       
        q_g_lm     = tuple(np.multiply((min_cost_ind[0], min_cost_ind[1] , 0.0, 1.0/self.resolution), self.resolution))
        print("q_g_lm = ", q_g_lm)

        return q_g_lm 
       

    # def make_plots(self, search_grid, min_cost_ind):
    def make_plots(self, lane1_index_poly, lane2_index_poly, lane1_on_circle, lane2_on_circle, search_grid_on_circle):

        if self.counter == 0: 
            # plt.plot([self.cost_arr[i][forward_step] for i in range(0, self.h)])
            # plt.plot([self.vf[i][forward_step] for i in range(0, self.h)])
            x       = range(0, self.w)
            y       = range(0, self.h)
            X, Y    = np.meshgrid(x, y)
            plt.pcolormesh(X, Y, self.vf)
            # #plot circle using self.circle_ind
            plt.scatter([l[1] for l in self.circle_ind], 
                        [l[0] for l in self.circle_ind])
            # plt.scatter(lane1_on_circle[0][1], lane1_on_circle[0][0])
            # plt.scatter(lane2_on_circle[0][1], lane2_on_circle[0][0])
            # plt.scatter(self.circle_ind[0][lanes_index], self.circle_ind[1][lanes_index])

            search_grid = np.sort(search_grid_on_circle)

            plt.scatter([l[1] for l in self.circle_ind[search_grid[0]:search_grid[1]]], 
                        [l[0] for l in self.circle_ind[search_grid[0]:search_grid[1]]])
            # plt.scatter(min_cost_ind[0], min_cost_ind[1])
            # vf_circle      = self.circle_vf(search_grid[0], search_grid[1])
            # cost_in_rng             = self.circle_cost_rng(search_grid[0], search_grid[1])
            # plt.plot(vf_circle)
            # plt.plot(cost_in_rng)
            #PLOT LANES
            # lane1_index, lane2_index = self.identify_lanes()
            # plt.scatter([l[1] for l in lane1_index], [l[0] for l in lane1_index])
            # plt.scatter([l[1] for l in lane2_index], [l[0] for l in lane2_index])

            plt.scatter([l[1] for l in lane1_index_poly], [l[0] for l in lane1_index_poly])
            plt.scatter([l[1] for l in lane2_index_poly], [l[0] for l in lane2_index_poly])
            plt.show()
        self.counter += 1

    def min_vf_goal(self, forward_step):

        search_grid             = self.grids_between_lanes(forward_step)
        goal_set_col            = [self.vf[i,forward_step] for i in range(search_grid[0], search_grid[1])]

        #select minimum value function
        min_cost_ind            = search_grid[0] + np.argmin(goal_set_col)
        min_cost                = np.min(goal_set_col)
        # self.min_cost_goal   = tuple(map(operator.sub, (5.0, 5.0, 0.0), tuple(np.multiply((forward_step, min_cost_ind, 0.0), 0.1))))
        self.q_g_r          = tuple(np.multiply((self.w/2.0 - forward_step, self.h/2.0 - min_cost_ind, 0.0), self.resolution))

    def goal_wrt_r(self, q_g_lm, T_r_map, T_lm_map):

        q_g_r  = np.dot(np.dot(np.linalg.inv(T_r_map),(T_lm_map)), q_g_lm)
        print("q_g_r  = ", q_g_r)
        return q_g_r


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

        o_lm_map = data.info.origin
        print("o_lm origin = ", o_lm_map)
        self.define_parameters(data)
        self.create_vf()

        self.o_r_map, T_r_map     = transform_r_map(self.current_pose_listener)
        T_lm_map    = transform_lm_map(o_lm_map)
        q_g_lm      = self.goal_on_circle_wrt_lm(T_lm_map)
        # self.q_g_r     = self.goal_wrt_r(q_g_lm, T_r_map, T_lm_map)

        # print("goal is = ", self.q_g_r )
        # result = setGoalClient(self.q_g_r, current_pose_listener )
        self.agent_prevpose_map = np.copy(self.o_r_map)     # NEEDS TO COME AFTER goal_on_circle_wrt_lm
        # if result:
        #     rospy.loginfo("Goal execution done!")
        #     self.num_steps += 1
        # print("Localmap location updated. This does not mean localcostmap/costmap_updates is updated")


    def costmap_subscriber(self):

        rospy.Subscriber("/bender_nav/local_costmap/costmap", OccupancyGrid, self.costmap_callback)
        # rospy.Subscriber("/bender_nav/local_costmap/costmap_updates", OccupancyGridUpdate, self.costmap_update_callback)



# if __name__ == '__main__':
#     try:
#         rospy.init_node('set_goal_client')
#         # result = setGoalClient((-0.5, 0.0, 0.0))

#         # if result:
#         #     rospy.loginfo("Goal execution done!")

#         cm = Costmap()

#         print("Ready to send goal")
#         cm.costmap_subscriber()
#         #publish cm.q_g_r
#         rospy.sleep(10)

#         rospy.spin()

#     except rospy.ROSInterruptException:
#         rospy.loginfo("Navigation test finished.")


##
# listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(10.0))
# trans, rot = listener.lookupTransform("/map","/base_link", rospy.Time(0))
##
# client.send_goal(goal)


# def create_pose_stamp(delta_base, frame):
#     p                   = PoseStamped()
#     p.header.frame_id   = frame
#     p.header.stamp      = rospy.Time.now()

#     p.pose.position.x   = delta_base[0]
#     p.pose.position.y   = delta_base[1]
#     p.pose.position.z   = delta_base[2]
    
#     return p

# def transform_goal_to_map(delta_map):

#     goal = MoveBaseGoal()

#     goal.target_pose.header.frame_id    = 'map'
#     goal.target_pose.header.stamp       = rospy.Time.now()

#     goal.target_pose.pose.position.x    = delta_map.pose.position.x 
#     goal.target_pose.pose.position.y    = delta_map.pose.position.y 
#     goal.target_pose.pose.position.z    = delta_map.pose.position.z 

#     # goal.target_pose.pose.position.x = delta_map[0]
#     # goal.target_pose.pose.position.y = delta_map[1]
#     # goal.target_pose.pose.position.z = delta_map[2]

#     #pass goal with normalized quaternion
#     #TODO: sign of the (xr-min_cost) depends on the heading of the robot wrt the local map indices.
#     #Replace the negative sign with dot product between the robot's heading and local map indexing
#     goal.target_pose.pose.orientation = Quaternion(*(tf.transformations.quaternion_from_euler(0,0,3.1415, axes='sxyz')))

#     return goal


# def setGoalClient(delta_base, current_pose_listener):
    
#     #receive current position
#     rospy.sleep(1)
#     cur_positon, cur_rot = current_position(current_pose_listener)
    
#     #define desired position as PoseStamped message 
#     p = create_pose_stamp(delta_base, 'base_footprint')
    
#     #transform delta_base into map frame
#     # rate = rospy.Rate(10.0)
#     now = rospy.Time.now()
#     current_pose_listener.waitForTransform("/base_footprint", "/map", now, rospy.Duration(1.0))
#     delta_map = current_pose_listener.transformPose("/map", p)
#     #pass delta_map through SimpleActionClient
#     client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

#     client.wait_for_server()
#     goal = transform_goal_to_map(delta_map)

#     client.send_goal(goal)
#     wait = client.wait_for_result()

#     if not wait:
#         rospy.logerr("Action server not available!")
#         # rospy.signal_shutdowm("Action server not available!")
#     else:
#         results =  client.get_result()
#         current_position(current_pose_listener)
#         return results


# def delta_pose():
#     return (-0.5, 0.0, 0.0)

# def current_position(current_pose):
#     current_pose.waitForTransform("/map", "/base_footprint", rospy.Time(), rospy.Duration(5.0))
#     now = rospy.Time.now()
    
#     success = False
#     while not (success):
#         try:
#             current_pose.waitForTransform("/map", "/base_footprint", now, rospy.Duration(5.0))
#             (trans, rot) = current_pose.lookupTransform("/map", "/base_footprint", now)
#             success = True
#         except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
#             print("In exception")
#         rospy.sleep(0.5)
#     print(trans)
#     return trans, rot