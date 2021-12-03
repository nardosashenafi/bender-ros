#! /usr/bin/python3

__author__ = 'NardosAshenafi'

from set_goal import *
from nav_msgs.msg import OccupancyGrid
import numpy as np
from map_msgs.msg import OccupancyGridUpdate
import matplotlib.pyplot as plt

class Costmap:
    def __init__(self):
        self.pf = None
        self.counter = 1

    def costmap_callback(self, data):
        cost = data.data

        eta = 0.1
        d0 = 100.0
        self.pf = np.zeros((80, 120))

        for i in range(120):
            for j in range(80):
                self.pf[j][i] = 1.0/2.0 * eta *(1.0/(1.0 / (cost[i*j] + 1e-3)) - 1.0/d0) ** 2.0
        # self.pf = tuple(1.0/2.0 * eta *(1.0/(1.0 / (c + 1e-3)) - 1.0/d0) ** 2.0 for c in cost)
        
        # if self.counter == 1:
        #     self.counter += 1
        #     print(self.pf[50])
        #     plt.plot(self.pf[0])
        #     plt.show()
            
       
    def costmap_subscriber(self):
        rospy.Subscriber("/bender_nav/local_costmap/costmap_updates", OccupancyGridUpdate, self.costmap_callback)
        

    def potential_field_goal(self):

        self.costmap_subscriber()
        thirieth_col    = [self.pf[i][30] for i in range(80)]
        min_cost_ind    = (np.argmin(thirieth_col), 30)
        min_cost        = np.min(thirieth_col)
        min_cost_goal   = tuple(np.multiply((30, min_cost_ind, 0), 0.01))
        return min_cost_goal
        

# def delta_pose():
#     potential_field()
#     return (-0.5, 0.0, 0.0)

if __name__ == '__main__':
    try:
        rospy.init_node('potential_field')
        cm = Costmap()
        # result = setGoalClient()
        # print(cm.pf)
        # if result:
        #     rospy.loginfo("Goal execution done!")
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")

