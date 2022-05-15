#! /usr/bin/python3

__author__ = 'NardosAshenafi'

import rospy
import actionlib
import tf
from nav_base import *
from set_goal_arc import Costmap 

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

GPS_GOAL = [[50, 50, 0]]
GOAL = "gps"
# GOAL = "arc"

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
    # p = create_pose_stamp(delta_base, 'base_footprint')
    p = delta_base 

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


if __name__ == '__main__':
    
    gps_goal_count = 0
    try:
        rospy.init_node('set_goal_client')

        current_pose_listener  = tf.TransformListener()

        if GOAL == "arc":
            cm = Costmap(current_pose_listener)

            print("Ready to send goal")
            cm.costmap_subscriber()
            # result = setGoalClient(cm.q_g_r, current_pose_listener)

            # if result:
            #     rospy.loginfo("Goal execution done!")
        elif GOAL == "gps":

            current_gps_goal    = create_pose_stamp(GPS_GOAL[gps_goal_count], 'base_footprint')
            #IF THE GOAL IS NOT WRT THE FOOTPRINT FRAME, TRANSFORM TO FOOTPRINT HERE AND PASS TO setGoalClient
            result              = setGoalClient(current_gps_goal, current_pose_listener)

            # if result:
            #     rospy.loginfo("Goal execution done!")      
            #     gps_goal_count += 1        

        rospy.sleep(10)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
