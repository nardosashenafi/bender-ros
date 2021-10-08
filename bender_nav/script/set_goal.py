#! /usr/bin/python3

__author__ = 'NardosAshenafi'
import roslib
roslib.load_manifest('bender_nav')
import rospy
import actionlib
import tf2_ros

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler

def setGoalClient():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)

    client.wait_for_server()
    #receive current position
    #TODO: subscribe to current location in map frame
    rospy.Subscriber('move_base/feedback', MoveBaseFeedback)

    point = set_goal_on_arc()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id='map'
    goal.target_pose.header.stamp = rospy.Time.now()

    #pass goal with normalized quaternion
    quat = Quaternion(*(quaternion_from_euler(0,0,0, axes='sxyz')))
    goal.target_pose.pose = Pose(Point(*point), quat)

    client.send_goal(goal)
    wait = client.wait_for_result()

    if not wait:
        rospy.logerr("Action server not available!")
        # rospy.signal_shutdowm("Action server not available!")
    else:
        return client.get_result()


def set_goal_on_arc():
    #TODO: point is in base_foot_print frame. Transform to map frame
    #TODO: use occupancy grid to find a valley for the goal

    
    x = 0.0
    y = 0.0
    x_goal = x + 0.1
    y_goal = y + 0.0
    z_goal = 0.0

    # tfBuffer = tf2_ros.Buffer()
    # transf = tfBuffer.lookup_transform('map', 'base_footprint', rospy.Time(0))    #from base_link to map
    # x_prime = transf.transform.translation.x
    # y_prime = transf.transform.translation.y
    
    return [x_goal, y_goal, z_goal]


if __name__ == '__main__':
    try:
        rospy.init_node('set_goal_client')
        result = setGoalClient()
        if result:
            rospy.loginfo("Goal execution done!")
        # rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")


##
# listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(10.0))
# trans, rot = listener.lookupTransform("/map","/base_link", rospy.Time(0))
##
# client.send_goal(goal)