
import rospy
import tf
import numpy as np
import math
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped

def transform_lm_map(o_lm_map):

    transformer = tf.TransformerROS(True, rospy.Duration(5.0))
    trans = [o_lm_map.position.x, o_lm_map.position.y, o_lm_map.position.z]
    rot = [o_lm_map.orientation.x, o_lm_map.orientation.y, o_lm_map.orientation.z, o_lm_map.orientation.w]
    return transformer.fromTranslationRotation(trans, rot)

def transform_r_map(listener):

    trans, rot = current_position(listener)
    o_r_map = create_pose_stamp(trans, 'map')
    o_r_map.pose.orientation.x, o_r_map.pose.orientation.y, o_r_map.pose.orientation.z, o_r_map.pose.orientation.w = rot
    
    transformer = tf.TransformerROS(True, rospy.Duration(5.0))
    return o_r_map, transformer.fromTranslationRotation(trans, rot)

def lm_to_map(v, T_lm_map):
    v1 = np.concatenate((v, [1]))
    v_map = np.dot(T_lm_map, v1)
    return v_map[0:3]

def map_to_lm(v, T_lm_map):
    v1 = np.concatenate((v, [1]))
    v_lm =  np.dot(np.linalg.inv(T_lm_map), v1)
    return v_lm[0:3]

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

def create_pose_stamp(delta_base, frame):
    p                   = PoseStamped()
    p.header.frame_id   = frame
    p.header.stamp      = rospy.Time.now()

    p.pose.position.x   = delta_base[0]
    p.pose.position.y   = delta_base[1]
    p.pose.position.z   = delta_base[2]
    
    return p