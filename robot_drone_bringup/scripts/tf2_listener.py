#!/usr/bin/env python
import rospy
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32
import math
import time

class poseros:
    def __init__(self):
        self.pose_sub = rospy.Subscriber('/cmd_out/pose',  PoseStamped ,self.poseCallback, queue_size=1)
        self.pose = PoseStamped()
        self.pose_init = False

    def poseCallback(self, pose):
        self.pose = pose
        self.pose_init = True

if __name__ == '__main__':
    rospy.init_node('tf2_listener')
    pose_ros = poseros()
    pose_msg = PoseStamped()
    rate = rospy.Rate(3)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    prev_seq = -1
    while not rospy.is_shutdown(): 
        if pose_ros.pose_init :
            pose_msg = pose_ros.pose
            if pose_msg.header.seq != prev_seq:
                prev_seq = pose_msg.header.seq
                try:
                    trans = tfBuffer.lookup_transform('map',pose_msg.header.frame_id, rospy.Time()) #Providing rospy.Time(0) will just get us the latest available transform.

                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException,tf2_ros.InvalidArgumentException):
                    rate.sleep()
                    continue

                pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_msg, trans)

                pose_transformed.pose.position.z = 0  # only x,y is needed in 2d costmap

                pose_transformed.pose.orientation.x = 0  # only yaw angle will be cosidered
                pose_transformed.pose.orientation.y = 0
                goal_pub.publish(pose_transformed)
            rate.sleep()
