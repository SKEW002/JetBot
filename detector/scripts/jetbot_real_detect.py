#!/usr/bin/env python

# ros import
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from move_base_msgs.msg import RecoveryStatus
from std_srvs.srv import SetBool, SetBoolResponse
from cv_bridge import CvBridge, CvBridgeError
import time

from detection_msgs.msg import BoundingBox, BoundingBoxes
from detector.detect import RobotStateEnum, MoveBaseStateEnum

import numpy as np
from math import atan, pi, sin, tan, isnan, isinf

# Inference


class JetbotDetect:
    def __init__(self):
        self.br = CvBridge()

        self.pub_target_angle = rospy.Publisher('/cmd_out/target_angle', Float32, queue_size=1)
        self.pub_goal = rospy.Publisher('/cmd_out/pose', PoseStamped, queue_size=1)
        self.pub_goal_map = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.pub_cmd_vel = rospy.Publisher('/jetbot_velocity_controller/cmd_vel', Twist, queue_size=1)

        self.target_angle_msg = Float32()
        self.goal_msg = PoseStamped()
        self.cmd_vel_msg = Twist()

        self.start_image = False
        self.found_object = False
        rospy.Subscriber('/zedm/zed_node/depth/depth_registered', Image, self.depth_callback)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)
        rospy.Subscriber('/move_base/recovery_status', RecoveryStatus, self.move_base_recovery_callback)
        rospy.Subscriber('/object_bounding_boxes', BoundingBoxes, self.bounding_boxes_callback)

        self.no_object_count = 0
        self.recovery = False
        self.distance = 0

        self.image_width = 672
        self.image_height = 376

        self.current_angle = 0
        self.robot_status = RobotStateEnum.HomeState
        self.move_base_status = MoveBaseStateEnum.PENDING

    def depth_callback(self, data):
        self.depth_image = self.br.imgmsg_to_cv2(data, desired_encoding="32FC1")


    def bounding_boxes_callback(self, data):
        bounding_boxes = data
        self.center = (0,0)
        if len(bounding_boxes) == 0:
            self.no_object_count += 1
        else:
            self.no_object_count = 0


        for i, bounding_box in enumerate(bounding_boxes.bounding_boxes):
            # print(bounding_box)
            if bounding_box.Class == "door":
                self.found_object = True
                if self.robot_status == RobotStateEnum.HomeState and self.move_base_status == MoveBaseStateEnum.PENDING:
                    self.robot_status = RobotStateEnum.Ingressing

                self.center = ((bounding_box.xmin + bounding_box.xmax)//2,(bounding_box.ymin + bounding_box.ymax)//2)
                self.target_angle_calculation(self.image_width/2 - self.center[0])
                # self.target_angle = atan((self.image_width / 2 - self.center[0]) / self.center[1])
                # print(self.target_angle * 180 / pi)

                # if abs(self.target_angle) < 7 * pi/180:
                #     self.target_angle = 0

                # elif self.target_angle > 40 * pi/180:
                #     pass

                self.distance = self.get_distance()

                break


    def move_base_status_callback(self, msg):
        # print(self.move_base_status)
        try:
            self.move_base_status = MoveBaseStateEnum(int(msg.status_list[0].status))

        except IndexError:
            pass


    def move_base_recovery_callback(self,msg):
        if msg.current_recovery_number:
            self.recovery = True


    def robot_action(self):
        if self.robot_status == RobotStateEnum.Ingressing:
            rospy.loginfo("Ingressing")
            self.robot_ingress()

        elif self.robot_status == RobotStateEnum.Egress:
            rospy.loginfo("Egressing")
            self.robot_egress()


    def robot_ingress(self):
        if self.distance >= 1:
            x = 1

        else:
            x = self.distance-0.3

        y = sin(self.target_angle) * x / 2.5 # down scaling
        #rospy.loginfo("%d", self.status)

        # if self.move_base_status == MoveBaseStateEnum.PREEMPTED:
        
        if self.found_object and (self.move_base_status == MoveBaseStateEnum.SUCCEEDED or self.move_base_status == MoveBaseStateEnum.PENDING):
            self.delay(5)
            rospy.loginfo("Publish Pose")
            self.publish_pose(x, y, self.target_angle)


        if (self.move_base_status == MoveBaseStateEnum.SUCCEEDED and self.no_object_count >= 20) or self.recovery == True:
            self.robot_status = RobotStateEnum.Egress


    def robot_egress(self):
        self.robot_status = RobotStateEnum.Egressing
        self.publish_pose(0, 0, 0, frame_id="map",tf2_listener=False)

        if self.move_base_status == MoveBaseStateEnum.SUCCEEDED:
            self.robot_status = RobotStateEnum.HomeState


    def get_distance(self):
        self.depth_array = np.array(self.depth_image, dtype=np.float32)

        # u = (self.x0 + self.x1)//2 
        # v = (self.y0 + self.y1)//2

        u = self.center[0]
        v = self.center[1]

        # print(self.depth_array[v,u])
        if isnan(self.depth_array[v,u]):
            return 0.7

        elif isinf(self.depth_array[v,u]):
            return 6.0
        else:
            return self.depth_array[v,u]


    def publish_pose(self, x, y, yaw, frame_id="zedm_left_camera_frame", tf2_listener=True):  # x=front, link=realsense_d435_link
        roll = 0 
        pitch = 0

        # eular to quaternion conversion
        #qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)   
        #qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        
        self.goal_msg.header.seq = 1
        self.goal_msg.header.stamp = rospy.Time.now()
        self.goal_msg.header.frame_id = frame_id
        self.goal_msg.pose.position.z = 0
        self.goal_msg.pose.orientation.x = 0
        self.goal_msg.pose.orientation.y = 0

        self.goal_msg.pose.position.x = x
        self.goal_msg.pose.position.y = y
        self.goal_msg.pose.orientation.z = qz
        self.goal_msg.pose.orientation.w = qw

        if tf2_listener:
            self.pub_goal.publish(self.goal_msg)

        else:
            self.pub_goal_map.publish(self.goal_msg)


    def delay(self, delay_second):
        start = time.time()
        while True:
            now = time.time()
            if (now - start) > delay_second:
                break
        return

    def stop_cmd(self):
        stop_cmd_vel = Twist()
        stop_cmd_vel.linear.x = 0
        stop_cmd_vel.linear.y = 0
        stop_cmd_vel.linear.z = 0

        stop_cmd_vel.angular.x = 0
        stop_cmd_vel.angular.y = 0
        stop_cmd_vel.angular.z = 0

        self.pub_cmd_vel.publish(stop_cmd_vel)

    def target_angle_calculation(self, x_pixel):
        self.camera_fov = 87 # degree
        degree_per_pixel = self.camera_fov/self.image_width
        self.target_angle = x_pixel * degree_per_pixel
        self.target_angle *= pi/180 # covert to radian
        
        # print(self.target_angle)



# Results
if __name__ == "__main__":
    rospy.init_node('jetbot_real_detect', anonymous=True)
    rate = rospy.Rate(15)
    jetbot_detect = JetbotDetect()
    try:
        while not rospy.is_shutdown():
            jetbot_detect.robot_action()
            if jetbot_detect.move_base_status == MoveBaseStateEnum.PREEMPTED:
                jetbot_detect.stop_cmd()
            rate.sleep()
    except (KeyboardInterrupt, StopIteration):
        pass