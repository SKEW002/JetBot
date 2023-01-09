#!/usr/bin/env python

# ros import
import rospy
from std_msgs.msg import Float32, Empty, UInt8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan, Range
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_srvs.srv import SetBool, SetBoolResponse

import numpy as np
from math import atan, pi, tan

from detection_msgs.msg import BoundingBox, BoundingBoxes
from detector.detect import RobotStateEnum, MoveBaseStateEnum


class Detection:
    def __init__(self):

        # Publisher
        self.pub_target_angle = rospy.Publisher('/cmd_out/target_angle', Float32, queue_size=1)
        self.pub_cmd_vel= rospy.Publisher('/uav1/cmd_vel', Twist, queue_size=1)
        self.pub_goal = rospy.Publisher('/cmd_out/pose', PoseStamped, queue_size=1)
        self.pub_goal_map = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)

        # Messages
        self.target_angle_msg = Float32()
        self.cmd_vel_msg = Twist()
        self.goal_msg = PoseStamped()

        # Subscriber
        rospy.Subscriber('/object_bounding_boxes', BoundingBoxes, self.bounding_boxes_callback)
        rospy.Subscriber('/uav1/scan', LaserScan, self.laser_callback)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.move_base_status_callback)
        rospy.Subscriber('/cmd_vel_raw', Twist, self.cmd_vel_raw_callback)
        rospy.Subscriber('/sonar_height', Range, self.sonar_height_callback)

        self.pub_the_msg = True

        # Coditions
        self.found_object = False
        self.multiple_target = False
        self.start_detect = True

        self.drone_state = RobotStateEnum.HomeState
        self.start_service = rospy.Service('detection_service', SetBool, self.trigger_publish)

        self.image_width = 640
        self.image_height = 480

        self.current_angle = 0
        self.move_base_status = MoveBaseStateEnum.PENDING
        self.distance = 0
        self.center = (0,0)
    
    def bounding_boxes_callback(self, data):
        bounding_boxes = data
        print(len(bounding_boxes.bounding_boxes))

        if len(bounding_boxes.bounding_boxes) > 1:
            self.multiple_target = True
        else:
            self.multiple_target = False

        self.start_detect = True

        for i, bounding_box in enumerate(bounding_boxes.bounding_boxes):
            # print(bounding_box)
            if bounding_box.Class == "stairs":
                if self.move_base_status == MoveBaseStateEnum.SUCCEEDED:
                    self.drone_state = RobotStateEnum.TravelStairs

                self.center = ((bounding_box.xmin + bounding_box.xmax)//2,(bounding_box.ymin + bounding_box.ymax)//2)



    def laser_callback(self, msg):
        self.distance = msg.ranges[540]



    def move_base_status_callback(self, msg):
        try:
            self.move_base_status = MoveBaseStateEnum(int(msg.status_list[0].status))
        except IndexError:
            pass


    def cmd_vel_raw_callback(self, msg):
        self.cmd_vel_msg = msg


    def sonar_height_callback(self, msg):
        self.height = msg.range # meter

        if self.drone_state == RobotStateEnum.TravelStairs:
            if self.height < 1.3:
                self.cmd_vel_msg.linear.z = 0.2

            elif self.height > 1.4:
                self.cmd_vel_msg.linear.z = -0.2

            else:
                self.cmd_vel_msg.linear.z = 0



    def drone_action(self):
        if self.start_detect:

            if self.drone_state == RobotStateEnum.TravelStairs:
                self.travel_stairs()

            # if self.move_base_status == MoveBaseStateEnum.SUCCEEDED:
            #     self.publish_pose(0, 0, 0, frame_id="map",tf2_listener=False)
            # else:
            #     pass

            self.pub_cmd_vel.publish(self.cmd_vel_msg)

        self.center = (0,0)

            
    def trigger_publish(self, request):
        if request.data:
            self.pub_the_msg = True
            return SetBoolResponse(True, 'Publishing data...')
        else:
            self.pub_the_msg = False
            return SetBoolResponse(False, 'Keep Quiet...')


    def get_distance(self):
        self.depth_array = np.array(self.depth_image, dtype=np.float32)

        u = (self.x0 + self.x1)//2 
        v = (self.y0 + self.y1)//2

        # print(self.depth_array[v,u])
        if isnan(self.depth_array[v,u]):
            return 0.7

        elif isinf(self.depth_array[v,u]):
            return 2.0
        else:
            return self.depth_array[v,u]


    def publish_pose(self, x, y, yaw, frame_id="front_cam_link", tf2_listener=True):  # x = front
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


    def ingree_egress_height_control(self):
    
        if self.center[1] > (self.image_height / 2 + tol):
            self.cmd_vel_msg.linear.z = -0.4

        elif self.center[1] < (self.image_height / 2  - tol) and self.center[1] > 0:
            self.cmd_vel_msg.linear.z = 0.4

        else:
            self.cmd_vel_msg.linear.z = 0


    def travel_stairs(self):
        if self.center[0] != 0 and self.center[1] != 0:
            try:
                target_angle = atan((self.image_width / 2 - self.center[0]) / self.center[1])
                target_angle += self.current_angle  

                if self.distance >= 10:
                    x = 10

                else:
                    x = self.distance-0.5

                y = tan(target_angle) * x / 1.6

                y = (abs(y) * 1/y) * 0.2
                target_angle *= 180/pi
                if abs(target_angle) < 7:
                    y = 0

                print(target_angle, y)

                if self.move_base_status == MoveBaseStateEnum.SUCCEEDED:
                    self.publish_pose(0.8, y, 0)
            except ZeroDivisionError:
                pass


            # if self.move_base_status == MoveBaseStateEnum.PENDING:
            #     self.publish_pose(x+2, y, target_angle)


    def reset(self):
        self.center = (0,0)


# Results
if __name__ == "__main__":
    rospy.init_node('detect', anonymous=True)
    rate = rospy.Rate(15)
    takeoff_pub = rospy.Publisher('/takeoff', Empty, queue_size=1)
    takeoff_pub.publish(Empty())
    detection = Detection()

    try:
        while not rospy.is_shutdown():
                
            detection.drone_action()
            rate.sleep()

    except (KeyboardInterrupt, StopIteration):
        pass
