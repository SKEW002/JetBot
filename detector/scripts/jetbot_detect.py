#!/usr/bin/env python

# ros import
import rospy
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import PoseStamped
from actionlib_msgs.msg import GoalStatusArray
from std_srvs.srv import SetBool, SetBoolResponse
from cv_bridge import CvBridge, CvBridgeError
import time

# obj detection
import cv2
import torch
import numpy as np
from math import atan, pi, tan
print("Using CUDA: ",torch.cuda.is_available())

# Inference
class Detection:
    def __init__(self):
        self.weights_path = rospy.get_param('~weights_path')

        self.model = torch.hub.load("ultralytics/yolov5","custom", path=self.weights_path)
        self.model.classes=[0]
        self.br = CvBridge()

        self.pub_target_angle = rospy.Publisher('/cmd_out/target_angle', Float32, queue_size=10)
        self.pub_cmd_vel= rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pub_goal = rospy.Publisher('/cmd_out/pose', PoseStamped, queue_size=10)
        self.pub_pwm = rospy.Publisher('/jetbot_motors/cmd_raw', String, queue_size=10)  # "left right"
        self.pub_goal_map = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        self.target_angle_msg = Float32()
        self.cmd_vel_msg = Twist()
        self.goal_msg = PoseStamped()
        self.pwm_msg = String()

        self.start_image = False

        rospy.Subscriber('/jetbot_camera/image', Image, self.image_callback)


        self.distance = 0
        self.depth = 0
        self.pub_the_msg = True 


        self.start_service = rospy.Service('detection_service', SetBool, self.trigger_publish)

        self.x0 = 0
        self.y0 = 0
        self.x1 = 0
        self.y1 = 0
        self.conf = 0
        self.obj = 30
        self.frame_num = 0
        self.current_angle = 0
        self.status = 0
    
    def image_callback(self, data):
        self.bgr_image = self.br.imgmsg_to_cv2(data,"bgr8")
        self.image_height, self.image_width, self.image_channel = self.bgr_image.shape # (480,640,3)
        self.start_image = True

    def laser_callback(self, msg):
        self.distance = msg.ranges[0]

    def move_base_status_callback(self, msg):
        try:
            self.status = msg.status_list[0].status
        except IndexError:
            pass

    def cmd_vel_raw_callback(self, msg):
        self.cmd_vel_msg = msg



    def inference(self):
        tol = 5
        results = self.model(self.bgr_image, size=320)  # includes NMS
        outputs = results.xyxy[0].cpu()
        if len(outputs) > 0:
            for i,detection in enumerate(outputs):
                self.x0 = int(outputs[i][0]) #xmin
                self.y0 = int(outputs[i][1]) #ymin
                self.x1 = int(outputs[i][2]) #xmax
                self.y1 = int(outputs[i][3]) #ymax
                self.conf = float(outputs[i][4])
                self.obj = int(outputs[i][5]) #object number
                self.center = ((self.x0+self.x1)//2,(self.y0+self.y1)//2)
                self.bgr_image = cv2.circle(self.bgr_image, [self.center[0], self.center[1]], 2,(0,0,255),2)



                if self.pub_the_msg == True:
                    #depth =  self.get_distance()
                    cv2.rectangle(self.bgr_image,(self.x0, self.y0),(self.x1,self.y1),(0,255,0),3)


            if self.center[1] > (self.image_height / 2 + tol):
                self.cmd_vel_msg.linear.z = -0.4

            elif self.center[1] < (self.image_height / 2  - tol) and self.center[1]>0:
                self.cmd_vel_msg.linear.z = 0.4

            else:
                self.cmd_vel_msg.linear.z = 0



        cv2.imshow("frame",self.bgr_image) 
        if cv2.waitKey(1) == ord('q'):  # q to quit
            cv2.destroyAllWindows()
            raise StopIteration  

        '''
        uint8 PENDING=0
        uint8 ACTIVE=1
        uint8 PREEMPTED=2
        uint8 SUCCEEDED=3
        uint8 ABORTED=4
        uint8 REJECTED=5
        uint8 PREEMPTING=6
        uint8 RECALLING=7
        uint8 RECALLED=8
        uint8 LOST=9
        '''

            
    def trigger_publish(self, request):
        if request.data:
            self.pub_the_msg = True
            return SetBoolResponse(True, 'Publishing data...')
        else:
            self.pub_the_msg = False
            return SetBoolResponse(False, 'Keep Quiet...')


    def get_distance(self): # depth image from: /realsense_d435/depth/image_raw
        self.depth_array = np.array(self.depth_image, dtype=np.float32)

        # u = (self.x0 + self.x1)//2 
        # v = (self.y0 + self.y1)//2

        u = self.center[0]
        v = self.center[1]

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



    def reset(self):
        self.center = (0,0)


# Results
if __name__ == "__main__":
    rospy.init_node('detect', anonymous=True)
    rate = rospy.Rate(15)
    detection = Detection()
    try:
        while not rospy.is_shutdown():
            if detection.start_image:
                
                detection.inference()
                #detection.pub_the_msg = True
            rate.sleep()

    except (KeyboardInterrupt, StopIteration):
        pass
