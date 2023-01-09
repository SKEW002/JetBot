#!/usr/bin/env python

# ros import
import rospy
from std_msgs.msg import Float32, Int16MultiArray, Empty
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

import time

# Inference
class Detection:
    def __init__(self):
        self.weights_path = rospy.get_param('~weights_path')

        self.model = torch.hub.load("ultralytics/yolov5","custom", path=self.weights_path)
        self.model.classes=[0]
        self.br = CvBridge()

        self.pub_cmd_vel= rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.target_angle_msg = Float32()
        self.cmd_vel_msg = Twist()
        self.goal_msg = PoseStamped()

        self.start_image = False

        rospy.Subscriber('/front_cam/camera/image', Image, self.image_callback)
        rospy.Subscriber('/scan', LaserScan, self.laser_callback)


        self.distance = 0
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
        self.image_height, self.image_width, self.image_chanel = self.bgr_image.shape # (480,640,3)
        self.start_image = True

    def laser_callback(self, msg):
        self.distance = msg.ranges[540]

    def move_base_status_callback(self, msg):
        try:
            self.status = msg.status_list[0].status
        except IndexError:
            pass

    def cmd_vel_raw_callback(self, msg):
        self.cmd_vel_msg = msg



    def inference(self): # left y +, left z +
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

                if self.center[1] > (self.image_height / 2 + tol):
                    self.cmd_vel_msg.linear.z = -0.1

                elif self.center[1] < (self.image_height / 2  - tol) and self.center[1]>0:
                    self.cmd_vel_msg.linear.z = 0.1

                else:
                    self.cmd_vel_msg.linear.z = 0


                if self.center[0] > (self.image_width / 2 + tol):
                	self.cmd_vel_msg.linear.y = -0.1
                	self.cmd_vel_msg.angular.z = -0.1

                elif self.center[0] < (self.image_width / 2  - tol) and self.center[0]>0:
                	self.cmd_vel_msg.linear.y = 0.1
                	self.cmd_vel_msg.angular.z = 0.1

                else:
                    self.cmd_vel_msg.linear.y = 0
                    self.cmd_vel_msg.angular.z = 0

                if self.distance > 2:
                	self.cmd_vel_msg.linear.x = 0.1


                if self.pub_the_msg == True:
                    # self.get_distance()
                    cv2.rectangle(self.bgr_image,(self.x0, self.y0),(self.x1,self.y1),(0,255,0),3)
                if self.conf > 0.7:
                    break
                else:
                    continue

            try:
                target_angle = atan((self.image_width / 2 - self.center[0]) / self.center[1]) * 180.0 / pi; # 640x480 ################################################## check
                target_angle += self.current_angle


                if self.distance >= 4:
                    x = 4
                else:
                    x = self.distance-0.5

                y = tan(target_angle * pi/180) * x


            except ZeroDivisionError:
                pass


        else:
        	self.cmd_vel_msg.angular.z = 0

        	
        self.pub_cmd_vel.publish(self.cmd_vel_msg)

        cv2.putText(self.bgr_image, str(self.distance),(480//2, 640//2), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
        self.bgr_image = cv2.circle(self.bgr_image, [640//2, 480//2], 2,(0,0,255),2)

        cv2.imshow("frame",self.bgr_image) 
        if cv2.waitKey(1) == ord('q'):  # q to quit
            cv2.destroyAllWindows()
            raise StopIteration  

            
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



    def reset(self):
        self.center = (0,0)


# Results
if __name__ == "__main__":
    rospy.init_node('detect', anonymous=True)
    rate = rospy.Rate(15)
    takeoff_pub = rospy.Publisher('/takeoff', Empty, queue_size=10)
    detection = Detection()
    try:
        takeoff_pub.publish(Empty())
        while not rospy.is_shutdown():
            if detection.start_image:
                
                detection.inference()
                #detection.pub_the_msg = True
            rate.sleep()

    except (KeyboardInterrupt, StopIteration):
        pass
