#!/usr/bin/env python
from __future__ import print_function

import roslib
roslib.load_manifest('detector')
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.width= 640 #int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    self.height= 480 #int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    self.writer= cv2.VideoWriter('test_video.mp4', cv2.VideoWriter_fourcc(*'DIVX'), 20, (self.width,self.height))
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/front_cam/camera/image",Image,self.callback)

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    self.writer.write(cv_image)
    cv2.imshow("Image window", cv_image)
    cv2.waitKey(3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
    except CvBridgeError as e:
      print(e)


if __name__ == '__main__':
    
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        ic.writer.release()
        cv2.destroyAllWindows()
