#!/usr/bin/env python3
import rospy
from detector.detect import Detection

# Results
if __name__ == "__main__":
    rospy.init_node('jetbot_detect', anonymous=True)
    rate = rospy.Rate(15)
    detection = Detection()
    try:
        while not rospy.is_shutdown():
            detection.YOLOv5_inference() 
            rate.sleep()
    except (KeyboardInterrupt, StopIteration):
        pass

