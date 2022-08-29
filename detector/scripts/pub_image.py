#!/usr/bin/env python3
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import nanocamera as nano



cap = nano.Camera(flip=0, width=3280//8, height=2464//8, fps=30)

bridge = CvBridge()

'''
def callback(BoundingBoxes):
	rospy.loginfo("I heard", BoundingBoxes)
'''

def talker():
	pub = rospy.Publisher('/jetbot_camera/image', Image, queue_size = 10)
	rospy.init_node('image', anonymous = False)
	rate = rospy.Rate(10)
	while not rospy.is_shutdown() and cap.isReady():
		#sub = rospy.Subscriber('/boundingbox', BoundingBoxes, callback)
		try:
			frame = cap.read()
			print(type(frame))
			msg = bridge.cv2_to_imgmsg(frame, "bgr8")
			pub.publish(msg)
			#cv2.imshow("frame",frame)
			if cv2.waitKey(10) & 0xFF == ord('q'):
				break

		except KeyboardInterrupt:
			cap.release()
			cv2.destroyAllWindows()


if __name__ == '__main__':
	try:
		talker()
	except rospy.ROSInterruptException:
		pass

