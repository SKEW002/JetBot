import rospy
import time
from math import sqrt
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class LinearVel:

    def __init__(self):
        rospy.Subscriber('/zed2/zed_node/odom',Odometry,self.odomCallback)

        self.odom_msg = Float32()
        self.start_odom = False
        self.odom_pub = rospy.Publisher('/cmd_out/odom', Float32 , queue_size=10)
        self.pose_x = 0
        self.pose_y = 0


    def linear_vel(self):
        prev_x = self.pose_x
        prev_y = self.pose_y

        start = time.time()
        timer = time.time()
        #rate = rospy.Rate(15)

        while not rospy.is_shutdown():
            if self.start_odom:
                if(start - timer) > 0.0666:
                    start = time.time()
                    distance_travelled = (sqrt(pow((self.pose_x - prev_x),2) + pow((self.pose_y - prev_y),2)))*100
                    velocity = distance_travelled/(abs(start-timer))
                    if velocity < 5:
                        velocity =  0.0

                    self.odom_msg.data = velocity # linear vel in cm/s
                    prev_x = self.pose_x
                    prev_y = self.pose_y
                    timer = time.time()
                    self.odom_pub.publish(self.odom_msg)

                else:
                    start = time.time()

                #rate.sleep()



    def odomCallback(self, odom):
        self.pose_x = odom.pose.pose.position.x
        self.pose_y = odom.pose.pose.position.y
        self.start_odom = True


if __name__ == '__main__':
    rospy.init_node('linear_vel', anonymous=True)
    linear_vel = LinearVel()
    try:
        linear_vel.linear_vel()
        #rospy.spin()

    except (KeyboardInterrupt, StopIteration):
        pass