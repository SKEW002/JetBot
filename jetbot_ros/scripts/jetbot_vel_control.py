#!/usr/bin/env python
import rospy
import time

from Adafruit_MotorHAT import Adafruit_MotorHAT
from std_msgs.msg import String
from geometry_msgs.msg import Twist



class Jetbot_Control:
	def __init__(self):
		rospy.Subscriber('/jetbot_velocity_controller/cmd_vel', Twist, self.on_cmd_raw)
		rospy.Subscriber('/cmd_str', String, self.on_cmd_str)

		self.motor_driver = Adafruit_MotorHAT(i2c_bus=1)

		self.motor_left_ID = 1
		self.motor_right_ID = 2

		self.motor_left = self.motor_driver.getMotor(self.motor_left_ID)
		self.motor_right = self.motor_driver.getMotor(self.motor_right_ID)

		self.all_stop()



	# sets motor speed between [-1.0, 1.0]
	def set_speed(self, motor_ID, value):
		max_pwm = 115.0
		speed = int(min(max(abs(value * max_pwm), 0), max_pwm))

		if motor_ID == 1:
			motor = self.motor_left
			ina = 1
			inb = 0
		elif motor_ID == 2:
			motor = self.motor_right
			ina = 2
			inb = 3
		else:
			rospy.logerror('set_speed(%d, %f) -> invalid motor_ID=%d', motor_ID, value, motor_ID)
			return
		
		motor.setSpeed(speed)

		if value > 0:
			motor.run(Adafruit_MotorHAT.FORWARD)
			self.motor_driver._pwm.setPWM(ina,0,speed*16)
			self.motor_driver._pwm.setPWM(inb,0,0)
		else:
			motor.run(Adafruit_MotorHAT.BACKWARD)
			self.motor_driver._pwm.setPWM(ina,0,0)
			self.motor_driver._pwm.setPWM(inb,0,speed*16)
	                                


	# stops all motors
	def all_stop(self):
		self.motor_left.setSpeed(0)
		self.motor_right.setSpeed(0)

		self.motor_left.run(Adafruit_MotorHAT.RELEASE)
		self.motor_right.run(Adafruit_MotorHAT.RELEASE)

		self.motor_driver._pwm.setPWM(0,0,0)
		self.motor_driver._pwm.setPWM(1,0,0)
		self.motor_driver._pwm.setPWM(2,0,0)
		self.motor_driver._pwm.setPWM(3,0,0)

	# raw L/R motor commands (speed, speed)
	def on_cmd_raw(self, msg):

		linear_x = msg.linear.x 
		angular_z = msg.angular.z #left +ve, right -ve
		speed = [0,0]

		if abs(linear_x) > 0:
			speed[0] = 0.2/linear_x * abs(linear_x) + linear_x
			speed[1] = 0.2/linear_x * abs(linear_x) + linear_x

			#speed[0] = linear_x
			#speed[1] = linear_x

		if abs(angular_z) > 0:
			#speed[0] -= 0.3/angular_z * abs(angular_z)
			#speed[1] += 0.3/angular_z * abs(angular_z)

			speed[0] -= 0.05/angular_z * abs(angular_z) + angular_z
			speed[1] += 0.05/angular_z * abs(angular_z) + angular_z


		self.set_speed(self.motor_left_ID,  -float(speed[0]))
		self.set_speed(self.motor_right_ID,  -float(speed[1])) 

	# simple string commands (left/right/forward/backward/stop)
	def on_cmd_str(self, msg):
		rospy.loginfo(rospy.get_caller_id() + ' cmd_str=%s', msg.data)

		if msg.data.lower() == "right":
			self.set_speed(self.motor_left_ID,  -1.0)
			self.set_speed(self.motor_right_ID,  1.0) 
		elif msg.data.lower() == "left":
			self.set_speed(self.motor_left_ID,   1.0)
			self.set_speed(self.motor_right_ID, -1.0) 
		elif msg.data.lower() == "backward":
			self.set_speed(self.motor_left_ID,   1.0)
			self.set_speed(self.motor_right_ID,  1.0)
		elif msg.data.lower() == "forward":
			self.set_speed(self.motor_left_ID,  -1.0)
			self.set_speed(self.motor_right_ID, -1.0)  
		elif msg.data.lower() == "stop":
			self.all_stop()
		else:
			rospy.logerror(rospy.get_caller_id() + ' invalid cmd_str=%s', msg.data)


# initialization
if __name__ == '__main__':

	# setup motor controller

	controller = Jetbot_Control()

	# setup ros node
	rospy.init_node('jetbot_motors')	

	# start running
	rospy.spin()

	# stop motors before exiting
	controller.all_stop()

