import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

class PID:
    def __init__(self, kp=1, ki=0, kd=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.cur_error = 0
        self.prev_error = 0
        self.error_list = []
        self.desired_value = 0
        self.current_value = 0
        self.cur_time = 0
        self.prev_time = 0
        self.time_list = []
        self.dedt = 0

        self.max_sample = 10
        self.initial_time = time.time()

    def update(self):
        self.cur_time = time.time()
        self.cur_error = self.desired_value - self.current_value

        if len(self.error_list) < self.max_sample:
            self.error_list.append(self.cur_error)
            self.time_list.append(self.cur_time)
        else:
            del self.error_list[0]
            del self.time_list[0]
            self.error_list.append(self.cur_error)
            self.time_list.append(self.cur_time)


        self.dedt = (self.cur_error - self.prev_error) / (self.cur_time - self.prev_time)
        self.prev_time = self.cur_time
        self.prev_error = self.cur_error


    def output(self):
        P = kp * self.cur_error
        I = ki * (sum(self.error_list) * (sum(self.time_list) - self.initial_time))
        D = kd * self.dedt

        return P, I, D


    def reset(self):
        self.cur_error = 0
        self.prev_error = 0
        self.error_list = []
        self.desired_value = 0
        self.current_value = 0
        self.cur_time = 0
        self.prev_time = 0
        self.time_list = []
        self.dedt = 0
