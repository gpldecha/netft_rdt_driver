#!/usr/bin/python
import rospy
from geometry_msgs.msg import WrenchStamped, Wrench
import numpy as np
from dynamic_reconfigure.server import Server
from force_polishing.cfg import force_filterConfig
# general filter that works with anything supporting
# 1) inc-mult with scalar
# 2) multiplication
# 3) inc-addition


class ExponentialSmoother(object):

    def __init__(self, alpha):
        self.y = None
        self.alpha = alpha

    def update(self, x):
        if self.y is None:
            self.y = x
        else:
            self.y *= (1 - self.alpha)
            self.y += self.alpha * x
        return self.y


def wrench_to_np(msg):
    arr = np.zeros(6)
    arr[0], arr[1], arr[2] = msg.force.x, msg.force.y, msg.force.z
    arr[3], arr[4], arr[5] = msg.torque.x, msg.torque.y, msg.torque.z
    return arr


def np_to_wrench(arr):
    msg = Wrench()
    msg.force.x, msg.force.y, msg.force.z = arr[0], arr[1], arr[2]
    msg.torque.x, msg.torque.y, msg.torque.z = arr[3], arr[4], arr[5]
    return msg


class ForceFilterNode(object):

    def __init__(self):
        rospy.init_node("force_filter")
        alpha = rospy.get_param("~smoothing_factor",0.1)
        self.exp_smoother = ExponentialSmoother(alpha)
        input_topic = rospy.get_param("~input_topic","netft_node/netft_data")
        output_topic = rospy.get_param("~output_topic","filtered_wrench")
        rospy.Subscriber(input_topic, WrenchStamped, self.input_callback)
        self.wrench_pub = rospy.Publisher(
            output_topic, WrenchStamped, queue_size=100)
        Server(force_filterConfig, self.dr_callback)
        rospy.spin()

    def input_callback(self, msg):
        filtered_wrench = self.exp_smoother.update(
            wrench_to_np(msg.wrench))
        pub_msg = WrenchStamped()
        pub_msg.header.frame_id = msg.header.frame_id
        pub_msg.header.stamp = msg.header.stamp
        pub_msg.wrench = np_to_wrench(filtered_wrench)
        self.wrench_pub.publish(pub_msg)

    def dr_callback(self, config, level):
        self.exp_smoother.alpha = config["alpha"]
        return config

if __name__=="__main__":
    node = ForceFilterNode()
