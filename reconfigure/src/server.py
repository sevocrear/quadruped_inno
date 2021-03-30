#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from reconfigure.cfg import configConfig

def callback(config, level):
    rospy.loginfo("""Reconfigure Request: {Kp_leg}, {Kp_hip},\ 
          {Kp_calf}, {Kd_leg}, {Kd_hip}, {Kd_calf}, {zero_pose}, {x_body_des}, {y_body_des}, {z_body_des}, {R_des}, {P_des}, {Y_des}""".format(**config))
    return config

if __name__ == "__main__":
    rospy.init_node("dynamic_reconfigure", anonymous = False)

    srv = Server(configConfig, callback)
    rospy.spin()