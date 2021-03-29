#!/usr/bin/env python3
# license removed for brevity
import rospy
from chee_control import cheetah_control
import numpy as np
import sys, os
import time
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))
from quadruped_kinematics import quadruped_kinematics
if __name__ == '__main__':
	cheetah_control_pos = cheetah_control(type_of_control = 'position')

	LB_leg = ['left_back_motor_leg','left_back_leg_hip', 'left_back_hip_calf']
	RB_leg = ['right_back_motor_leg','right_back_leg_hip', 'right_back_hip_calf']
	LF_leg = ['left_forward_motor_leg','left_forward_leg_hip', 'left_forward_hip_calf']
	RF_leg = ['right_forward_motor_leg','right_forward_leg_hip', 'right_forward_hip_calf']

	start_time = time.time()
	while not rospy.is_shutdown():
		try:
			if not cheetah_control_pos.joints_positions:
				continue
			t = time.time() - start_time

			cheetah_control_pos.move_joint(LB_leg[0], 0)
			cheetah_control_pos.move_joint(LB_leg[1], np.pi/4*np.sin(3*t))
			cheetah_control_pos.move_joint(LB_leg[2], -np.pi/4*np.sin(3*t))
			cheetah_control_pos.rate.sleep()
		except rospy.ROSInterruptException:
			pass