#!/usr/bin/env python3
# license removed for brevity
import rospy
from chee_control import cheetah_control
import numpy as np
import sys, os

if __name__ == '__main__':
	# print(os.path.realpath(__file__))
	cheetah_control_pos = cheetah_control(type_of_control = 'torque')
	# chain = kp.build_chain_from_urdf(open(os.path.join(os.path.dirname(sys.argv[0]) ,os.path.relpath("../../quadruped_robot/urdf/quadruped_robot.urdf"))).read().encode())

	goal = [0.0,0.0,0.0]*4
	K = [50, 50,10]
	d = [2, 2, 0.1]
	joints = list(cheetah_control_pos.joints.keys())
	while not rospy.is_shutdown():
		try:
			if not cheetah_control_pos.joints_positions:
				continue
			for i in range(0,6,3):
				idx = 0+i,1+i,2+i
				torque_0 = K[0]*(goal[idx[0]]-cheetah_control_pos.joints_positions[cheetah_control_pos.joints[joints[idx[0]]][0]]) + d[0]*(0-cheetah_control_pos.joints_velocities[cheetah_control_pos.joints[joints[idx[0]]][0]])

				torque_1 = K[1]*(goal[idx[1]]-cheetah_control_pos.joints_positions[cheetah_control_pos.joints[joints[idx[1]]][0]]) + d[1]*(0-cheetah_control_pos.joints_velocities[cheetah_control_pos.joints[joints[idx[1]]][0]])

				torque_2 = K[2]*(goal[idx[2]]-cheetah_control_pos.joints_positions[cheetah_control_pos.joints[joints[idx[2]]][0]]) + d[2]*(0-cheetah_control_pos.joints_velocities[cheetah_control_pos.joints[joints[idx[2]]][0]])

				cheetah_control_pos.move_joint(joints[idx[0]], torque_0)
				cheetah_control_pos.move_joint(joints[idx[1]], torque_1)
				cheetah_control_pos.move_joint(joints[idx[2]], torque_2)
			# print(cheetah_control_pos.body_position)
			# print(cheetah_control_pos.joints_positions)
			
			cheetah_control_pos.rate.sleep()
		except rospy.ROSInterruptException:
			pass