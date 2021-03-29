#!/usr/bin/env python3
# license removed for brevity
import rospy
from chee_control import cheetah_control
import numpy as np
import sys, os
import time
import sympy as sym
sys.path.append("../..")
from quadruped_kinematics import quadruped_kinematics

if __name__ == '__main__':
	cheetah_control_pos = cheetah_control(type_of_control = 'torque')

	# q_goal_LB = np.array([[0],
	# 					  [0],
	# 					  [0]], dtype = np.float)

	Kp = np.array([[10,0,0],
				   [0,5,0],
				   [0,0,2]])
	Kd = np.array([[0.3,0,0],
				   [0,0.2,0],
				   [0,0,0.1]])

	q_dot_goal_LB = np.array([[0],
						  [0],
						  [0]], dtype = np.float)
	LB_leg = ['left_back_motor_leg','left_back_leg_hip', 'left_back_hip_calf']
	RB_leg = ['right_back_motor_leg','right_back_leg_hip', 'right_back_hip_calf']
	LF_leg = ['left_forward_motor_leg','left_forward_leg_hip', 'left_forward_hip_calf']
	RF_leg = ['right_forward_motor_leg','right_forward_leg_hip', 'right_forward_hip_calf']

	start_time = time.time()

	links_sizes_mm = [162.75, 65, 72.25, 82.25, 208, 159, 58.5]
	quad_kin = quadruped_kinematics([link_size / 1000 for link_size in links_sizes_mm])
	t = sym.symbols("t")
	x_des,x_dot_des, y_des, y_dot_des, z_des, z_dot_des = quad_kin.get_cart_trajectory_by_sym(quad_kin.links_size[1] + quad_kin.links_size[3], -(quad_kin.links_size[0] + quad_kin.links_size[2]), -(quad_kin.links_size[4] + quad_kin.links_size[5] + quad_kin.links_size[6]) +  0.18 + 0.15*sym.sin(1*t))
	while not rospy.is_shutdown():
		try:
			if not cheetah_control_pos.joints_positions:
				continue
			t = time.time() - start_time

			# q_goal_LF = np.array([[0],
			# 			  [np.pi/4*np.sin(3*t)],
			# 			  [-np.pi/4*np.sin(3*t)]], dtype = np.float)
						  
			q_cur_LF = np.array([[0],
						  [0],
						  [0]], dtype = np.float)
			
			q_dot_cur_LF = np.array([[0],
						  [0],
						  [0]], dtype = np.float)

			for idx, motor in enumerate(LF_leg):
				q_cur_LF[idx] = cheetah_control_pos.joints_positions[motor]
				q_dot_cur_LF[idx] = cheetah_control_pos.joints_velocities[motor]
			
			x_des_,x_dot_des_, y_des_, y_dot_des_, z_des_, z_dot_des_ = x_des(t),x_dot_des(t), y_des(t), y_dot_des(t), z_des(t), z_dot_des(t)

			pos_des = [x_des_, y_des_, z_des_]
			vel_des = [x_dot_des_, y_dot_des_, z_dot_des_]
			q_des = quad_kin.leg_ik(base = quad_kin.base_LF, pos = pos_des)

			q_dot_goal_LF = - np.dot(np.linalg.pinv(quad_kin.LF_leg_Jac_sym(q_des[0], q_des[1], q_des[2])), np.array([[vel_des[0]],																												[vel_des[1]],
																										[vel_des[2]]]))
			
			q_goal_LF = - np.array([[q_des[0]],
                                [q_des[1]],
                                [q_des[2]]]) # "-" is because the z-axis is inverted for each motor

			U = np.dot(Kp,q_goal_LF - q_cur_LF) + np.dot(Kd,q_dot_goal_LF - q_dot_cur_LF)
			cheetah_control_pos.move_joint(LF_leg[0], U[0])
			cheetah_control_pos.move_joint(LF_leg[1], U[1])
			cheetah_control_pos.move_joint(LF_leg[2], U[2])
			cheetah_control_pos.rate.sleep()
		except rospy.ROSInterruptException:
			pass