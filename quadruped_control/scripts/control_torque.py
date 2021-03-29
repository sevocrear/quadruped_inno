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

	traj_des_LF = quad_kin.get_cart_trajectory_by_sym(quad_kin.links_size[1] + quad_kin.links_size[3], -(quad_kin.links_size[0] + quad_kin.links_size[2]), -(quad_kin.links_size[4] + quad_kin.links_size[5] + quad_kin.links_size[6]) +  0.18 + 0.15*sym.sin(1*t))

	traj_des_LB = quad_kin.get_cart_trajectory_by_sym(quad_kin.links_size[1] + quad_kin.links_size[3], (quad_kin.links_size[0] + quad_kin.links_size[2]), -(quad_kin.links_size[4] + quad_kin.links_size[5] + quad_kin.links_size[6]) +  0.18 + 0.15*sym.sin(1*t))

	traj_des_RF = quad_kin.get_cart_trajectory_by_sym(-(quad_kin.links_size[1] + quad_kin.links_size[3]), -(quad_kin.links_size[0] + quad_kin.links_size[2]), -(quad_kin.links_size[4] + quad_kin.links_size[5] + quad_kin.links_size[6]) +  0.18 + 0.15*sym.sin(1*t))

	traj_des_RB = quad_kin.get_cart_trajectory_by_sym(-(quad_kin.links_size[1] + quad_kin.links_size[3]), (quad_kin.links_size[0] + quad_kin.links_size[2]), -(quad_kin.links_size[4] + quad_kin.links_size[5] + quad_kin.links_size[6]) +  0.18 + 0.15*sym.sin(1*t))

	def traj_track(t, leg_name, motors_names, traj_des, Kd, Kp):
		q_cur = np.array([[0],
						  [0],
						  [0]], dtype = np.float)
			
		q_dot_cur = np.array([[0],
						[0],
						[0]], dtype = np.float)

		for idx, motor in enumerate(motors_names):
			q_cur[idx] = cheetah_control_pos.joints_positions[motor]
			q_dot_cur[idx] = cheetah_control_pos.joints_velocities[motor]
		
		x_des_,x_dot_des_, y_des_, y_dot_des_, z_des_, z_dot_des_ = list(map(lambda x: x(t), traj_des))

		pos_des = [x_des_, y_des_, z_des_]
		vel_des = [x_dot_des_, y_dot_des_, z_dot_des_]
		if leg_name == "LF_leg":
			q_des = quad_kin.leg_ik(base = quad_kin.base_LF, pos = pos_des)

			q_dot_goal = - np.dot(np.linalg.pinv(quad_kin.LF_LEG_JAC_LAMBD(q_des[0], q_des[1], q_des[2])), np.array([[vel_des[0]],																												[vel_des[1]],
																										[vel_des[2]]]))
		elif leg_name == "LB_leg":
			q_des = quad_kin.leg_ik(base = quad_kin.base_LB, pos = pos_des, flag = 1)

			q_dot_goal = - np.dot(np.linalg.pinv(quad_kin.LB_LEG_JAC_LAMBD(q_des[0], q_des[1], q_des[2])), np.array([[vel_des[0]],																												[vel_des[1]],
																										[vel_des[2]]]))
		elif leg_name == "RB_leg":
			q_des = quad_kin.leg_ik(base = quad_kin.base_RB, pos = pos_des)

			q_dot_goal = - np.dot(np.linalg.pinv(quad_kin.RB_LEG_JAC_LAMBD(q_des[0], q_des[1], q_des[2])), np.array([[vel_des[0]],																												[vel_des[1]],
																										[vel_des[2]]]))
		elif leg_name == "RF_leg":
			q_des = quad_kin.leg_ik(base = quad_kin.base_RF, pos = pos_des, flag = 1)

			q_dot_goal = - np.dot(np.linalg.pinv(quad_kin.RF_LEG_JAC_LAMBD(q_des[0], q_des[1], q_des[2])), np.array([[vel_des[0]],																												[vel_des[1]],
																										[vel_des[2]]]))		
		else: print('wrong input to traj')
		

		# q_dot_goal = np.array([[0],
		# 			  [0],
		# 			  [0]], dtype = np.float)

		q_goal = - np.array([[q_des[0]],
							[q_des[1]],
							[q_des[2]]]) # "-" is because the z-axis is inverted for each motor

		U = np.dot(Kp,q_goal - q_cur) + np.dot(Kd,q_dot_goal - q_dot_cur)

		cheetah_control_pos.move_joint(motors_names[0], U[0])
		cheetah_control_pos.move_joint(motors_names[1], U[1])
		cheetah_control_pos.move_joint(motors_names[2], U[2])
	while not rospy.is_shutdown():
		try:
			if not cheetah_control_pos.joints_positions:
				continue
			t = time.time() - start_time

			traj_track(t, 'LF_leg', LF_leg, traj_des_LF, Kd, Kp)
			traj_track(t, 'LB_leg', LB_leg, traj_des_LB, Kd, Kp)
			traj_track(t, 'RB_leg', RB_leg, traj_des_RB, Kd, Kp)
			traj_track(t, 'RF_leg', RF_leg, traj_des_RF, Kd, Kp)
			cheetah_control_pos.rate.sleep()
		except rospy.ROSInterruptException:
			pass