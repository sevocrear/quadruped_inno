#!/usr/bin/env python3
# license removed for brevity
import sympy as sym
import time
import numpy as np
from cheetah_control import cheetah_control
import rospy
import os
import sys
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
sys.path.append("../..")
from quadruped_kinematics import quadruped_kinematics


if __name__ == '__main__':

    cheetah_control_pos = cheetah_control(type_of_control='torque')

    # q_goal_LB = np.array([[0],
    # 					  [0],
    # 					  [0]], dtype = np.float)

    Kp = np.array([[100, 0, 0],
                   [0, 100, 0],
                   [0, 0, 100]])
    Kd = np.array([[0.5, 0, 0],
                   [0, 0.5, 0],
                   [0, 0, 0.5]])

    start_time = time.time()

    links_sizes_mm = [162.75, 65, 72.25, 82.25, 208, 159, 58.5]
    quad_kin = quadruped_kinematics(
        [link_size / 1000 for link_size in links_sizes_mm])

    LF_foot_pos = [quad_kin.links_size[1] + quad_kin.links_size[3], -
                   (quad_kin.links_size[0] + quad_kin.links_size[3]), 0]
    RF_foot_pos = [-(quad_kin.links_size[1] + quad_kin.links_size[3]), -
                   (quad_kin.links_size[0] + quad_kin.links_size[3]), 0]
    LB_foot_pos = [(quad_kin.links_size[1] + quad_kin.links_size[3]),
                   (quad_kin.links_size[0] + quad_kin.links_size[3]), 0]
    RB_foot_pos = [-(quad_kin.links_size[1] + quad_kin.links_size[3]),
                   (quad_kin.links_size[0] + quad_kin.links_size[3]), 0]
    # t = sym.symbols("t")

    # traj_des_LF = quad_kin.get_cart_trajectory_by_sym(quad_kin.links_size[1] + quad_kin.links_size[3], -(quad_kin.links_size[0] + quad_kin.links_size[2]), -(
    #     quad_kin.links_size[4] + quad_kin.links_size[5] + quad_kin.links_size[6]) + 0.18 + 0.15*sym.sin(1*t))

    # traj_des_LB = quad_kin.get_cart_trajectory_by_sym(quad_kin.links_size[1] + quad_kin.links_size[3], (quad_kin.links_size[0] + quad_kin.links_size[2]), -(
    #     quad_kin.links_size[4] + quad_kin.links_size[5] + quad_kin.links_size[6]) + 0.18 + 0.15*sym.sin(1*t))

    # traj_des_RF = quad_kin.get_cart_trajectory_by_sym(-(quad_kin.links_size[1] + quad_kin.links_size[3]), -(
    #     quad_kin.links_size[0] + quad_kin.links_size[2]), -(quad_kin.links_size[4] + quad_kin.links_size[5] + quad_kin.links_size[6]) + 0.18 + 0.15*sym.sin(1*t))

    # traj_des_RB = quad_kin.get_cart_trajectory_by_sym(-(quad_kin.links_size[1] + quad_kin.links_size[3]), (quad_kin.links_size[0] + quad_kin.links_size[2]), -(
    #     quad_kin.links_size[4] + quad_kin.links_size[5] + quad_kin.links_size[6]) + 0.18 + 0.15*sym.sin(1*t))

    while not rospy.is_shutdown():
        try:
            if not cheetah_control_pos.joints_positions:
                continue
            t = time.time() - start_time
            Kp, Kd = cheetah_control_pos.update_PD(Kp, Kd)

            # cheetah_control_pos.leg_traj_track(
            #     t, 'LF_leg', cheetah_control_pos.LF_leg, traj_des_LF, Kd, Kp, quad_kin)
            # cheetah_control_pos.leg_traj_track(
            #     t, 'LB_leg', cheetah_control_pos.LB_leg, traj_des_LB, Kd, Kp, quad_kin)
            # cheetah_control_pos.leg_traj_track(
            #     t, 'RB_leg', cheetah_control_pos.RB_leg, traj_des_RB, Kd, Kp, quad_kin)
            # cheetah_control_pos.leg_traj_track(
            #     t, 'RF_leg', cheetah_control_pos.RF_leg, traj_des_RF, Kd, Kp, quad_kin)
            U, flag = cheetah_control_pos.go_to_desired_RPY_of_base(
                quad_kin, LF_foot_pos, RF_foot_pos, LB_foot_pos, RB_foot_pos, Kd, Kp)
            print(U)
            cheetah_control_pos.go_to_zero_all(Kp, Kd)
            cheetah_control_pos.rate.sleep()

        except rospy.ROSInterruptException:
            pass
