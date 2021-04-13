#!/usr/bin/env python3
# license removed for brevity
import os
import sys
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
sys.path.append("../..")
sys.path.append("../../SPI-Py/")

from quadruped_kinematics import quadruped_kinematics
# from SPIne import SPIne
# from motors.tmotor import TMotorQDD
import sympy as sym
import time
import numpy as np
from cheetah_control import cheetah_control
import rospy

if __name__ == '__main__':

    cheetah_control_pos = cheetah_control(
        type_of_control='torque', use_ros= True)

    # q_goal_LB = np.array([[0],
    # 					  [0],
    # 					  [0]], dtype = np.float)

    Kp = np.array([[10, 0, 0],
                   [0, 10, 0],
                   [0, 0, 5]])
    Kd = np.array([[1, 0, 0],
                   [0, 1, 0],
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

    # motor_0_LF = TMotorQDD(motor_id=0, CAN_ID=1)
    # motor_1_LF = TMotorQDD(motor_id=1, CAN_ID=1)
    # motor_2_LF = TMotorQDD(motor_id=2, CAN_ID=1)

    # motor_0_RF = TMotorQDD(motor_id=0, CAN_ID=0)
    # motor_1_RF = TMotorQDD(motor_id=1, CAN_ID=0)
    # motor_2_RF = TMotorQDD(motor_id=2, CAN_ID=0)

    # motor_0_LB = TMotorQDD(motor_id=0, CAN_ID=3)
    # motor_1_LB = TMotorQDD(motor_id=1, CAN_ID=3)
    # motor_2_LB = TMotorQDD(motor_id=2, CAN_ID=3)

    # motor_0_RB = TMotorQDD(motor_id=0, CAN_ID=2)
    # motor_1_RB = TMotorQDD(motor_id=1, CAN_ID=2)
    # motor_2_RB = TMotorQDD(motor_id=2, CAN_ID=2)

    # motors = {
    #     'LF_leg': [motor_0_LF, motor_1_LF, motor_2_LF],
    #     'RF_leg': [motor_0_RF, motor_1_RF, motor_2_RF],
    #     'LB_leg': [motor_0_LB, motor_1_LB, motor_2_LB],
    #     'RB_leg': [motor_0_RB, motor_1_RB, motor_2_RB]
    # }
    # spine = SPIne(motors=motors)
    # for motor in motors:
    #     motors[motor][0].set_torque_limit(1)
    #     motors[motor][1].set_torque_limit(1)
    #     motors[motor][2].set_torque_limit(1)
    # try:
    #     for motor in motors:
    #         motors[motor][0].enable()
    #         motors[motor][1].enable()
    #         motors[motor][2].enable()
    #     # time.sleep(0.5)
    #     spine.transfer_and_receive()
    #     # time.sleep(0.5)

    #     for motor in motors:
    #         motors[motor][0].set_zero()
    #         motors[motor][1].set_zero()
    #         motors[motor][2].set_zero()
    #     # time.sleep(0.1)
    #     spine.transfer_and_receive()
    #     # time.sleep(0.1)
    # try:
    while not rospy.is_shutdown():
        try:
            if not cheetah_control_pos.joints_positions:
                continue
            t = time.time() - start_time
            Kp, Kd = cheetah_control_pos.update_PD(Kp, Kd)

            U, flag = cheetah_control_pos.go_to_desired_RPY_of_base(
                quad_kin, LF_foot_pos, RF_foot_pos, LB_foot_pos, RB_foot_pos, Kd, Kp, tmotors= {})
            print(U)
            cheetah_control_pos.go_to_zero_all(Kp, Kd)
            cheetah_control_pos.rate.sleep()
            # for motor in motors:
            #     if motor == 'RB_leg':
            #         motors[motor][0].set_torque(U[motor][0])
            #         motors[motor][1].set_torque(U[motor][1])
            #         motors[motor][2].set_torque(U[motor][2])
            #     else:
            #         motors[motor][0].set_torque(0)
            #         motors[motor][1].set_torque(0)
            #         motors[motor][2].set_torque(0)
            # spine.transfer_and_receive()
        except rospy.ROSInterruptException:
            pass
    #         for motor in motors:
    #             motors[motor][0].disable()
    #             motors[motor][1].disable()
    #             motors[motor][2].disable()
    #         spine.transfer_and_receive()

    # except KeyboardInterrupt:
    #     for motor in motors:
    #         motors[motor][0].disable()
    #         motors[motor][1].disable()
    #         motors[motor][2].disable()
    #     spine.transfer_and_receive()

    # except Exception as e:
    #     print(e)

