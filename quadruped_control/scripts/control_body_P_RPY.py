#!/usr/bin/env python3
# license removed for brevity
import pandas as pd
import rospy
import os
import sys
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
sys.path.append("../..")
sys.path.append("../../SPI-Py/")
from cheetah_control import cheetah_control
import numpy as np
import time
import sympy as sym
from quadruped_kinematics import quadruped_kinematics
from log_data import save_data

if __name__ == '__main__':
    use_ros = True

    if use_ros == False:
        from SPIne import SPIne
        from motors.tmotor import TMotorQDD
    cheetah_control_pos = cheetah_control(
        type_of_control='torque', use_ros=use_ros)

    # Proportional-Derivative coefficients
    Kp = np.array([[1, 0, 0],
                   [0, 1, 0],
                   [0, 0, 1]])*20
    Kd = np.array([[1, 0, 0],
                   [0, 1, 0],
                   [0, 0, 1]])

    links_sizes_mm = [162.75, 65, 72.25, 82.25,
                      208, 159, 58.5]  # leg links sizes
    quad_kin = quadruped_kinematics(
        [link_size / 1000 for link_size in links_sizes_mm])

    # initialize feet poses wrt the world
    LF_foot_pos = [quad_kin.links_size[1] + quad_kin.links_size[3], -
                   (quad_kin.links_size[0] + quad_kin.links_size[3]), 0]
    RF_foot_pos = [-(quad_kin.links_size[1] + quad_kin.links_size[3]), -
                   (quad_kin.links_size[0] + quad_kin.links_size[3]), 0]
    LB_foot_pos = [(quad_kin.links_size[1] + quad_kin.links_size[3]),
                   (quad_kin.links_size[0] + quad_kin.links_size[3]), 0]
    RB_foot_pos = [-(quad_kin.links_size[1] + quad_kin.links_size[3]),
                   (quad_kin.links_size[0] + quad_kin.links_size[3]), 0]

    # initialize motors
    motors = {'LF_leg': [], 'RF_leg': [], 'LB_leg': [], 'RB_leg': []}
    if not use_ros:
        motor_0_LF = TMotorQDD(motor_id=0, CAN_ID=1)
        motor_1_LF = TMotorQDD(motor_id=1, CAN_ID=1)
        motor_2_LF = TMotorQDD(motor_id=2, CAN_ID=1)

        motor_0_RF = TMotorQDD(motor_id=0, CAN_ID=0)
        motor_1_RF = TMotorQDD(motor_id=1, CAN_ID=0)
        motor_2_RF = TMotorQDD(motor_id=2, CAN_ID=0)

        motor_0_LB = TMotorQDD(motor_id=0, CAN_ID=3)
        motor_1_LB = TMotorQDD(motor_id=1, CAN_ID=3)
        motor_2_LB = TMotorQDD(motor_id=2, CAN_ID=3)

        motor_0_RB = TMotorQDD(motor_id=0, CAN_ID=2)
        motor_1_RB = TMotorQDD(motor_id=1, CAN_ID=2)
        motor_2_RB = TMotorQDD(motor_id=2, CAN_ID=2)

        motors = {
            'LF_leg': [motor_0_LF, motor_1_LF, motor_2_LF],
            'RF_leg': [motor_0_RF, motor_1_RF, motor_2_RF],
            'LB_leg': [motor_0_LB, motor_1_LB, motor_2_LB],
            'RB_leg': [motor_0_RB, motor_1_RB, motor_2_RB]
        }

        spine = SPIne(motors=motors)
        # Set torque limits for hip, thigh and knee
        for motor in motors:
            motors[motor][0].set_torque_limit(10)
            motors[motor][1].set_torque_limit(10)
            motors[motor][2].set_torque_limit(10)
        try:
            # Set zero poses for hip, thigh and knee
            for motor in motors:
                motors[motor][0].set_zero()
                motors[motor][1].set_zero()
                motors[motor][2].set_zero()
            spine.transfer_and_receive()
            # Launch actuators
            for motor in motors:
                motors[motor][0].enable()
                motors[motor][1].enable()
                motors[motor][2].enable()
            spine.transfer_and_receive()
        except:
            print('Motors didn\'t initialize. Try again')
            exit()

    # object for storing data
    data_saver = save_data()

    start_time = time.time()
    while not rospy.is_shutdown():
        try:
            if use_ros:
                if not cheetah_control_pos.joints_positions:
                    continue

            t = time.time() - start_time
            Kp, Kd = cheetah_control_pos.update_PD(Kp, Kd)
            U, flag, q_cur, q_dot_cur, q_des, q_dot_des = cheetah_control_pos.go_to_desired_RPY_of_base(
                quad_kin, LF_foot_pos, RF_foot_pos, LB_foot_pos, RB_foot_pos, Kd, Kp, tmotors=motors, xyz=[0, 0, 0.4 + 0.025*np.cos(3*t)], use_input_traj = True)
            cheetah_control_pos.go_to_zero_all(Kp, Kd)
            if not use_ros:
                for motor in motors:
                    if motor in ['RF_leg']:
                        # motors[motor][0].set_torque(0)
                        # motors[motor][1].set_torque(0)
                        # motors[motor][2].set_torque(0)
                        motors[motor][0].set_torque(U[motor][0])
                        motors[motor][1].set_torque(U[motor][1])
                        motors[motor][2].set_torque(U[motor][2])
                    else:
                        motors[motor][0].set_torque(0)
                        motors[motor][1].set_torque(0)
                        motors[motor][2].set_torque(0)
                spine.transfer_and_receive()

            data_saver.update(q_cur, q_dot_cur, q_des, q_dot_des, t, motors)
            
            if use_ros:
                cheetah_control_pos.rate.sleep()
        except rospy.ROSInterruptException:
            pass
        except KeyboardInterrupt:
            # save dataframe to csv file
            data_saver.save_data_to_csv()
            if not use_ros:
                for motor in motors:
                    motors[motor][0].disable()
                    motors[motor][1].disable()
                    motors[motor][2].disable()
                spine.transfer_and_receive()
                break
            else:
                break
        # except:
        #     data_saver.save_data_to_csv()
        #     if not use_ros:
        #         for motor in motors:
        #             motors[motor][0].disable()
        #             motors[motor][1].disable()
        #             motors[motor][2].disable()
        #         spine.transfer_and_receive()
        #     else:
        #         break
    data_saver.save_data_to_csv()
    if not use_ros:
        for motor in motors:
            motors[motor][0].disable()
            motors[motor][1].disable()
            motors[motor][2].disable()
        spine.transfer_and_receive()
