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
from multiprocessing import Process, Manager
from tk_reconfigure import tk_reconfigure_xyz_rpy

def control_main(shared_variables, use_ros, use_input_traj, shared_t = None, shared_q_cur = None, shared_q_des = None, shared_q_dot_cur = None, shared_q_dot_des = None):

    if use_ros == False:
        from SPIne import SPIne
        from motors.tmotor import TMotorQDD
        os.system("sudo chmod a+rw /dev/spidev1.0")
        os.system("sudo chmod a+rw /dev/spidev1.1")
        
    cheetah_control_pos = cheetah_control(
        type_of_control='position', use_ros=use_ros, rate_value= 1000)

    links_sizes_mm = [162.75, 65, 72.25, 82.25,
                      208, 159, 58.5]  # leg links sizes
    quad_kin = quadruped_kinematics(
        [link_size / 1000 for link_size in links_sizes_mm])

    # initialize feet poses wrt the world
    LF_foot_pos = [quad_kin.links_size[1] + quad_kin.links_size[3], -
                   (quad_kin.links_size[0] + quad_kin.links_size[2]), 0]

    RF_foot_pos = [-(quad_kin.links_size[1] + quad_kin.links_size[3]), -
                   (quad_kin.links_size[0] + quad_kin.links_size[2]), 0]

    LB_foot_pos = [(quad_kin.links_size[1] + quad_kin.links_size[3]),
                   (quad_kin.links_size[0] + quad_kin.links_size[2]), 0]

    RB_foot_pos = [-(quad_kin.links_size[1] + quad_kin.links_size[3]),
                   (quad_kin.links_size[0] + quad_kin.links_size[2]), 0]

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

    start_time = time.time()

    Kp, Kd = np.eye(3), np.eye(3)*0.1

    while not rospy.is_shutdown():
        try:
            
            if use_ros:
                if not cheetah_control_pos.joints_positions:
                    continue
            # Proportional-Derivative coefficients
            if not use_ros:
                Kp = np.array([[shared_variables[0], 0, 0],
                            [0, shared_variables[1], 0],
                            [0, 0, shared_variables[2]]])
                Kd = np.array([[shared_variables[3], 0, 0],
                            [0, shared_variables[4], 0],
                            [0, 0, shared_variables[5]]])
                XYZ = [shared_variables[6],shared_variables[7],shared_variables[8]]
                RPY = [shared_variables[9],shared_variables[10],shared_variables[11]]
            else:
                Kp, Kd = cheetah_control_pos.update_PD(Kp, Kd) # for ros
                XYZ = [0,0,0.4]
                RPY = [0,0,0]
            t = time.time() - start_time
            U, flag, q_cur, q_dot_cur, q_des, q_dot_des = cheetah_control_pos.go_to_desired_RPY_of_base(
                quad_kin, LF_foot_pos, RF_foot_pos, LB_foot_pos, RB_foot_pos, Kd, Kp, tmotors=motors, xyz=XYZ, rpy = RPY, use_input_traj = use_input_traj)
            cheetah_control_pos.go_to_zero_all(Kp, Kd)
            if not use_ros:
                cheetah_control_pos.motor_set_pos(motors['LF_leg'], q_des['LF_leg'], [Kp[0,0],Kp[1,1],Kp[2,2]], [Kd[0,0],Kd[1,1],Kd[2,2]])
                cheetah_control_pos.motor_set_pos(motors['RF_leg'], q_des['RF_leg'], [Kp[0,0],Kp[1,1],Kp[2,2]], [Kd[0,0],Kd[1,1],Kd[2,2]])

                cheetah_control_pos.motor_set_pos(motors['LB_leg'], q_des['LB_leg'], [Kp[0,0],Kp[1,1],Kp[2,2]], [Kd[0,0],Kd[1,1],Kd[2,2]])
                cheetah_control_pos.motor_set_pos(motors['RB_leg'], q_des['RB_leg'], [Kp[0,0],Kp[1,1],Kp[2,2]], [Kd[0,0],Kd[1,1],Kd[2,2]])
                # cheetah_control_pos.motor_set_torque(motors['LF_leg'], [0,0,0])
                # cheetah_control_pos.motor_set_torque(motors['RF_leg'], [0,0,0])
                # cheetah_control_pos.motor_set_torque(motors['LB_leg'], [0,0,0])
                # cheetah_control_pos.motor_set_torque(motors['RB_leg'], [0,0,0])
                spine.transfer_and_receive()

            # UNCOMMENT IF YOU WANT TO RECORD DATA
            # shared_t.value = t
            # shared_q_cur['LF_leg'] = q_cur['LF_leg']
            # shared_q_cur['RF_leg'] = q_cur['RF_leg']
            # shared_q_cur['LB_leg'] = q_cur['LB_leg']
            # shared_q_cur['RB_leg'] = q_cur['RB_leg']

            # shared_q_des['LF_leg'] = q_des['LF_leg']
            # shared_q_des['RF_leg'] = q_des['RF_leg']
            # shared_q_des['LB_leg'] = q_des['LB_leg']
            # shared_q_des['RB_leg'] = q_des['RB_leg']

            # shared_q_dot_cur['LF_leg'] = q_dot_cur['LF_leg']
            # shared_q_dot_cur['RF_leg'] = q_dot_cur['RF_leg']
            # shared_q_dot_cur['LB_leg'] = q_dot_cur['LB_leg']
            # shared_q_dot_cur['RB_leg'] = q_dot_cur['RB_leg']

            # shared_q_dot_des['LF_leg'] = q_dot_des['LF_leg']
            # shared_q_dot_des['RF_leg'] = q_dot_des['RF_leg']
            # shared_q_dot_des['LB_leg'] = q_dot_des['LB_leg']
            # shared_q_dot_des['RB_leg'] = q_dot_des['RB_leg']
            if use_ros:
                cheetah_control_pos.rate.sleep()
        except rospy.ROSInterruptException:
            pass
        except KeyboardInterrupt:
            # save dataframe to csv file
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
        #     if not use_ros:
        #         for motor in motors:
        #             motors[motor][0].disable()
        #             motors[motor][1].disable()
        #             motors[motor][2].disable()
        #         spine.transfer_and_receive()
        #     else:
        #         break
    if not use_ros:
        for motor in motors:
            motors[motor][0].disable()
            motors[motor][1].disable()
            motors[motor][2].disable()
        spine.transfer_and_receive()

def saving_data(shared_t, shared_q_cur, shared_q_des, shared_q_dot_cur, shared_q_dot_des):
    # # object for storing data
    data_saver = save_data()
    while True:
        try:
            data_saver.update(shared_q_cur, shared_q_dot_cur, shared_q_des, shared_q_dot_des, shared_t, ['LF_leg', 'RF_leg', 'RB_leg', 'LB_leg'])
        except KeyboardInterrupt:
            # save dataframe to csv file
            data_saver.save_data_to_csv()
            break
        except:
            data_saver.save_data_to_csv()
            break

if __name__ == '__main__':


    use_ros = False
    use_input_traj = True # control from tk GUI

    manager = Manager()
    shared_variables = manager.Array('f', [1,1,1, 0,0,0, 0,0,0.425, 0,0,0 ]) # Kp,Kd, xyz, RPY
    # UNCOMMENT BELOW VARIABLES IF YOU WANT TO RECORD
    # shared_q_cur = manager.dict()
    # shared_q_des = manager.dict()
    # shared_q_dot_cur = manager.dict()
    # shared_q_dot_des = manager.dict()
    # shared_t = manager.Value('d', 0)
    process_control = Process(target = control_main, args = [shared_variables, use_ros,use_input_traj,
                                                            #  shared_t, shared_q_cur, shared_q_des, shared_q_dot_cur, shared_q_dot_des #uncomment if you want to record data
                                                             ])
                                                            
    process_GUI = Process(target = tk_reconfigure_xyz_rpy, args = [shared_variables])
    # process_save_data = Process(target = saving_data, args = [shared_t, shared_q_cur, shared_q_des, shared_q_dot_cur, shared_q_dot_des])
    process_control.start()
    if not use_ros:
        process_GUI.start()
    # process_save_data.start()
    process_control.join()
    if not use_ros:
        process_GUI.join()
    # process_save_data.join()


