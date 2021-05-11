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
import tkinter as tk

def tk_reconfigure_xyz_rpy(shared_variables):
    window = tk.Tk()
    window.title('XYZ RPY CONTROL')
    window.geometry('1920x1080') 
    
    l = tk.Label(window, bg='white', fg='black', width=20, text='')
    l.pack()
    
    l1 = tk.Label(window, bg='white', fg='black', width=20, text='')
    l1.pack()

    l2 = tk.Label(window, bg='white', fg='black', width=20, text='')
    l2.pack()

    l3 = tk.Label(window, bg='white', fg='black', width=20, text='')
    l3.pack()

    l4 = tk.Label(window, bg='white', fg='black', width=20, text='')
    l4.pack()

    l5 = tk.Label(window, bg='white', fg='black', width=20, text='')
    l5.pack()

    l6 = tk.Label(window, bg='white', fg='black', width=20, text='')
    l6.pack()

    l7 = tk.Label(window, bg='yellow', fg='black', width=20, text='')
    l7.pack()

    l8 = tk.Label(window, bg='yellow', fg='black', width=20, text='')
    l8.pack()

    l9 = tk.Label(window, bg='yellow', fg='black', width=20, text='')
    l9.pack()

    l10 = tk.Label(window, bg='yellow', fg='black', width=20, text='')
    l10.pack()

    l11 = tk.Label(window, bg='yellow', fg='black', width=20, text='')
    l11.pack()

    def print_selection_Kp_abad(v):
        l.config(text='Kp abad =' + v)
        shared_variables[0] = float(v)
    
    def print_selection_Kp_thigh(v):
        l2.config(text='Kp thigh =' + v)
        shared_variables[1] = float(v)
    
    def print_selection_Kp_knee(v):
        l4.config(text='Kp knee =' + v)
        shared_variables[2] = float(v)

    def print_selection_Kd_abad(v):
        l1.config(text='Kd abad =' + v)
        shared_variables[3] = float(v)
    
    def print_selection_Kd_thigh(v):
        l3.config(text='Kd thigh =' + v)
        shared_variables[4] = float(v)
    
    def print_selection_Kd_knee(v):
        l5.config(text='Kd knee =' + v)
        shared_variables[5] = float(v)

    def print_selection_X(v):
        l6.config(text='X =' + v)
        shared_variables[6] = float(v)
    
    def print_selection_Y(v):
        l7.config(text='Y =' + v)
        shared_variables[7] = float(v)
    
    def print_selection_Z(v):
        l8.config(text='Z =' + v)
        shared_variables[8] = float(v)

    def print_selection_roll(v):
        l9.config(text='roll =' + v)
        shared_variables[9] = float(v)

    def print_selection_pitch(v):
        l10.config(text='pitch =' + v)
        shared_variables[10] = float(v)
    
    def print_selection_yaw(v):
        l11.config(text='yaw =' + v)
        shared_variables[11] = float(v)

    s = tk.Scale(window, label='Kp abad', from_= 1, to=500, orient=tk.HORIZONTAL, length=500, showvalue=0,tickinterval=100, resolution=0.01, command=print_selection_Kp_abad)
    s.pack()
    s1 = tk.Scale(window, label='Kd abad', from_=0, to=50, orient=tk.HORIZONTAL, length=500, showvalue=0,tickinterval=25, resolution=0.01, command=print_selection_Kd_abad)
    s1.pack()

    s2 = tk.Scale(window, label='Kp tjigh', from_= 1, to=500, orient=tk.HORIZONTAL, length=500, showvalue=0,tickinterval=100, resolution=0.01, command=print_selection_Kp_thigh)
    s2.pack()
    s3 = tk.Scale(window, label='Kd thigh', from_=0, to=50, orient=tk.HORIZONTAL, length=500, showvalue=0,tickinterval=25, resolution=0.01, command=print_selection_Kd_thigh)
    s3.pack()

    s4 = tk.Scale(window, label='Kp knee', from_= 1, to=500, orient=tk.HORIZONTAL, length=500, showvalue=0,tickinterval=100, resolution=0.01, command=print_selection_Kp_knee)
    s4.pack()
    s5 = tk.Scale(window, label='Kd knee', from_=0, to=50, orient=tk.HORIZONTAL, length=500, showvalue=0,tickinterval=25, resolution=0.01, command=print_selection_Kd_knee)
    s5.pack()

    s6 = tk.Scale(window, label='X', from_=0, to=0, orient=tk.HORIZONTAL, length=500, showvalue=0,tickinterval=25, resolution=0.01, command=print_selection_X)
    s6.pack()

    s7 = tk.Scale(window, label='Y', from_=0, to=0, orient=tk.HORIZONTAL, length=500, showvalue=0,tickinterval=25, resolution=0.01, command=print_selection_Y)
    s7.pack()

    var_Z = tk.DoubleVar()
    var_Z.set(0.425)
    s8 = tk.Scale(window, label='Z', from_=0.1, to=0.425, orient=tk.HORIZONTAL, length=300, showvalue=0,tickinterval=0.1, resolution=0.001, command=print_selection_Z, variable = var_Z)
    s8.pack()

    var_R = tk.DoubleVar()
    var_R.set(0.0)
    s9 = tk.Scale(window, label='R', from_=-0.15, to=0.15, orient=tk.HORIZONTAL, length=300, showvalue=0,tickinterval=0.1, resolution=0.001, command=print_selection_roll, variable = var_R)
    s9.pack()

    var_P = tk.DoubleVar()
    var_P.set(0.0)
    s10 = tk.Scale(window, label='P', from_=-0.15, to=0.15, orient=tk.HORIZONTAL, length=300, showvalue=0,tickinterval=0.1, resolution=0.001, command=print_selection_pitch, variable = var_P)
    s10.pack()

    var_Y = tk.DoubleVar()
    var_Y.set(0.0)
    s11 = tk.Scale(window, label='Y', from_=-0.15, to=0.15, orient=tk.HORIZONTAL, length=300, showvalue=0,tickinterval=0.1, resolution=0.001, command=print_selection_yaw, variable = var_Y)
    s11.pack()
    window.mainloop()

def control_main(shared_variables):
    use_ros = False

    if use_ros == False:
        from SPIne import SPIne
        from motors.tmotor import TMotorQDD
    cheetah_control_pos = cheetah_control(
        type_of_control='torque', use_ros=use_ros, rate_value= 1000)

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
                    pass
            # Proportional-Derivative coefficients
            Kp = np.array([[shared_variables[0], 0, 0],
                        [0, shared_variables[1], 0],
                        [0, 0, shared_variables[2]]])
            Kd = np.array([[shared_variables[3], 0, 0],
                        [0, shared_variables[4], 0],
                        [0, 0, shared_variables[5]]])
            XYZ = [shared_variables[6],shared_variables[7],shared_variables[8]]
            RPY = [shared_variables[9],shared_variables[10],shared_variables[11]]
            st = time.time()
            t = time.time() - start_time
            Kp, Kd = cheetah_control_pos.update_PD(Kp, Kd) # for ros
            U, flag, q_cur, q_dot_cur, q_des, q_dot_des = cheetah_control_pos.go_to_desired_RPY_of_base(
                quad_kin, LF_foot_pos, RF_foot_pos, LB_foot_pos, RB_foot_pos, Kd, Kp, tmotors=motors, xyz=XYZ, rpy = RPY, use_input_traj = True)
            cheetah_control_pos.go_to_zero_all(Kp, Kd)
            if not use_ros:
                cheetah_control_pos.motor_set_pos(motors['LF_leg'], q_des['LF_leg'], [Kp[0,0],Kp[1,1],Kp[2,2]], [Kd[0,0],Kd[1,1],Kd[2,2]])
                cheetah_control_pos.motor_set_pos(motors['RF_leg'], [0,0,0], [Kp[0,0],Kp[1,1],Kp[2,2]], [Kd[0,0],Kd[1,1],Kd[2,2]])

                cheetah_control_pos.motor_set_pos(motors['LB_leg'], [0,0,0], [Kp[0,0],Kp[1,1],Kp[2,2]], [Kd[0,0],Kd[1,1],Kd[2,2]])
                cheetah_control_pos.motor_set_pos(motors['RB_leg'], [0,0,0], [Kp[0,0],Kp[1,1],Kp[2,2]], [Kd[0,0],Kd[1,1],Kd[2,2]])
                # cheetah_control_pos.motor_set_torque(motors['LF_leg'], [0,0,0])
                # cheetah_control_pos.motor_set_torque(motors['RF_leg'], [0,0,0])
                # cheetah_control_pos.motor_set_torque(motors['LB_leg'], [0,0,0])
                # cheetah_control_pos.motor_set_torque(motors['RB_leg'], [0,0,0])
                spine.transfer_and_receive()
            # data_saver.update(q_cur, q_dot_cur, q_des, q_dot_des, t, motors)
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
    # if t % 2 < 10e-3:
    #     print(Kp, Kd)

if __name__ == '__main__':
    manager = Manager()
    shared_variables = manager.Array('f', [1,1,1, 0,0,0, 0,0,0.425, 0,0,0 ]) # Kp,Kd, xyz, RPY
    process_control = Process(target = control_main, args = [shared_variables])
    process_GUI = Process(target = tk_reconfigure_xyz_rpy, args = [shared_variables])

    process_control.start()
    process_GUI.start()
    process_control.join()
    process_GUI.join()


