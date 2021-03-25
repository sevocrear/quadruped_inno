import numpy as np
import spi
import time
import sys
import os.path
import sympy as sym
sys.path.append(
    os.path.abspath(os.path.join(os.path.dirname(__file__), os.path.pardir)))

# from can import CAN_Bus
from motors.tmotor import TMotorQDD
from SPIne import SPIne
from quadruped_kinematics import quadruped_kinematics
# bus = CAN_Bus()
# // 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27
# // 3 01 FF FF FF FF FF FF FF FF 02 FF FF FF FF FF FF FF FF 03 FF FF FF FF FF FF FF FF

# // 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55
# // 3  01 FF FF FF FF FF FF FF FF 02 FF FF FF FF FF FF FF FF 03 FF FF FF FF FF FF FF FF


def main():
    
    links_sizes_mm = [162.75, 65, 72.25, 82.25, 208, 159, 58.5]
    quad_kin = quadruped_kinematics([link_size / 1000 for link_size in links_sizes_mm])

    motor_0_LF = TMotorQDD(motor_id = 0, CAN_ID = 1)
    motor_1_LF = TMotorQDD(motor_id = 1, CAN_ID = 1)
    motor_2_LF = TMotorQDD(motor_id = 2, CAN_ID = 1)

    motor_0_RF = TMotorQDD(motor_id = 0, CAN_ID = 0)
    motor_1_RF = TMotorQDD(motor_id = 1, CAN_ID = 0)
    motor_2_RF = TMotorQDD(motor_id = 2, CAN_ID = 0)

    motor_0_LB = TMotorQDD(motor_id = 0, CAN_ID = 3)
    motor_1_LB = TMotorQDD(motor_id = 1, CAN_ID = 3)
    motor_2_LB = TMotorQDD(motor_id = 2, CAN_ID = 3)

    motor_0_RB = TMotorQDD(motor_id = 0, CAN_ID = 2)
    motor_1_RB = TMotorQDD(motor_id = 1, CAN_ID = 2)
    motor_2_RB = TMotorQDD(motor_id = 2, CAN_ID = 2)

    motors = [
                motor_0_LF, motor_1_LF, motor_2_LF, 
                # motor_0_RF, motor_1_RF, motor_2_RF, 
                # motor_0_LB, motor_1_LB, motor_2_LB, 
                # motor_0_RB, motor_1_RB, motor_2_RB, 
                ]
    spine = SPIne(motors = motors)
    for motor in motors:
        motor.set_torque_limit(10)
        
    kp, kd = 50, 2.5
    Kp = np.eye(3,3)*kp
    Kd = np.eye(3,3)*kd

    try:
        for motor in motors:
            motor.enable()
        # time.sleep(0.5)
        spine.transfer_and_receive()
        # time.sleep(0.5)

        for motor in motors:
            motor.set_zero()
        # time.sleep(0.1)
        spine.transfer_and_receive()
        # time.sleep(0.1)

        print(f'You are going to turn on motors, press "Y" to continue...\n')
        user_input = input()
        start_time = time.time()

        w = 3
        t = sym.symbols("t")
        poses_0 = quad_kin.lf_leg_fk(base = quad_kin.base_LF, angles = [motors[0].state['pos'], motors[1].state['pos'], motors[2].state['pos']])
        x_0, y_0, z_0 = poses_0[3]
        print(x_0, y_0, z_0)
        x_des,x_dot_des, y_des, y_dot_des, z_des, z_dot_des = quad_kin.get_cart_trajectory_by_sym(x_0, y_0, z_0 + z_0/2)


        if user_input == 'Y' or user_input == 'y':
            while 1:
                t = time.time() - start_time
                q_pos = [motors[0].state['pos'], motors[1].state['pos'], motors[2].state['pos']]
                q_vel = [motors[0].state['vel'], motors[1].state['vel'], motors[2].state['vel']]
                # for motor in motors:

                x_des_,x_dot_des_, y_des_, y_dot_des_, z_des_, z_dot_des_ = x_des(t),x_dot_des(t), y_des(t), y_dot_des(t), z_des(t), z_dot_des(t)

                print(x_des_, y_des_, z_des_)
                th_1_ik, th_2_ik, th_3_ik = quad_kin.leg_ik(base = quad_kin.base_LF, pos = [x_des_, y_des_, z_des_])
                # print(q_pos)
                _, _, _, pos = quad_kin.lf_leg_fk(base = quad_kin.base_LF, angles = q_pos)
                
                vel = quad_kin.get_cartesian_velocities(jac = quad_kin.LF_leg_Jac_sym(q_pos[0], q_pos[1], q_pos[2]), angle_velocities = q_vel)

                u = np.linalg.multi_dot([np.transpose(quad_kin.LF_leg_Jac_sym(q_pos[0], q_pos[1], q_pos[2])),Kp,np.array([x_des_-pos[0], y_des_ - pos[1], z_des_ - pos[2]]).reshape(3,1)]) + np.linalg.multi_dot([np.transpose(quad_kin.LF_leg_Jac_sym(q_pos[0], q_pos[1], q_pos[2])),Kd,np.array([x_dot_des_-vel[0], y_dot_des_-vel[1], z_dot_des_-vel[2]]).reshape(3,1)])
                
                
                i = 0
                for motor in motors:
                    motor.set_torque(u[i,0])
                    i+=1

                spine.transfer_and_receive()

                # if time.time() - start_time > 20:
                #     spine.data_out_0[56] = 0xAE
                # time.sleep(40e-3)
                # for motor in motors:
                #     print(motor.state["pos"], end = '\t')
                # print("\n")

                # time.sleep(10e-3)

            #     # print(end - start)
                # print(motor_1.state)
        for motor in motors:
            motor.disable()
        spine.transfer_and_receive()

    except KeyboardInterrupt:
        for motor in motors:
            motor.disable()
        spine.transfer_and_receive()

    except Exception as e:
        print(e)
        for motor in motors:
            motor.disable()
        spine.transfer_and_receive()
    
    # Close file descriptors
    spi.closeSPI(spine.device_0)
    spi.closeSPI(spine.device_1)


if __name__ == "__main__":
    main()
