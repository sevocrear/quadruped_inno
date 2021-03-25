import numpy as np
import spi
import time
# from can import CAN_Bus
from motors.tmotor import TMotorQDD
from SPIne import SPIne
# bus = CAN_Bus()
# // 0  1  2  3  4  5  6  7  8  9 10 11 12 13 14 15 16 17 18 19 20 21 22 23 24 25 26 27
# // 3 01 FF FF FF FF FF FF FF FF 02 FF FF FF FF FF FF FF FF 03 FF FF FF FF FF FF FF FF

# // 28 29 30 31 32 33 34 35 36 37 38 39 40 41 42 43 44 45 46 47 48 49 50 51 52 53 54 55
# // 3  01 FF FF FF FF FF FF FF FF 02 FF FF FF FF FF FF FF FF 03 FF FF FF FF FF FF FF FF


def main():

    motor_0_LF = TMotorQDD(motor_id = 0, CAN_ID = 2)
    motor_1_LF = TMotorQDD(motor_id = 1, CAN_ID = 2)
    motor_2_LF = TMotorQDD(motor_id = 2, CAN_ID = 2)

    motor_0_RF = TMotorQDD(motor_id = 0, CAN_ID = 0)
    motor_1_RF = TMotorQDD(motor_id = 1, CAN_ID = 0)
    motor_2_RF = TMotorQDD(motor_id = 2, CAN_ID = 0)

    motor_0_LB = TMotorQDD(motor_id = 0, CAN_ID = 3)
    motor_1_LB = TMotorQDD(motor_id = 1, CAN_ID = 3)
    motor_2_LB = TMotorQDD(motor_id = 2, CAN_ID = 3)

    motor_0_RB = TMotorQDD(motor_id = 0, CAN_ID = 2)
    motor_1_RB = TMotorQDD(motor_id = 1, CAN_ID = 2)
    motor_2_RB = TMotorQDD(motor_id = 2, CAN_ID = 2)

    # motor_3 = TMotorQDD(motor_id = 1, CAN_ID = 1)
    # motor_4 = TMotorQDD(motor_id = 0, CAN_ID = 1)

    motors = [
                # motor_0_LF, motor_1_LF, motor_2_LF, 
                # motor_0_RF, motor_1_RF, motor_2_RF, 
                # motor_0_LB, motor_1_LB, motor_2_LB, 
                motor_0_RB, motor_1_RB, motor_2_RB, 
                ]
    spine = SPIne(motors = motors)
    for motor in motors:
        motor.set_torque_limit(10)
    pos_des = 0
    kp, kd = 10, 0.5
    try:
        for motor in motors:
            motor.enable()
        # time.sleep(0.5)
        spine.transfer_and_receive()
        # time.sleep(0.5)

        # for motor in motors:
        #     motor.set_zero()
        # time.sleep(0.1)
        spine.transfer_and_receive()
        # time.sleep(0.1)

        print(f'You are going to turn on motors, press "Y" to continue...\n')
        user_input = input()
        start_time = time.time()
        if user_input == 'Y' or user_input == 'y':
            while 1:
                for motor in motors:
                    start = time.time()
                    end = time.time()
                    pos = motor.state['pos']
                    vel = motor.state['vel']
                    u = kp*(pos_des - pos) - kd*vel
                    motor.set_torque(u)

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

    except:
        for motor in motors:
            motor.disable()
        spine.transfer_and_receive()
    
    # Close file descriptors
    spi.closeSPI(spine.device_0)
    spi.closeSPI(spine.device_1)


if __name__ == "__main__":
    main()
