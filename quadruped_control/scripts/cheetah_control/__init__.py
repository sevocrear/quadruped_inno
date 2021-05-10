#!/usr/bin/env python3
# license removed for brevity
import time
import copy
import numpy as np


class cheetah_control():
    def __init__(self, type_of_control='position', time_pause_before_control=0, rate_value=1000, use_ros = True):
        self.use_ros = use_ros
        if self.use_ros:
            from std_msgs.msg import Float64
            from sensor_msgs.msg import JointState
            import rospy
            from dynamic_reconfigure.msg import Config  # dynamics reconfigure
            rospy.init_node('cheetah_control', anonymous=True)
            from gazebo_msgs.msg import LinkStates
        self.type_of_control = type_of_control
        self.time_pause_before_control = time_pause_before_control
        time.sleep(self.time_pause_before_control)

        # Declare the joint names of each leg
        self.LB_leg = ['left_back_motor_leg','left_back_leg_hip', 'left_back_hip_calf']
        self.RB_leg = ['right_back_motor_leg','right_back_leg_hip', 'right_back_hip_calf']
        self.LF_leg = ['left_forward_motor_leg','left_forward_leg_hip', 'left_forward_hip_calf']
        self.RF_leg = ['right_forward_motor_leg','right_forward_leg_hip', 'right_forward_hip_calf']
        
        self.joints = {'FL_hip': self.LF_leg[0],
                       'FL_thigh': self.LF_leg[1],
                       'FL_calf': self.LF_leg[2],

                       'FR_hip': self.RF_leg[0],
                       'FR_thigh': self.RF_leg[1],
                       'FR_calf': self.RF_leg[2],

                       'RL_hip': self.RB_leg[0],
                       'RL_thigh': self.RB_leg[1],
                       'RL_calf': self.RB_leg[2],

                       'RR_hip':self.RB_leg[0],
                       'RR_thigh': self.RB_leg[1],
                       'RR_calf': self.RB_leg[2]
                       }

        if self.use_ros:
            if type_of_control == 'position':
                self.controllers = list(
                    map(lambda x: x + '_position_controller', self.joints))
            elif type_of_control == 'torque':
                self.controllers = list(
                    map(lambda x: x + '_Effort_controller', self.joints))
            self.pub = {}
            i = 0
            for joint in self.joints:
                self.joints[joint].append(self.controllers[i])
                self.pub[joint] = rospy.Publisher(self.joint_name(
                    self.joints[joint][1]), Float64, queue_size=10)
                i += 1

            self.cheetah_joints_sub = rospy.Subscriber(
                "/quadruped/joint_states", JointState, self.joints_pos_callback)
            self.cheetah_links_sub = rospy.Subscriber(
                "/gazebo/link_states", LinkStates, self.body_pos_callback)

            self.rate = rospy.Rate(rate_value)
            self.joints_positions = {}
            self.joints_velocities = {}

            self.body_position = {}
            self.body_orientation = {}
            self.body_twist_angular = {}
            self.body_twist_linear = {}

            self.reconfigure_sub = rospy.Subscriber(
                '/dynamic_reconfigure/parameter_updates', Config, self.reconfigure_cb)  # dynamics reconfigure subscriber
            self.config = Config()  # dynamics reconfigure

    def move_joint(self, joint, pos):
        if self.use_ros:
            joint = list(self.joints_to_corresp.keys())[
                list(self.joints_to_corresp.values()).index([joint])]
            self.pub[joint].publish(pos)

    def joint_name(self, joint):
        joint_name = '/quadruped/'+joint+'/command'
        return joint_name

    def reconfigure_cb(self, data):  # dynamics reconfigure callback
        self.config = data

    # callback function for robot pos
    def joints_pos_callback(self, cheetah_joints_pos):
        '''
        joints:
        ['FL_calf', 'FL_hip', 'FL_thigh', 
        'FR_calf', 'FR_hip', 'FR_thigh', 
        'RL_calf', 'RL_hip', 'RL_thigh', 
        'RR_calf', 'RR_hip', 'RR_thigh']
        '''
        self.joints_positions = dict(
            zip(cheetah_joints_pos.name, cheetah_joints_pos.position))
        self.joints_velocities = dict(
            zip(cheetah_joints_pos.name, cheetah_joints_pos.velocity))

    # callback function for robot pos
    def body_pos_callback(self, cheetah_links_pos):
        '''
        links:

        '''
        cheetah_links_pos.name
        link_poses = dict(zip(cheetah_links_pos.name, cheetah_links_pos.pose))
        link_twists = dict(
            zip(cheetah_links_pos.name, cheetah_links_pos.twist))

        self.body_position = link_poses['quadruped_robot::body'].position
        self.body_orientation = link_poses['quadruped_robot::body'].orientation
        self.body_twist_angular = link_twists['quadruped_robot::body'].angular
        self.body_twist_linear = link_twists['quadruped_robot::body'].linear

    def update_PD(self, Kp, Kd):
        if self.use_ros:
            for i in range(len(self.config.doubles)):
                if self.config.doubles[i].name == 'Kp_leg':
                    Kp[0, 0] = self.config.doubles[i].value
                elif self.config.doubles[i].name == 'Kp_hip':
                    Kp[1, 1] = self.config.doubles[i].value
                elif self.config.doubles[i].name == 'Kp_calf':
                    Kp[2, 2] = self.config.doubles[i].value
                if self.config.doubles[i].name == 'Kd_leg':
                    Kd[0, 0] = self.config.doubles[i].value
                elif self.config.doubles[i].name == 'Kd_hip':
                    Kd[1, 1] = self.config.doubles[i].value
                elif self.config.doubles[i].name == 'Kd_calf':
                    Kd[2, 2] = self.config.doubles[i].value
        return Kp, Kd

    def go_to_zero_leg(self, leg_name, motors_names, Kd, Kp):
        q_cur = np.array([[0],
                          [0],
                          [0]], dtype=np.float)

        q_dot_cur = np.array([[0],
                              [0],
                              [0]], dtype=np.float)
        if self.use_ros:
            for idx, motor in enumerate(motors_names):
                q_cur[idx] = self.joints_positions[motor]
                q_dot_cur[idx] = self.joints_velocities[motor]

        q_dot_goal = np.array([[0],
                               [0],
                               [0]], dtype=np.float)

        q_goal = np.array([[0],
                           [0],
                           [0]])  # "-" is because the z-axis is inverted for each motor
        U = np.dot(Kp, q_goal - q_cur) + np.dot(Kd, q_dot_goal - q_dot_cur)
        if self.type_of_control == 'torque':
            if self.use_ros:
                self.move_joint(motors_names[0], U[0])
                self.move_joint(motors_names[1], U[1])
                self.move_joint(motors_names[2], U[2])
        return [U[0][0], U[1][0], U[2][0]], 1

    def go_to_zero_all(self, Kp, Kd):
        
        U_sum = {'LF_leg':[], 'RF_leg':[], 'LB_leg':[], 'RB_leg': []}
        if self.use_ros:
            if self.check_zero_flag():
                U = self.go_to_zero_leg('LF_leg', self.LF_leg, Kd, Kp)
                U_sum['LF_leg'] = U
                U = self.go_to_zero_leg('LB_leg', self.LB_leg, Kd, Kp)
                U_sum['LB_leg'] = U
                U = self.go_to_zero_leg('RF_leg', self.RF_leg, Kd, Kp)
                U_sum['RF_leg'] = U
                U = self.go_to_zero_leg('RB_leg', self.RB_leg, Kd, Kp)
                U_sum['RB_leg'] = U
        else:
            U = self.go_to_zero_leg('LF_leg', self.LF_leg, Kd, Kp)
            U_sum['LF_leg'] = U
            U = self.go_to_zero_leg('LB_leg', self.LB_leg, Kd, Kp)
            U_sum['LB_leg'] = U
            U = self.go_to_zero_leg('RF_leg', self.RF_leg, Kd, Kp)
            U_sum['RF_leg'] = U
            U = self.go_to_zero_leg('RB_leg', self.RB_leg, Kd, Kp)
            U_sum['RB_leg'] = U
        return U_sum


    def check_zero_flag(self,):
        result = False
        for i in range(len(self.config.bools)):
            if self.config.bools[i].name == 'zero_pose':
                result = self.config.bools[i].value
        return result

    
    def motor_go_to_des_pos_wrt_body(self, leg_name, motors_names, pos_des, Kd, Kp, quad_kin, tmotors = {}):
        if self.use_ros:
            if self.check_zero_flag():
                return None, 0, None, None, None, None
        q_cur = np.array([[0],
                                [0],
                                [0]], dtype = np.float)
                    
        q_dot_cur = np.array([[0],
                        [0],
                        [0]], dtype = np.float)

        if self.use_ros:
            for idx, motor in enumerate(motors_names):
                q_cur[idx] = self.joints_positions[motor]
                q_dot_cur[idx] = self.joints_velocities[motor]
        else:
            for idx, motor in enumerate(tmotors[leg_name]):
                q_cur[idx] = motor.state['pos']
                q_dot_cur[idx] = motor.state['vel']
                if idx == 2:
                    q_cur[idx] = motor.state['pos']/1.5
                    q_dot_cur[idx] = motor.state['vel']/1.5
        x_des_,x_dot_des_, y_des_, y_dot_des_, z_des_, z_dot_des_ = pos_des[0], 0, pos_des[1], 0, pos_des[2], 0

        pos_des = [x_des_, y_des_, z_des_]
        vel_des = [x_dot_des_, y_dot_des_, z_dot_des_]
        if leg_name == "LF_leg":
            q_des = quad_kin.leg_ik(base = quad_kin.base_LF, pos = pos_des)

            q_dot_goal = 0
        elif leg_name == "LB_leg":
            q_des = quad_kin.leg_ik(base = quad_kin.base_LB, pos = pos_des, flag = 1)
            
            q_dot_goal = 0
        elif leg_name == "RB_leg":
            q_des = quad_kin.leg_ik(base = quad_kin.base_RB, pos = pos_des, flag_inv= 1)
            
            q_dot_goal = 0
        elif leg_name == "RF_leg":
            q_des = quad_kin.leg_ik(base = quad_kin.base_RF, pos = pos_des, flag = 1, flag_inv = 1)

            q_dot_goal = 0 
        else: print('wrong input to traj')
        

        q_goal =  np.array([[q_des[0]],
                            [q_des[1]],
                            [q_des[2]]]) 
        
        q_dot_goal = np.array([[0],
                                [0],
                                [0]], dtype = np.float)

        U = np.dot(Kp,q_goal - q_cur) + np.dot(Kd,q_dot_goal - q_dot_cur)
        if self.type_of_control == 'torque':
            if self.use_ros:
                self.move_joint(motors_names[0], U[0])
                self.move_joint(motors_names[1], U[1])
                self.move_joint(motors_names[2], U[2])
        else:
            pass
        return [U[0][0], U[1][0], U[2][0]], 1, q_cur, q_goal, q_dot_cur, q_dot_goal
        

    def go_to_desired_RPY_of_base(self, quad_kin, LF_leg_pos, RF_leg_pos, LB_leg_pos, RB_leg_pos, Kd, Kp, tmotors = {}, rpy = [0,0,0], xyz = [0,0,0.425], use_input_traj = False):
        def calc(leg_name, leg_pos, leg):
            P_leg_0 = np.array([[leg_pos[0]],
                                [leg_pos[1]],
                                [leg_pos[2]],
                                [1]])
            p_leg_B = np.dot(T_B_0_inv, P_leg_0)[0:3].reshape(3,)
            U, flag, q_c, q_d, q_c_dot, q_d_dot = self.motor_go_to_des_pos_wrt_body(leg_name, leg, p_leg_B, Kd, Kp, quad_kin, tmotors)
            if flag:
                U_sum[leg_name] = U
                q_cur[leg_name] = q_c
                q_cur_dot[leg_name] = q_c_dot
                q_des_dot[leg_name] = q_d_dot
                q_des[leg_name] = q_d

        if self.use_ros and not use_input_traj:
            if not self.check_zero_flag():
                for i in range(len(self.config.doubles)):
                    if self.config.doubles[i].name == 'x_body_des':
                        x_des = self.config.doubles[i].value
                    elif self.config.doubles[i].name == 'y_body_des':
                        y_des = self.config.doubles[i].value
                    elif self.config.doubles[i].name == 'z_body_des':
                        z_des = self.config.doubles[i].value
                    if self.config.doubles[i].name == 'R_des':
                        roll = self.config.doubles[i].value
                    elif self.config.doubles[i].name == 'P_des':
                        pitch = self.config.doubles[i].value
                    elif self.config.doubles[i].name == 'Y_des':
                        yaw = self.config.doubles[i].value
            else:
                x_des, y_des, z_des, roll, pitch, yaw =  0,0,0.425, 0,0,0
        elif use_input_traj:
            x_des, y_des, z_des, roll, pitch, yaw =  xyz[0], xyz[1], xyz[2], rpy[0], rpy[1], rpy[2]

        R =  np.linalg.multi_dot([quad_kin.transforms.Rz(yaw), quad_kin.transforms.Ry(pitch), quad_kin.transforms.Rx(roll)])[0:3,0:3]
        d = np.array([[x_des],
                        [y_des],
                        [z_des]], dtype = np.float)
        T_B_0_inv = np.zeros((4,4))
        T_B_0_inv[0:3,0:3] = np.transpose(R)
        T_B_0_inv[0:3,3] = -np.dot(np.transpose(R),d).reshape(3,)
        T_B_0_inv[3,3] = 1

        U_sum = {'LF_leg':[], 'RF_leg':[], 'LB_leg':[], 'RB_leg': []}
        q_cur = copy.deepcopy(U_sum)
        q_cur_dot = copy.deepcopy(U_sum)
        q_des_dot = copy.deepcopy(U_sum)
        q_des = copy.deepcopy(U_sum)
        calc('RF_leg', RF_leg_pos, self.RF_leg)
        calc('LF_leg', LF_leg_pos, self.LF_leg)
        calc('RB_leg', RB_leg_pos, self.RB_leg)
        calc('LB_leg', LB_leg_pos, self.LB_leg)
        return U_sum, 1, q_cur, q_cur_dot, q_des, q_des_dot

    def motor_set_torque(self,motor, U):
        # print("Function a is running at time: " + str(int(time.time())) + " seconds.")

        motor[0].set_torque(U[0])
        motor[1].set_torque(U[1])
        motor[2].set_torque(U[2])

    def motor_set_pos(self,motor, pos, Kp, Kd):
        motor[0].set_pos(pos[0], Kp[0], Kd[0])
        motor[1].set_pos(pos[1], Kp[1], Kd[1])
        motor[2].set_pos(pos[2], Kp[2], Kd[2])
        