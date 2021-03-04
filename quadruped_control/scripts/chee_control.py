#!/usr/bin/env python3
# license removed for brevity
import rospy
import time
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
class cheetah_control():
    def __init__(self, type_of_control = 'position', time_pause_before_control = 0, rate_value = 1000):
        rospy.init_node('cheetah_control', anonymous=True)
        self.type_of_control = type_of_control
        self.time_pause_before_control = time_pause_before_control
        time.sleep(self.time_pause_before_control)
        
        self.joints = {'FL_hip': ['left_forward_motor_leg'] , 
                    'FL_thigh' : ['left_forward_leg_hip'], 
                     'FL_calf' : ['left_forward_hip_calf'],

                       'FR_hip' : ['right_forward_motor_leg'], 
                     'FR_thigh' : ['right_forward_leg_hip'], 
                      'FR_calf' : ['right_forward_hip_calf'],

                       'RL_hip': ['left_back_motor_leg'],
                     'RL_thigh': ['left_back_leg_hip'], 
                      'RL_calf': ['left_back_hip_calf'],

                       'RR_hip' : ['right_back_motor_leg'], 
                     'RR_thigh' : ['right_back_leg_hip'], 
                       'RR_calf': ['right_back_hip_calf']}

        if type_of_control == 'position':
            self.controllers = list(map(lambda x: x + '_position_controller', self.joints))
        elif type_of_control == 'torque':
            self.controllers = list(map(lambda x: x + '_Effort_controller', self.joints))
        self.pub = {}
        i = 0
        
        for joint in self.joints:
            self.joints[joint].append(self.controllers[i])
            self.pub[joint]=rospy.Publisher(self.joint_name(self.joints[joint][1]), Float64, queue_size=10)
            i += 1

        self.cheetah_joints_sub = rospy.Subscriber("/quadruped/joint_states", JointState, self.joints_pos_callback)
        self.rate = rospy.Rate(rate_value)
        self.joints_positions = {}
        self.joints_velocities = {}
    def move_joint(self,joint, pos):
        """
        i - number of joint: 
        1: 'FL_hip_controller', 
        2: 'FL_thigh_controller', 
        3: 'FL_calf_controller',
        4: 'FR_hip_controller', 
        5: 'FR_thigh_controller', 
        6: 'FR_calf_controller',
        7: 'RL_hip_controller', 
        8: 'RL_thigh_controller', 
        9: 'RL_calf_controller',
        10: 'RR_hip_controller', 
        11: 'RR_thigh_controller', 
        12: 'RR_calf_controller'
        pos - angle in rad
        """	
        self.pub[joint].publish(pos)	

    def joint_name(self, joint):
        joint_name = '/quadruped/'+joint+'/command'
        return joint_name

    #callback function for robot pos
    def joints_pos_callback(self, cheetah_joints_pos):
        '''
        joints:
        ['FL_calf_joint', 'FL_hip_joint', 'FL_thigh_joint', 
        'FR_calf_joint', 'FR_hip_joint', 'FR_thigh_joint', 
        'RL_calf_joint', 'RL_hip_joint', 'RL_thigh_joint', 
        'RR_calf_joint', 'RR_hip_joint', 'RR_thigh_joint']
        '''
        self.joints_positions = dict(zip(cheetah_joints_pos.name, cheetah_joints_pos.position))
        self.joints_velocities = dict(zip(cheetah_joints_pos.name, cheetah_joints_pos.velocity))