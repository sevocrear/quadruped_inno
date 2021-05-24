import os
import sys

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
sys.path.append("../..")
from quadruped_kinematics import quadruped_kinematics

links_sizes_mm = [162.75, 65, 72.25, 82.25,
                      208, 159, 58.5]  # leg links sizes
quad_kin = quadruped_kinematics(
    [link_size / 1000 for link_size in links_sizes_mm])

pos_des = [0.14725, -0.15,   -0.25]

q_des = quad_kin.leg_ik(base = quad_kin.base_LF, pos = pos_des)

p_m, p_h, p_c, p_f = quad_kin.lf_leg_fk(q_des,base = quad_kin.base_LF)

print(p_f,'\n', pos_des)

pos_des = [-0.14725, 0.15,   -0.25 ] #
q_des = quad_kin.leg_ik(base = quad_kin.base_RB, pos = pos_des, flag_inv = 1)
p_m, p_h, p_c, p_f = quad_kin.lf_leg_fk(q_des,base = quad_kin.base_RB)

print(p_f,'\n', pos_des)

pos_des = [-0.14725, -0.15,   -0.25 ]

q_des = quad_kin.leg_ik(base = quad_kin.base_RF, pos = pos_des, flag = 1 , flag_inv = 1)
p_m, p_h, p_c, p_f = quad_kin.lb_leg_fk(q_des,base = quad_kin.base_RF)

print(p_f,'\n', pos_des)

pos_des = [0.14725, 0.15,   -0.25 ] #
q_des = quad_kin.leg_ik(base = quad_kin.base_LB, pos = pos_des, flag = 1)
p_m, p_h, p_c, p_f = quad_kin.lb_leg_fk(q_des,base = quad_kin.base_LB)

print(p_f, '\n',pos_des)


        #     q_dot_goal = 0
        # elif leg_name == "LB_leg":
        #     q_des = quad_kin.leg_ik(base = quad_kin.base_LB, pos = pos_des, flag = 1)
            
        #     q_dot_goal = 0
        # elif leg_name == "RB_leg":
        #     q_des = quad_kin.leg_ik(base = quad_kin.base_RB, pos = pos_des, flag_inv= 1)
            
        #     q_dot_goal = 0
        # elif leg_name == "RF_leg":
        #     q_des = quad_kin.leg_ik(base = quad_kin.base_RF, pos = pos_des, flag = 1, flag_inv = 1)

        #     q_dot_goal = 0 