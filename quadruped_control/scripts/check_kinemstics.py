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

q_des = [0.3,0.2,0.2]
q_des = [0.3,0.2,-0.2]
q_des = [0.3,-0.2,0.2]
q_des = [0.3,-0.2,-0.2]
q_des = [-0.3,0.2,0.2]
q_des = [-0.3,0.2,-0.2]
q_des = [-0.3,-0.2,0.2]
q_des = [-0.3,-0.2,-0.2]
p_m, p_h, p_c, p_f = quad_kin.lf_leg_fk(q_des,base = quad_kin.base_LF)
q_ik = quad_kin.leg_ik(base = quad_kin.base_LF, pos = p_f)
print(q_des,'\n',list(map(lambda x: round(x,2),q_ik)))

p_m, p_h, p_c, p_f = quad_kin.lb_leg_fk(q_des,base = quad_kin.base_LB)
q_ik = quad_kin.leg_ik(base = quad_kin.base_LB, pos = p_f, flag_lf=0)
print(q_des,'\n',list(map(lambda x: round(x,2),q_ik)))

p_m, p_h, p_c, p_f = quad_kin.lf_leg_fk(q_des,base = quad_kin.base_RB)
q_ik = quad_kin.leg_ik(base = quad_kin.base_RB, pos = p_f)
print(q_des,'\n',list(map(lambda x: round(x,2),q_ik)))

p_m, p_h, p_c, p_f = quad_kin.lb_leg_fk(q_des,base = quad_kin.base_RF)
q_ik = quad_kin.leg_ik(base = quad_kin.base_RF, pos = p_f)
print(q_des,'\n',list(map(lambda x: round(x,2),q_ik)))