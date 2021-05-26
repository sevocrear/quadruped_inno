import os
import sys
import numpy as np
import sympy as sym
import unittest

abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
sys.path.append("../..")
from quadruped_kinematics import quadruped_kinematics

class kinematics_test(unittest.TestCase):
    def test_FK(self):
        print('Checking FK solutions for all legs...')
        links_sizes_mm = [162.75, 65, 72.25, 82.25,
                      208, 159, 58.5]  # leg links sizes
        link_sizes = [link_size / 1000 for link_size in links_sizes_mm]
        quad_kin = quadruped_kinematics(link_sizes)
        x,y,z = quad_kin.lf_leg_fk([0,0,0],base = quad_kin.base_LF)
        x_des, y_des, z_des = [ 0.14725, 
                                -0.235,   
                                -0.4255 ]
        self.assertAlmostEqual(x,x_des, places = 5)
        self.assertAlmostEqual(y,y_des, places = 5)
        self.assertAlmostEqual(z,z_des, places = 5)

        x,y,z = quad_kin.lf_leg_fk([0,0,np.pi/2],base = quad_kin.base_LF)
        x_des, y_des, z_des = [ 0.14725,
                     -0.235+(links_sizes_mm[5]+links_sizes_mm[6])/1000,
                        -0.4255+(links_sizes_mm[5]+links_sizes_mm[6])/1000 ]
        self.assertAlmostEqual(x,x_des, places = 5)
        self.assertAlmostEqual(y,y_des, places = 5)
        self.assertAlmostEqual(z,z_des, places = 5)

        x,y,z = quad_kin.lf_leg_fk([np.pi/2,0,0],base = quad_kin.base_LF)
        x_des, y_des, z_des = [ 0.14725 + (links_sizes_mm[4]+links_sizes_mm[5]+links_sizes_mm[6]-links_sizes_mm[3])/1000,
                             -0.235,   
                             links_sizes_mm[3]/1000]
        self.assertAlmostEqual(x,x_des, places = 5)
        self.assertAlmostEqual(y,y_des, places = 5)
        self.assertAlmostEqual(z,z_des, places = 5)

        x,y,z = quad_kin.lf_leg_fk([np.pi/2,0,np.pi/2],base = quad_kin.base_LF)
        x_des, y_des, z_des = [ 0.14725 + (links_sizes_mm[4]-links_sizes_mm[3])/1000,
                                 -0.235 + (links_sizes_mm[5]+links_sizes_mm[6])/1000,
                                links_sizes_mm[3]/1000]
        self.assertAlmostEqual(x,x_des, places = 5)
        self.assertAlmostEqual(y,y_des, places = 5)
        self.assertAlmostEqual(z,z_des, places = 5)

        x,y,z = quad_kin.lb_leg_fk([0,0,0],base = quad_kin.base_LB)
        x_des, y_des, z_des = [ 0.14725, 
                                0.235,   
                                -0.4255 ]
        self.assertAlmostEqual(x,x_des, places = 5)
        self.assertAlmostEqual(y,y_des, places = 5)
        self.assertAlmostEqual(z,z_des, places = 5)

        x,y,z = quad_kin.lb_leg_fk([-np.pi/2,0,0],base = quad_kin.base_LB)
        x_des, y_des, z_des = [ 0.14725 + (link_sizes[4]+link_sizes[5]+link_sizes[6]-link_sizes[3]),
                             0.235,   
                             link_sizes[3]]
        self.assertAlmostEqual(x,x_des, places = 5)
        self.assertAlmostEqual(y,y_des, places = 5)
        self.assertAlmostEqual(z,z_des, places = 5)

        x,y,z = quad_kin.lb_leg_fk([-np.pi/2,0,0],base = quad_kin.base_LB)
        x_des, y_des, z_des = [ 0.14725 + (link_sizes[4]+link_sizes[5]+link_sizes[6]-link_sizes[3]),
                             0.235,   
                             link_sizes[3]]
        self.assertAlmostEqual(x,x_des, places = 5)
        self.assertAlmostEqual(y,y_des, places = 5)
        self.assertAlmostEqual(z,z_des, places = 5)

        x,y,z = quad_kin.lb_leg_fk([-np.pi/2,0,-np.pi/2],base = quad_kin.base_LB)
        x_des, y_des, z_des = [ 0.14725 + (link_sizes[4]-link_sizes[3]),
                                 0.235 - (link_sizes[5]+link_sizes[6]),
                                link_sizes[3]]
        self.assertAlmostEqual(x,x_des, places = 5)
        self.assertAlmostEqual(y,y_des, places = 5)
        self.assertAlmostEqual(z,z_des, places = 5)

    def test_IK(self):
        print('Checking IK solutions for all legs...')
        links_sizes_mm = [162.75, 65, 72.25, 82.25,
                      208, 159, 58.5]  # leg links sizes
        link_sizes = [link_size / 1000 for link_size in links_sizes_mm]
        quad_kin = quadruped_kinematics(link_sizes)
        q1_des, q2_des, q3_des = [ 0, 
                                   0,   
                                   0]
        x,y,z = quad_kin.lf_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_LF)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_LF, pos = [x,y,z], flag_up = 1)
        self.assertAlmostEqual(q1,q1_des, places = 5)
        self.assertAlmostEqual(q2,q2_des, places = 5)
        self.assertAlmostEqual(q3,q3_des, places = 5)

        q1_des, q2_des, q3_des = [ np.pi/8, 
                                   0,   
                                   0]
        x,y,z = quad_kin.lf_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_LF)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_LF, pos = [x,y,z], flag_up = 1)
        self.assertAlmostEqual(q1,q1_des, places = 5)
        self.assertAlmostEqual(q2,q2_des, places = 5)
        self.assertAlmostEqual(q3,q3_des, places = 5)

        q1_des, q2_des, q3_des = [ 0, 
                                   np.pi/4,   
                                   -np.pi/2]
        x,y,z = quad_kin.lf_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_LF)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_LF, pos = [x,y,z], flag_up = 1)
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ 0, 
                                   np.pi/3,   
                                   -np.pi/4]
        x,y,z = quad_kin.lf_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_LF)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_LF, pos = [x,y,z], flag_up = 1)
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ 0, 
                                   3*np.pi/4,   
                                   -3*np.pi/4]
        x,y,z = quad_kin.lf_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_LF)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_LF, pos = [x,y,z], flag_up = 1)
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ 0, 
                                   -3*np.pi/4,   
                                   +3*np.pi/4]
        x,y,z = quad_kin.lf_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_LB)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_LB, pos = [x,y,z], flag_up = 0)
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ 0, 
                                   -3*np.pi/4,   
                                   +3*np.pi/4]
        x,y,z = quad_kin.lf_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_LB)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_LB, pos = [x,y,z], flag_up = 0)
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ 0, 
                                   0,   
                                   0]
        x,y,z = quad_kin.lb_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_LB)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_LB, pos = [x,y,z], flag_up = 0)
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ 0, 
                                   np.pi/3,   
                                   -np.pi/4]
        x,y,z = quad_kin.lb_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_LB)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_LB, pos = [x,y,z], flag_up = 0, leg = 'LB')
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ 0, 
                                   np.pi/4,   
                                   -np.pi/4]
        x,y,z = quad_kin.lb_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_LB)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_LB, pos = [x,y,z], flag_up = 0, leg = 'LB')
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ 0, 
                                   np.pi/4,   
                                   -np.pi/3]
        x,y,z = quad_kin.lb_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_LB)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_LB, pos = [x,y,z], flag_up = 0, leg = 'LB')
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ -np.pi/6, 
                                   np.pi/4,   
                                   -np.pi/3]
        x,y,z = quad_kin.lb_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_LB)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_LB, pos = [x,y,z], flag_up = 0, leg = 'LB')
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ np.pi/6, 
                                   np.pi/4,   
                                   -np.pi/3]
        x,y,z = quad_kin.lb_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_LB)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_LB, pos = [x,y,z], flag_up = 0, leg = 'LB')
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ np.pi/6, 
                                   3*np.pi/4,   
                                   -2*np.pi/3]
        x,y,z = quad_kin.lb_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_LB)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_LB, pos = [x,y,z], flag_up = 0, leg = 'LB')
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        # RF_leg
        q1_des, q2_des, q3_des = [ 0, 
                                   -np.pi/4,   
                                   np.pi/3]                           
        x,y,z = quad_kin.lb_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_RF)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_RF, pos = [x,y,z], flag_up = 0, leg = 'RF')
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ -np.pi/6, 
                                   -np.pi/4,   
                                   np.pi/3]                           
        x,y,z = quad_kin.lb_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_RF)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_RF, pos = [x,y,z], flag_up = 0, leg = 'RF')
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ -np.pi/6, 
                                   -3*np.pi/4,   
                                   2*np.pi/3]                           
        x,y,z = quad_kin.lb_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_RF)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_RF, pos = [x,y,z], flag_up = 0, leg = 'RF')
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ 0, 
                                   0,   
                                   0]                           
        x,y,z = quad_kin.lb_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_RF)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_RF, pos = [x,y,z], flag_up = 0, leg = 'RF')
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)


        #RB leg
        q1_des, q2_des, q3_des = [ 0, 
                                   0,   
                                   0]                           
        x,y,z = quad_kin.lf_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_RB)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_RB, pos = [x,y,z], flag_up = 0, leg = 'RB')
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ 0, 
                                   -np.pi/6,   
                                   np.pi/6]                           
        x,y,z = quad_kin.lf_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_RB)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_RB, pos = [x,y,z], flag_up = 0, leg = 'RB')
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ 0, 
                                   -np.pi/4,   
                                   np.pi/6]                           
        x,y,z = quad_kin.lf_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_RB)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_RB, pos = [x,y,z], flag_up = 0, leg = 'RB')
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ 0, 
                                   -np.pi/6,   
                                   np.pi/4]                           
        x,y,z = quad_kin.lf_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_RB)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_RB, pos = [x,y,z], flag_up = 0, leg = 'RB')
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        q1_des, q2_des, q3_des = [ 0, 
                                   -3*np.pi/4,   
                                   2*np.pi/3]                           
        x,y,z = quad_kin.lf_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_RB)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_RB, pos = [x,y,z], flag_up = 0, leg = 'RB')
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)
        q1_des, q2_des, q3_des = [ np.pi/6, 
                                   -3*np.pi/4,   
                                   2*np.pi/3]                           
        x,y,z = quad_kin.lf_leg_fk([q1_des,q2_des,q3_des],base = quad_kin.base_RB)
        q1,q2,q3 = quad_kin.leg_ik(base = quad_kin.base_RB, pos = [x,y,z], flag_up = 0, leg = 'RB')
        self.assertAlmostEqual(q1,q1_des, places = 3)
        self.assertAlmostEqual(q3,q3_des, places = 3)
        self.assertAlmostEqual(q2,q2_des, places = 3)

        
        

        
if __name__ == '__main__':
    unittest.main()