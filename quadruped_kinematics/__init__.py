
import numpy as np
import sympy as sym
import warnings
sym.init_printing(use_unicode=False, wrap_line=False)
warnings.simplefilter("error", RuntimeWarning)


class quadruped_kinematics():
    def __init__(self, links_size):
        self.links_size = links_size
        self.transforms = self.transforms()
        self.base_LF = np.linalg.multi_dot(
            [self.transforms.Ty(-self.links_size[0]), self.transforms.Tx(self.links_size[1]), self.transforms.Rx(np.pi/2)])

        self.base_LB = np.linalg.multi_dot([self.transforms.Ty(
            self.links_size[0]), self.transforms.Tx(self.links_size[1]), self.transforms.Rx(-np.pi/2)])

        self.base_RF = np.linalg.multi_dot([self.transforms.Ty(-self.links_size[0]), self.transforms.Tx(
            -self.links_size[1]), self.transforms.Rz(np.pi), self.transforms.Rx(-np.pi/2)])

        self.base_RB = np.linalg.multi_dot([self.transforms.Ty(self.links_size[0]), self.transforms.Tx(
            -self.links_size[1]), self.transforms.Rz(np.pi), self.transforms.Rx(np.pi/2)])

    def lf_leg_fk(self, angles, base):
        p_m = base[0:3, 3]
        T_b_h = np.linalg.multi_dot([base, self.transforms.Rz(angles[0]), self.transforms.Tz(
            self.links_size[2]), self.transforms.Tx(self.links_size[3]), self.transforms.Ry(np.pi/2)])  # transform brom base to hip
        p_h = T_b_h[0:3, 3]
        T_h_c = np.linalg.multi_dot([self.transforms.Rz(
            angles[1]), self.transforms.Ty(-self.links_size[4])])   # transform from hip to calf
        T_c_f = np.linalg.multi_dot([self.transforms.Rz(np.pi), self.transforms.Rz(
            angles[2]), self.transforms.Ty(self.links_size[5] + self.links_size[6])])  # transform from calf to foot

        T_b_f = np.linalg.multi_dot([T_b_h, T_h_c, T_c_f])
        return T_b_f[0:3, 3]

    def lb_leg_fk(self, angles, base):
        T_b_h = np.linalg.multi_dot([base, self.transforms.Rz(angles[0]), self.transforms.Tz(
            self.links_size[2]), self.transforms.Tx(self.links_size[3]), self.transforms.Ry(np.pi/2)])  # transform brom base to hip
        T_h_c = np.linalg.multi_dot([self.transforms.Rz(angles[1]), self.transforms.Ty(
            self.links_size[4])])   # transform from hip to calf
        T_c_f = np.linalg.multi_dot([self.transforms.Rz(angles[2]), self.transforms.Ty(
            self.links_size[5] + self.links_size[6])])  # transform from calf to foot

        T_b_f = np.linalg.multi_dot([T_b_h, T_h_c, T_c_f])
        return T_b_f[0:3, 3]

    def leg_ik(self, base, pos, flag_up=0, leg = 'LF'):
        '''
        method for solving inverse kinematics of each quadruped leg
        '''
        try:
            base_inv = np.zeros((4, 4))
            base_inv[0:3, 0:3] = base[0:3, 0:3].T
            base_inv[0:3, 3] = -np.dot(base[0:3, 0:3].T, base[0:3, 3])
            base_inv[3, 3] = 1
            pos = np.dot(base_inv, np.array(
                [pos[0], pos[1], pos[2], 1]).reshape(4, 1))
            x, y, z = pos[0, 0], pos[1, 0], pos[2, 0]

            r = np.sqrt(x**2 + y**2)
            phi = np.arctan2(abs(y), abs(x))
            alpha = np.arctan2(
                np.sqrt(r**2-self.links_size[3]**2), self.links_size[3])

            if x >= 0:
                theta_1 = alpha - phi
            if x < 0:
                theta_1 = alpha + phi - np.pi

            r = np.sqrt(x**2 + y**2 - self.links_size[3]**2)

            s = z - self.links_size[2]

            D = (r**2 + s**2 - self.links_size[4]**2 - (self.links_size[5] + self.links_size[6])**2)/(
                2*self.links_size[4]*(self.links_size[5]+self.links_size[6]))
            D = round(D, 6)
            if flag_up:
                theta_3 = -np.arctan2(np.sqrt(1-D**2), D)
            else:
                theta_3 = +np.arctan2(np.sqrt(1-D**2), D)

            gamma = np.arctan2(s, r)

            theta_2 = -gamma - np.arctan2((self.links_size[5] + self.links_size[6])*np.sin(
                theta_3), self.links_size[4]+(self.links_size[5] + self.links_size[6])*np.cos(theta_3))
            if leg == 'RF':
                theta_2 = gamma - np.arctan2((self.links_size[5] + self.links_size[6])*np.sin(
                theta_3), self.links_size[4]+(self.links_size[5] + self.links_size[6])*np.cos(theta_3))
            if leg == 'LB':
                theta_1 = -theta_1
                theta_2 = -theta_2
                theta_3 = -theta_3
            if leg == 'RF':
                theta_1 = -theta_1
            return [theta_1, theta_2, theta_3]
        except RuntimeWarning:
            print('inapropriate pos! Not possible. Going to the zero pos')
            return [0, 0, 0]

    class transforms():
        def __init__(self,):
            pass

        def Tx(self, a):
            Tx = np.eye(4, 4)
            Tx[0, 3] = a
            return Tx

        def Ty(self, a):
            Ty = np.eye(4, 4)
            Ty[1, 3] = a
            return Ty

        def Tz(self, a):
            Tz = np.eye(4, 4)
            Tz[2, 3] = a
            return Tz

        def Rx(self, alpha):
            Rx = np.eye(4, 4)
            Rx[1, 1] = np.cos(alpha)
            Rx[1, 2] = -np.sin(alpha)
            Rx[2, 1] = np.sin(alpha)
            Rx[2, 2] = np.cos(alpha)
            return Rx

        def Ry(self, alpha):
            Ry = np.eye(4, 4)
            Ry[0, 0] = np.cos(alpha)
            Ry[0, 2] = np.sin(alpha)
            Ry[2, 0] = -np.sin(alpha)
            Ry[2, 2] = np.cos(alpha)
            return Ry

        def Rz(self, alpha):
            Rz = np.eye(4, 4)
            Rz[0, 0] = np.cos(alpha)
            Rz[0, 1] = -np.sin(alpha)
            Rz[1, 0] = np.sin(alpha)
            Rz[1, 1] = np.cos(alpha)
            return Rz

        def Tx_sym(self, a):
            Tx = sym.eye(4, 4)
            Tx[0, 3] = a
            return Tx

        def Ty_sym(self, a):
            Ty = sym.eye(4, 4)
            Ty[1, 3] = a
            return Ty

        def Tz_sym(self, a):
            Tz = sym.eye(4, 4)
            Tz[2, 3] = a
            return Tz

        def Rx_sym(self, alpha=0):
            Rx = sym.eye(4, 4)
            if isinstance(alpha, str):
                alpha = sym.symbols('alpha')
            Rx[1, 1] = sym.cos(alpha)
            Rx[1, 2] = -sym.sin(alpha)
            Rx[2, 1] = sym.sin(alpha)
            Rx[2, 2] = sym.cos(alpha)
            return Rx

        def Ry_sym(self, beta=0):
            Ry = sym.eye(4, 4)
            if type(beta) == str:
                beta = sym.symbols('beta')
            Ry[0, 0] = sym.cos(beta)
            Ry[0, 2] = sym.sin(beta)
            Ry[2, 0] = -sym.sin(beta)
            Ry[2, 2] = sym.cos(beta)
            return Ry

        def Rz_sym(self, gamma=0):
            Rz = sym.eye(4, 4)
            if type(gamma) == str:
                gamma = sym.symbols(gamma)
            Rz[0, 0] = sym.cos(gamma)
            Rz[0, 1] = -sym.sin(gamma)
            Rz[1, 0] = sym.sin(gamma)
            Rz[1, 1] = sym.cos(gamma)
            return Rz
