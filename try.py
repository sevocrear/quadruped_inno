import numpy as np
Jac = np.eye(3,3)*7
vel_des = [1,2,3]
Kp = np.eye(3,3)*25
Kd = np.eye(3,3)*3
q_dot_des = - np.dot(np.linalg.inv(Jac), np.array([[vel_des[0]],
                                                                             [vel_des[1]],
                                                                             [vel_des[2]]]))  # "-" is because the z-axis is inverted for each motor
q_des = [0,np.pi/4,-np.pi/3]
q_des = - np.array([[q_des[0]],
                [q_des[1]],
                [q_des[2]]]) # "-" is because the z-axis is inverted for each motor

q_cur = [0,np.pi/3,-np.pi/2]
q_cur = np.array([[q_cur[0]],
                [q_cur[1]],
                [q_cur[2]]])
q_dot_cur = [0,np.pi/3,-np.pi/2]
q_dot_cur = np.array([[q_dot_cur[0]],
                [q_dot_cur[1]],
                [q_dot_cur[2]]])
u = np.dot(Kp, q_des - q_cur) + np.dot(Kd, q_dot_des - q_dot_cur)
print(u[0], u[1], u[2])