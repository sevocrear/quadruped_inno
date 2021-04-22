from quadruped_kinematics import quadruped_kinematics
import numpy as np
import sympy as sym

links_sizes_mm = [162.75, 65, 72.25, 82.25, 208, 159, 58.5]
quad_kin = quadruped_kinematics([link_size / 1000 for link_size in links_sizes_mm])

import matplotlib.pyplot as plt
import mpl_toolkits.mplot3d.axes3d as p3
import matplotlib.animation as animation
import time

A = np.pi/10
w = 1
N = 50
t = np.linspace(0,10,50)
theta_1 = list(map(lambda x: np.pi/18, t))
theta_2= list(map(lambda x: A*np.sin(w*x), t))
theta_3 = list(map(lambda x: -A*np.sin(w*x), t))

poses_motor_0 = np.zeros((N,3))
poses_hip_0 = np.zeros((N,3))
poses_calf_0 = np.zeros((N,3))
poses_foot_0 = np.zeros((N,3))

poses_motor_1 = np.zeros((N,3))
poses_hip_1 = np.zeros((N,3))
poses_calf_1 = np.zeros((N,3))
poses_foot_1 = np.zeros((N,3))

poses_motor_2 = np.zeros((N,3))
poses_hip_2 = np.zeros((N,3))
poses_calf_2 = np.zeros((N,3))
poses_foot_2 = np.zeros((N,3))

poses_motor_3 = np.zeros((N,3))
poses_hip_3 = np.zeros((N,3))
poses_calf_3 = np.zeros((N,3))
poses_foot_3 = np.zeros((N,3))

for i in range(N):
    p_m, p_h, p_c, p_f=quad_kin.lf_leg_fk([theta_1[i],theta_2[i],theta_3[i]], quad_kin.base_LF)
    poses_motor_0[i] = p_m
    poses_hip_0[i] = p_h
    poses_calf_0[i] = p_c
    poses_foot_0[i] = p_f

    p_m, p_h, p_c, p_f=quad_kin.lf_leg_fk([theta_1[i],-theta_2[i],-theta_3[i]], quad_kin.base_RB)
    poses_motor_1[i] = p_m
    poses_hip_1[i] = p_h
    poses_calf_1[i] = p_c
    poses_foot_1[i] = p_f

    p_m, p_h, p_c, p_f=quad_kin.lb_leg_fk([-theta_1[i],theta_2[i],theta_3[i]], quad_kin.base_LB)
    poses_motor_2[i] = p_m
    poses_hip_2[i] = p_h
    poses_calf_2[i] = p_c
    poses_foot_2[i] = p_f

    p_m, p_h, p_c, p_f=quad_kin.lb_leg_fk([-theta_1[i],-theta_2[i],-theta_3[i]], quad_kin.base_RF)
    poses_motor_3[i] = p_m
    poses_hip_3[i] = p_h
    poses_calf_3[i] = p_c
    poses_foot_3[i] = p_f
def animate(i):
  
  line1.set_data(np.array([poses_motor_0[i][0], poses_hip_0[i][0]]), np.array([poses_motor_0[i][1], poses_hip_0[i][1]]))
  line1.set_3d_properties( np.array([poses_motor_0[i][2], poses_hip_0[i][2]]))
  line2.set_data(np.array([poses_hip_0[i][0], poses_calf_0[i][0]]), np.array([poses_hip_0[i][1], poses_calf_0[i][1]]))
  line2.set_3d_properties( np.array([poses_hip_0[i][2], poses_calf_0[i][2]]))
  line3.set_data(np.array([poses_calf_0[i][0], poses_foot_0[i][0]]), np.array([poses_calf_0[i][1], poses_foot_0[i][1]]))
  line3.set_3d_properties( np.array([poses_calf_0[i][2], poses_foot_0[i][2]]))

  line4.set_data(np.array([poses_motor_1[i][0], poses_hip_1[i][0]]), np.array([poses_motor_1[i][1], poses_hip_1[i][1]]))
  line4.set_3d_properties( np.array([poses_motor_1[i][2], poses_hip_1[i][2]]))
  line5.set_data(np.array([poses_hip_1[i][0], poses_calf_1[i][0]]), np.array([poses_hip_1[i][1], poses_calf_1[i][1]]))
  line5.set_3d_properties( np.array([poses_hip_1[i][2], poses_calf_1[i][2]]))
  line6.set_data(np.array([poses_calf_1[i][0], poses_foot_1[i][0]]), np.array([poses_calf_1[i][1], poses_foot_1[i][1]]))
  line6.set_3d_properties( np.array([poses_calf_1[i][2], poses_foot_1[i][2]]))

  line7.set_data(np.array([poses_motor_2[i][0], poses_hip_2[i][0]]), np.array([poses_motor_2[i][1], poses_hip_2[i][1]]))
  line7.set_3d_properties( np.array([poses_motor_2[i][2], poses_hip_2[i][2]]))
  line8.set_data(np.array([poses_hip_2[i][0], poses_calf_2[i][0]]), np.array([poses_hip_2[i][1], poses_calf_2[i][1]]))
  line8.set_3d_properties( np.array([poses_hip_2[i][2], poses_calf_2[i][2]]))
  line9.set_data(np.array([poses_calf_2[i][0], poses_foot_2[i][0]]), np.array([poses_calf_2[i][1], poses_foot_2[i][1]]))
  line9.set_3d_properties( np.array([poses_calf_2[i][2], poses_foot_2[i][2]]))

  line10.set_data(np.array([poses_motor_3[i][0], poses_hip_3[i][0]]), np.array([poses_motor_3[i][1], poses_hip_3[i][1]]))
  line10.set_3d_properties( np.array([poses_motor_3[i][2], poses_hip_3[i][2]]))
  line11.set_data(np.array([poses_hip_3[i][0], poses_calf_3[i][0]]), np.array([poses_hip_3[i][1], poses_calf_3[i][1]]))
  line11.set_3d_properties( np.array([poses_hip_3[i][2], poses_calf_3[i][2]]))
  line12.set_data(np.array([poses_calf_3[i][0], poses_foot_3[i][0]]), np.array([poses_calf_3[i][1], poses_foot_3[i][1]]))
  line12.set_3d_properties( np.array([poses_calf_3[i][2], poses_foot_3[i][2]]))


#   fk_text.set_text(f'Forward kinematics')

#   ax.view_init(60, i/2) # Change point of view
  return (line1,line2, line3, line4, line5, line6, line7, line8, line9, line10, line11, line12)


# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)
# plt.close()
line1, = ax.plot([], [],[], lw=2, color = 'r')  
line2, = ax.plot([], [], [],lw=2, color = 'g')  
line3, = ax.plot([], [], [],lw=2, color = 'b')  

line4, = ax.plot([], [],[], lw=2, color = 'r')  
line5, = ax.plot([], [], [],lw=2, color = 'g')  
line6, = ax.plot([], [], [],lw=2, color = 'b')  

line7, = ax.plot([], [],[], lw=2, color = 'r')  
line8, = ax.plot([], [], [],lw=2, color = 'g')  
line9, = ax.plot([], [], [],lw=2, color = 'b')  

line10, = ax.plot([], [],[], lw=2, color = 'r')  
line11, = ax.plot([], [], [],lw=2, color = 'g')  
line12, = ax.plot([], [], [],lw=2, color = 'b')  
fk_text = ax.text(2.5, 0, 0, '')
LF_leg_text = ax.text(quad_kin.base_LF[0,3], quad_kin.base_LF[1,3], quad_kin.base_LF[2,3], 'LF leg')
LB_leg_text = ax.text(quad_kin.base_LB[0,3], quad_kin.base_LB[1,3], quad_kin.base_LB[2,3], 'LB leg')
RF_leg_text = ax.text(quad_kin.base_RF[0,3], quad_kin.base_RF[1,3], quad_kin.base_RF[2,3], 'RF leg')
RB_leg_text = ax.text(quad_kin.base_RB[0,3], quad_kin.base_RB[1,3], quad_kin.base_RB[2,3], 'RB leg')

# Setting the axes properties
ax.set_xlim3d([-0.5, 0.5])
ax.set_xlabel('X')

ax.set_ylim3d([-0.5, 0.5])
ax.set_ylabel('Y')

ax.set_zlim3d([-0.5, 0.5])
ax.set_zlabel('Z')

ax.set_title('Forward kinematics of Quadruped Leg')

# Creating the Animation object
line_ani = animation.FuncAnimation(fig, animate, int(N), fargs=(),
                                   interval=200, blit=False)
plt.rc('animation', html='jshtml')
line_ani
plt.show()

# # CHECK IK OF LF leg
# theta_1 = np.linspace(-np.pi/4, np.pi/4, 50)
# theta_2 = np.linspace(-np.pi/3, np.pi/3, 50)
# theta_3 = np.linspace(-np.pi/2, np.pi/2, 50)
# for th_1 in theta_1:
#   for th_2 in theta_2:
#     for th_3 in theta_3:
#       poses_fk = quad_kin.lf_leg_fk(base = quad_kin.base_LF, angles = [th_1, th_2, th_3])
#       # print('poses after fk',poses_fk[3])
#       th_1_ik, th_2_ik, th_3_ik = quad_kin.leg_ik(base = quad_kin.base_LF, pos = poses_fk[3])
#       # print('angles after ik', angles)

#       poses_ik = quad_kin.lf_leg_fk(base = quad_kin.base_LF, angles = [th_1_ik, th_2_ik, th_3_ik])
#       if abs(poses_ik[3][0] - poses_fk[3][0]) >= 10e-3 or abs(poses_ik[3][1] - poses_fk[3][1]) >= 10e-3 or abs(poses_ik[3][2] - poses_fk[3][2]) >= 10e-3:
#         print(":(")
#         print(poses_fk[3])p_m, p_h, p_c, p_f
t = sym.symbols("t")
poses_0 = quad_kin.lf_leg_fk(base = quad_kin.base_LF, angles = [0, 0, 0])
x_0, y_0, z_0 = poses_0[3]

x_des,x_dot_des, y_des, y_dot_des, z_des, z_dot_des = quad_kin.get_cart_trajectory_by_sym(x_0, y_0, z_0 + 0.1 + 0.1*sym.sin(w*t))

poses_motor_0 = np.zeros((N,3))
poses_hip_0 = np.zeros((N,3))
poses_calf_0 = np.zeros((N,3))
poses_foot_0 = np.zeros((N,3))
# start_time = time.time()
t = np.linspace(0,5, 50)
for i in range(50):
  x_des_,x_dot_des_, y_des_, y_dot_des_, z_des_, z_dot_des_ = x_des(t[i]),x_dot_des(t[i]), y_des(t[i]), y_dot_des(t[i]), z_des(t[i]), z_dot_des(t[i])
  # print(x_des, y_des, z_des)
  th_1_ik, th_2_ik, th_3_ik = quad_kin.leg_ik(base = quad_kin.base_LF, pos = [x_des_, y_des_, z_des_])

  p_m, p_h, p_c, p_f = quad_kin.lf_leg_fk(base = quad_kin.base_LF, angles = [th_1_ik, th_2_ik, th_3_ik])
  poses_motor_0[i] = p_m
  poses_hip_0[i] = p_h
  poses_calf_0[i] = p_c
  poses_foot_0[i] = p_f
  cart_vel = quad_kin.get_cartesian_velocities(jac = quad_kin.LF_leg_Jac_sym(th_1_ik, th_2_ik, th_3_ik), angle_velocities = [x_dot_des_, y_dot_des_, z_dot_des_])
  print(cart_vel)



def animate(i):
  
  line1.set_data(np.array([poses_motor_0[i][0], poses_hip_0[i][0]]), np.array([poses_motor_0[i][1], poses_hip_0[i][1]]))
  line1.set_3d_properties( np.array([poses_motor_0[i][2], poses_hip_0[i][2]]))
  line2.set_data(np.array([poses_hip_0[i][0], poses_calf_0[i][0]]), np.array([poses_hip_0[i][1], poses_calf_0[i][1]]))
  line2.set_3d_properties( np.array([poses_hip_0[i][2], poses_calf_0[i][2]]))
  line3.set_data(np.array([poses_calf_0[i][0], poses_foot_0[i][0]]), np.array([poses_calf_0[i][1], poses_foot_0[i][1]]))
  line3.set_3d_properties( np.array([poses_calf_0[i][2], poses_foot_0[i][2]]))

  # line4.set_data(np.array([poses_motor_1[i][0], poses_hip_1[i][0]]), np.array([poses_motor_1[i][1], poses_hip_1[i][1]]))
  # line4.set_3d_properties( np.array([poses_motor_1[i][2], poses_hip_1[i][2]]))
  # line5.set_data(np.array([poses_hip_1[i][0], poses_calf_1[i][0]]), np.array([poses_hip_1[i][1], poses_calf_1[i][1]]))
  # line5.set_3d_properties( np.array([poses_hip_1[i][2], poses_calf_1[i][2]]))
  # line6.set_data(np.array([poses_calf_1[i][0], poses_foot_1[i][0]]), np.array([poses_calf_1[i][1], poses_foot_1[i][1]]))
  # line6.set_3d_properties( np.array([poses_calf_1[i][2], poses_foot_1[i][2]]))

  # line7.set_data(np.array([poses_motor_2[i][0], poses_hip_2[i][0]]), np.array([poses_motor_2[i][1], poses_hip_2[i][1]]))
  # line7.set_3d_properties( np.array([poses_motor_2[i][2], poses_hip_2[i][2]]))
  # line8.set_data(np.array([poses_hip_2[i][0], poses_calf_2[i][0]]), np.array([poses_hip_2[i][1], poses_calf_2[i][1]]))
  # line8.set_3d_properties( np.array([poses_hip_2[i][2], poses_calf_2[i][2]]))
  # line9.set_data(np.array([poses_calf_2[i][0], poses_foot_2[i][0]]), np.array([poses_calf_2[i][1], poses_foot_2[i][1]]))
  # line9.set_3d_properties( np.array([poses_calf_2[i][2], poses_foot_2[i][2]]))

  # line10.set_data(np.array([poses_motor_3[i][0], poses_hip_3[i][0]]), np.array([poses_motor_3[i][1], poses_hip_3[i][1]]))
  # line10.set_3d_properties( np.array([poses_motor_3[i][2], poses_hip_3[i][2]]))
  # line11.set_data(np.array([poses_hip_3[i][0], poses_calf_3[i][0]]), np.array([poses_hip_3[i][1], poses_calf_3[i][1]]))
  # line11.set_3d_properties( np.array([poses_hip_3[i][2], poses_calf_3[i][2]]))
  # line12.set_data(np.array([poses_calf_3[i][0], poses_foot_3[i][0]]), np.array([poses_calf_3[i][1], poses_foot_3[i][1]]))
  # line12.set_3d_properties( np.array([poses_calf_3[i][2], poses_foot_3[i][2]]))


#   fk_text.set_text(f'Forward kinematics')

#   ax.view_init(60, i/2) # Change point of view
  return (line1,line2, line3,
          #  line4, line5, line6, 
          #  line7, line8, line9,
          #   line10, line11, line12
            )


# Attaching 3D axis to the figure
fig = plt.figure()
ax = p3.Axes3D(fig)
# plt.close()
line1, = ax.plot([], [],[], lw=2, color = 'r')  
line2, = ax.plot([], [], [],lw=2, color = 'g')  
line3, = ax.plot([], [], [],lw=2, color = 'b')  

# line4, = ax.plot([], [],[], lw=2, color = 'r')  
# line5, = ax.plot([], [], [],lw=2, color = 'g')  
# line6, = ax.plot([], [], [],lw=2, color = 'b')  

# line7, = ax.plot([], [],[], lw=2, color = 'r')  
# line8, = ax.plot([], [], [],lw=2, color = 'g')  
# line9, = ax.plot([], [], [],lw=2, color = 'b')  

# line10, = ax.plot([], [],[], lw=2, color = 'r')  
# line11, = ax.plot([], [], [],lw=2, color = 'g')  
# line12, = ax.plot([], [], [],lw=2, color = 'b')  
fk_text = ax.text(2.5, 0, 0, '')
LF_leg_text = ax.text(quad_kin.base_LF[0,3], quad_kin.base_LF[1,3], quad_kin.base_LF[2,3], 'LF leg')
LB_leg_text = ax.text(quad_kin.base_LB[0,3], quad_kin.base_LB[1,3], quad_kin.base_LB[2,3], 'LB leg')
RF_leg_text = ax.text(quad_kin.base_RF[0,3], quad_kin.base_RF[1,3], quad_kin.base_RF[2,3], 'RF leg')
RB_leg_text = ax.text(quad_kin.base_RB[0,3], quad_kin.base_RB[1,3], quad_kin.base_RB[2,3], 'RB leg')

# Setting the axes properties
ax.set_xlim3d([-0.5, 0.5])
ax.set_xlabel('X')

ax.set_ylim3d([-0.5, 0.5])
ax.set_ylabel('Y')

ax.set_zlim3d([-0.5, 0.5])
ax.set_zlabel('Z')

ax.set_title('Forward kinematics of Quadruped Leg')

# Creating the Animation object
line_ani = animation.FuncAnimation(fig, animate, int(N), fargs=(),
                                   interval=200, blit=False)
plt.rc('animation', html='jshtml')
line_ani
plt.show()


