import pandas as pd
import matplotlib.pyplot as plt
import os
import sys
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
# https://www.codespeedy.com/visualize-data-from-csv-file-in-python/
csv_file = "21_34_47_19_04_2021.csv"
df_data = pd.read_csv(csv_file)

# hip, thigh, knee
LF_hip_angles_cur = df_data['LF_leg_thigh_q_cur']
LF_hip_angles_des = df_data['LF_leg_thigh_q_des']
time = df_data['time']
LF_hip_angles_error = LF_hip_angles_des.subtract(LF_hip_angles_cur, fill_value = 0)

plt.plot(time, LF_hip_angles_cur, label = 'q_cur')
plt.plot(time, LF_hip_angles_des, label = 'q_des')
plt.plot(time, LF_hip_angles_error, label = 'error')
plt.xlabel('time, sec')
plt.ylabel('LF_hip_q')
plt.grid()
plt.title('Angles Plot')
plt.legend()
plt.show()