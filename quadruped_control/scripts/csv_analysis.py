import pandas as pd
import matplotlib.pyplot as plt
import os
import sys
abspath = os.path.abspath(__file__)
dname = os.path.dirname(abspath)
os.chdir(dname)
# https://www.codespeedy.com/visualize-data-from-csv-file-in-python/

name = "A_0_075_w_3_14"
csv_file = f"{name}.csv"
df_data = pd.read_csv(csv_file)

# thigh, thigh, thigh
data_to_show = ["U",""]

RB_thigh_angles_cur = df_data[f'RB_leg_thigh_{data_to_show[0]}_cur{data_to_show[1]}']
RB_thigh_angles_des = df_data[f'RB_leg_thigh_{data_to_show[0]}_des{data_to_show[1]}']
time = df_data['time']
RB_thigh_angles_error = RB_thigh_angles_des.subtract(RB_thigh_angles_cur, fill_value = 0)

RB_knee_angles_cur = df_data[f'RB_leg_knee_{data_to_show[0]}_cur{data_to_show[1]}']
RB_knee_angles_des = df_data[f'RB_leg_knee_{data_to_show[0]}_des{data_to_show[1]}']
RB_knee_angles_error = RB_knee_angles_des.subtract(RB_knee_angles_cur, fill_value = 0)

fig, ax = plt.subplots(2, 1)

ax[0].plot(time, RB_thigh_angles_cur, label = f'{data_to_show[0]}_cur{data_to_show[1]}')
ax[0].plot(time, RB_thigh_angles_des, label = f'{data_to_show[0]}_des{data_to_show[1]}')
ax[0].plot(time, RB_thigh_angles_error, label = 'error')
ax[0].set_ylabel('thigh angles, rad')
ax[0].grid()
ax[0].legend()

ax[1].plot(time, RB_knee_angles_cur, label = f'{data_to_show[0]}_cur{data_to_show[1]}')
ax[1].plot(time, RB_knee_angles_des, label = f'{data_to_show[0]}_des{data_to_show[1]}')
ax[1].plot(time, RB_knee_angles_error, label = 'error')
ax[1].set_xlabel('time, sec')
ax[1].set_ylabel('knee angles, rad')
ax[1].grid()
ax[1].legend()



plt.show()
fig.savefig(f"plots/{data_to_show[0]}_{data_to_show[1]}_{name}.svg", format="svg", dpi=300)