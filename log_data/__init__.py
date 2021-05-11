import pandas as pd
import time
class save_data():
    def __init__(self,):
        # create pandas dataframe to collect data
        self.df_data = pd.DataFrame(columns=["LF_leg_hip_q_cur", "LF_leg_thigh_q_cur", "LF_leg_knee_q_cur", "LF_leg_hip_q_dot_cur", "LF_leg_thigh_q_dot_cur", "LF_leg_knee_q_dot_cur",
                                        "RF_leg_hip_q_cur", "RF_leg_thigh_q_cur", "RF_leg_knee_q_cur", "RF_leg_hip_q_dot_cur", "RF_leg_thigh_q_dot_cur", "RF_leg_knee_q_dot_cur",
                                        "LB_leg_hip_q_cur", "LB_leg_thigh_q_cur", "LB_leg_knee_q_cur", "LB_leg_hip_q_dot_cur", "LB_leg_thigh_q_dot_cur", "LB_leg_knee_q_dot_cur",
                                        "RB_leg_hip_q_cur", "RB_leg_thigh_q_cur", "RB_leg_knee_q_cur", "RB_leg_hip_q_dot_cur", "RB_leg_thigh_q_dot_cur", "RB_leg_knee_q_dot_cur",


                                        "LF_leg_hip_q_des", "LF_leg_thigh_q_des", "LF_leg_knee_q_des", "LF_leg_hip_q_dot_des", "LF_leg_thigh_q_dot_des", "LF_leg_knee_q_dot_des",
                                        "RF_leg_hip_q_des", "RF_leg_thigh_q_des", "RF_leg_knee_q_des", "RF_leg_hip_q_dot_des", "RF_leg_thigh_q_dot_des", "RF_leg_knee_q_dot_des",
                                        "LB_leg_hip_q_des", "LB_leg_thigh_q_des", "LB_leg_knee_q_des", "LB_leg_hip_q_dot_des", "LB_leg_thigh_q_dot_des", "LB_leg_knee_q_dot_des",
                                        "RB_leg_hip_q_des", "RB_leg_thigh_q_des", "RB_leg_knee_q_des", "RB_leg_hip_q_dot_des", "RB_leg_thigh_q_dot_des", "RB_leg_knee_q_dot_des",



                                        "x_cur", "y_cur", "z_cur", "R_cur", "P_cur", "Y_cur",
                                        "x_des", "y_des", "z_des", "R_des", "P_des", "Y_des",
                                        "time"

                                        ])
    def update(self, q_cur, q_dot_cur, q_des, q_dot_des, t, motors):
        # Update dataframe      
        try:
                df_len = len(self.df_data)
                for motor in motors:
                        self.df_data.at[df_len,
                                f'{motor}_hip_q_cur'] = q_cur[motor][0][0]
                        self.df_data.at[df_len,
                                f'{motor}_thigh_q_cur'] = q_cur[motor][1][0]
                        self.df_data.at[df_len,
                                f'{motor}_knee_q_cur'] = q_cur[motor][2][0]
                        self.df_data.at[df_len,
                                f'{motor}_hip_q_dot_cur'] = q_dot_cur[motor][0][0]
                        self.df_data.at[df_len,
                                f'{motor}_thigh_q_dot_cur'] = q_dot_cur[motor][1][0]
                        self.df_data.at[df_len,
                                f'{motor}_knee_q_dot_cur'] = q_dot_cur[motor][2][0]

                        self.df_data.at[df_len,
                                f'{motor}_hip_q_des'] = q_des[motor][0][0]
                        self.df_data.at[df_len,
                                f'{motor}_thigh_q_des'] = q_des[motor][1][0]
                        self.df_data.at[df_len,
                                f'{motor}_knee_q_des'] = q_des[motor][2][0]
                        self.df_data.at[df_len,
                                f'{motor}_hip_q_dot_des'] = q_dot_des[motor][0][0]
                        self.df_data.at[df_len,
                                f'{motor}_thigh_q_dot_des'] = q_dot_des[motor][1][0]
                        self.df_data.at[df_len,
                                f'{motor}_knee_q_dot_des'] = q_dot_des[motor][2][0]
                self.df_data.at[df_len, 'x_cur'] = 0
                self.df_data.at[df_len, 'y_cur'] = 0
                self.df_data.at[df_len, 'z_cur'] = 0

                self.df_data.at[df_len, 'x_des'] = 0
                self.df_data.at[df_len, 'y_des'] = 0
                self.df_data.at[df_len, 'z_des'] = 0

                self.df_data.at[df_len, 'R_cur'] = 0
                self.df_data.at[df_len, 'P_cur'] = 0
                self.df_data.at[df_len, 'Y_cur'] = 0

                self.df_data.at[df_len, 'R_des'] = 0
                self.df_data.at[df_len, 'P_des'] = 0
                self.df_data.at[df_len, 'Y_des'] = 0
                self.df_data.at[df_len, 'time'] = t.value
        except IndexError:
                pass
        except KeyError:
                pass
    def save_data_to_csv(self,):
        self.df_data.to_csv(time.strftime(
                "%H_%M_%S_%d_%m_%Y.csv", time.gmtime()))
    