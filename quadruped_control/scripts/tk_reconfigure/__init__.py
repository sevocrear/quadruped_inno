
import tkinter as tk
def tk_reconfigure_xyz_rpy(shared_variables):
    window = tk.Tk()
    window.title('XYZ RPY CONTROL')
    window.geometry('1920x1080') 
    
    l = tk.Label(window, bg='white', fg='black', width=20, text='')
    l.pack()
    
    l1 = tk.Label(window, bg='white', fg='black', width=20, text='')
    l1.pack()

    l2 = tk.Label(window, bg='white', fg='black', width=20, text='')
    l2.pack()

    l3 = tk.Label(window, bg='white', fg='black', width=20, text='')
    l3.pack()

    l4 = tk.Label(window, bg='white', fg='black', width=20, text='')
    l4.pack()

    l5 = tk.Label(window, bg='white', fg='black', width=20, text='')
    l5.pack()

    l6 = tk.Label(window, bg='white', fg='black', width=20, text='')
    l6.pack()

    l7 = tk.Label(window, bg='yellow', fg='black', width=20, text='')
    l7.pack()

    l8 = tk.Label(window, bg='yellow', fg='black', width=20, text='')
    l8.pack()

    l9 = tk.Label(window, bg='yellow', fg='black', width=20, text='')
    l9.pack()

    l10 = tk.Label(window, bg='yellow', fg='black', width=20, text='')
    l10.pack()

    l11 = tk.Label(window, bg='yellow', fg='black', width=20, text='')
    l11.pack()

    def print_selection_Kp_abad(v):
        l.config(text='Kp abad =' + v)
        shared_variables[0] = float(v)
    
    def print_selection_Kp_thigh(v):
        l2.config(text='Kp thigh =' + v)
        shared_variables[1] = float(v)
    
    def print_selection_Kp_knee(v):
        l4.config(text='Kp knee =' + v)
        shared_variables[2] = float(v)

    def print_selection_Kd_abad(v):
        l1.config(text='Kd abad =' + v)
        shared_variables[3] = float(v)
    
    def print_selection_Kd_thigh(v):
        l3.config(text='Kd thigh =' + v)
        shared_variables[4] = float(v)
    
    def print_selection_Kd_knee(v):
        l5.config(text='Kd knee =' + v)
        shared_variables[5] = float(v)

    def print_selection_X(v):
        l6.config(text='X =' + v)
        shared_variables[6] = float(v)
    
    def print_selection_Y(v):
        l7.config(text='Y =' + v)
        shared_variables[7] = float(v)
    
    def print_selection_Z(v):
        l8.config(text='Z =' + v)
        shared_variables[8] = float(v)

    def print_selection_roll(v):
        l9.config(text='roll =' + v)
        shared_variables[9] = float(v)

    def print_selection_pitch(v):
        l10.config(text='pitch =' + v)
        shared_variables[10] = float(v)
    
    def print_selection_yaw(v):
        l11.config(text='yaw =' + v)
        shared_variables[11] = float(v)

    s = tk.Scale(window, label='Kp abad', from_= 1, to=500, orient=tk.HORIZONTAL, length=500, showvalue=0,tickinterval=100, resolution=0.01, command=print_selection_Kp_abad)
    s.pack()
    s1 = tk.Scale(window, label='Kd abad', from_=0, to=50, orient=tk.HORIZONTAL, length=500, showvalue=0,tickinterval=25, resolution=0.01, command=print_selection_Kd_abad)
    s1.pack()

    s2 = tk.Scale(window, label='Kp tjigh', from_= 1, to=500, orient=tk.HORIZONTAL, length=500, showvalue=0,tickinterval=100, resolution=0.01, command=print_selection_Kp_thigh)
    s2.pack()
    s3 = tk.Scale(window, label='Kd thigh', from_=0, to=50, orient=tk.HORIZONTAL, length=500, showvalue=0,tickinterval=25, resolution=0.01, command=print_selection_Kd_thigh)
    s3.pack()

    s4 = tk.Scale(window, label='Kp knee', from_= 1, to=500, orient=tk.HORIZONTAL, length=500, showvalue=0,tickinterval=100, resolution=0.01, command=print_selection_Kp_knee)
    s4.pack()
    s5 = tk.Scale(window, label='Kd knee', from_=0, to=50, orient=tk.HORIZONTAL, length=500, showvalue=0,tickinterval=25, resolution=0.01, command=print_selection_Kd_knee)
    s5.pack()

    s6 = tk.Scale(window, label='X', from_=0, to=0, orient=tk.HORIZONTAL, length=500, showvalue=0,tickinterval=25, resolution=0.01, command=print_selection_X)
    s6.pack()

    s7 = tk.Scale(window, label='Y', from_=0, to=0, orient=tk.HORIZONTAL, length=500, showvalue=0,tickinterval=25, resolution=0.01, command=print_selection_Y)
    s7.pack()

    var_Z = tk.DoubleVar()
    var_Z.set(0.425)
    s8 = tk.Scale(window, label='Z', from_=0.1, to=0.425, orient=tk.HORIZONTAL, length=300, showvalue=0,tickinterval=0.1, resolution=0.001, command=print_selection_Z, variable = var_Z)
    s8.pack()

    var_R = tk.DoubleVar()
    var_R.set(0.0)
    s9 = tk.Scale(window, label='R', from_=-0.3, to=0.3, orient=tk.HORIZONTAL, length=300, showvalue=0,tickinterval=0.1, resolution=0.001, command=print_selection_roll, variable = var_R)
    s9.pack()

    var_P = tk.DoubleVar()
    var_P.set(0.0)
    s10 = tk.Scale(window, label='P', from_=-0.3, to=0.3, orient=tk.HORIZONTAL, length=300, showvalue=0,tickinterval=0.1, resolution=0.001, command=print_selection_pitch, variable = var_P)
    s10.pack()

    var_Y = tk.DoubleVar()
    var_Y.set(0.0)
    s11 = tk.Scale(window, label='Y', from_=-0.3, to=0.3, orient=tk.HORIZONTAL, length=300, showvalue=0,tickinterval=0.1, resolution=0.001, command=print_selection_yaw, variable = var_Y)
    s11.pack()
    window.mainloop()