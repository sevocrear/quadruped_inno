import tkinter as tk
 
window = tk.Tk()
window.title('XYZ RPY CONTROL')
window.geometry('500x300') 
 
l = tk.Label(window, bg='white', fg='black', width=20, text='empty')
l.pack()
 
l1 = tk.Label(window, bg='white', fg='black', width=20, text='empty')
l1.pack()

def print_selection_X(v):
    l.config(text='X =' + v)
def print_selection_Y(v):
    l1.config(text='Y =' + v)

s = tk.Scale(window, label='try me', from_=0, to=10, orient=tk.HORIZONTAL, length=200, showvalue=0,tickinterval=2, resolution=0.01, command=print_selection_X)
s.pack()
 
s1 = tk.Scale(window, label='try me', from_=0, to=10, orient=tk.HORIZONTAL, length=200, showvalue=0,tickinterval=2, resolution=0.01, command=print_selection_Y)
s1.pack()
window.mainloop()