from tkinter import *
from class_main import *
from func_main import *

# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation

# Global Vars
sec_pass, inc, radius = 0, 0, 1
speed = 25
path_arr = []
error = 0

# PID Parameters
I = 0
D = 0
LE = 0

# Define Canvas and its Properties
w, h = 1400, 900
root = Tk()
root.title('RobotVision')
root.geometry(str(w) + 'x' + str(h))
root.state('zoomed')
c = Canvas(root, width=w, height=h, bg='white')
c.pack(padx=2, pady=2)


def create_grid(event=None):

    # Creates all vertical grid lines
    for i in range(0, w, 25):
        c.create_line([(i, 0), (i, h)], tag='grid_line', fill='#D9E2E5')

    # Creates all horizontal grid lines
    for i in range(0, h, 25):
        c.create_line([(0, i), (w, i)], tag='grid_line', fill='#D9E2E5')

    c.create_oval(w/2-3, h/2-3, w/2+3, h/2+3, fill='black')

def auto_move():
    
    global sec_pass, inc

    if inc < speed * 100:
        bot.move()
        bot.rotateCW(r=radius)
        sec_pass += 1000/speed
        inc += 1
        l.config(text='Simulation Time (s): ' + str(float(sec_pass/1000)))
        root.after(int(1000/speed), auto_move)

def PID_move():

    global path_arr, bot, sec_pass, PID_ctrl

    if np.abs(error) < 200:

        # Bang-bang Control: Rotate only depending on sign of error
            # if e_rot > 0:
            #     bot.rotateCCW(r=1.5)
            # elif e_rot < 0:
            #     bot.rotateCW(r=1.5)

        # Update gains
        PID_ctrl.compute_dist_dir(path_arr, bot.pos, bot.angle)
        E, e_rot = PID_ctrl.update_gains(speed)
        
        # PID Control
        if e_rot > 0:
            bot.rotateCCW(r=(1/25)/E)
        elif e_rot < 0:
            bot.rotateCW(r=(1/25)/E)

        bot.move()
        
        # Update simulation time on screen
        sec_pass += 1000/speed
        l.config(text='Simulation Time (s): ' + str(float(sec_pass/1000)))
        # l2.config(text='PID Parameters: Error, E, I, D = ' + str(float(e_dist)) + ', ' + str(float(E)) + ', ' + str(float(I)) + ', ' + str(float(D)))
        root.after(int(1000/speed), PID_move)



def Kalman_move():



    if True:

        root.after(int(1000/speed), PID_move)


def get_x_and_y(event):
    global lasx, lasy, path_arr
    lasx, lasy = event.x, event.y
    path_arr += [[lasx, lasy]]

def draw_path(event):
    global lasx, lasy, path_arr
    c.create_line((lasx, lasy, event.x, event.y), fill='black', width=5)
    lasx, lasy = event.x, event.y
    path_arr += [[lasx, lasy]]

# Create TurtleBot
bot = Car(c)
bot.build([float(w/2), float(h/2)])
c.bind('<Configure>', create_grid)
c.bind('<Button-1>', get_x_and_y)
c.bind('<B1-Motion>', draw_path)

# Setup PID
PID_ctrl = PID(kp=5, ki=0.00001, kd=0.5, f_dist=0.1)

# Add buttons and labels
b = Button(root, text='Run', command=PID_move, font=('Helvetica', 16), fg='black')
b.place(x=58, y=10)
l = Label(root, text='Simulation Time (s): 0', fg='black')
l.place(x=58, y=60)
l2 = Label(root, text='PID Parameters: ', fg='black')
l2.place(x=58, y=110)

# d = Dubin(c)
# d.create_paths()

root.mainloop()