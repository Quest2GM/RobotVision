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
        # bot.rotateCW(r=radius)
        sec_pass += 1000/speed
        inc += 1
        l.config(text='Simulation Time (s): ' + str(float(sec_pass/1000)))
        root.after(int(1000/speed), auto_move)

def PID_move():

    global path_arr, bot, sec_pass, PID_ctrl

    # if np.abs(error) < 200:

    # Bang-bang Control: Rotate only depending on sign of error
        # if e_rot > 0:
        #     bot.rotateCCW(r=1.5)
        # elif e_rot < 0:
        #     bot.rotateCW(r=1.5)
    print('___')

    x_pos, y_pos = pixel_2_grid(*bot.pos)
    print('X_old: ', x_pos, y_pos)
    print('Angle: ', bot.angle)

    # Update gains
    PID_ctrl.compute_dist_dir(path_arr, bot.pos, bot.angle)
    E, e_rot = PID_ctrl.update_gains(speed)

    print('W: ', E)
    
    # PID Control
    if e_rot > 0:
        bot.rotateCCW(r=(1/25)/E)
    elif e_rot < 0:
        bot.rotateCW(r=(1/25)/E)

    bot.move()

    x_pos, y_pos = pixel_2_grid(*bot.pos)
    print('X_old: ', x_pos, y_pos)
    print('Angle_new: ', bot.angle)
    
    # Update simulation time on screen
    sec_pass += 1000/speed
    l.config(text='Simulation Time (s): ' + str(float(sec_pass/1000)))
    # l2.config(text='PID Parameters: Error, E, I, D = ' + str(float(e_dist)) + ', ' + str(float(E)) + ', ' + str(float(I)) + ', ' + str(float(D)))
    root.after(int(1000/speed), PID_move)



def Kalman_move():

    global path_arr, bot, sec_pass, PID_ctrl, perr, rerr, kf, x0, y0, c

    if True:
        
        # Update gains
        PID_ctrl.compute_dist_dir(path_arr, bot.pos, bot.angle)
        E, e_rot = PID_ctrl.update_gains(speed)

        # PID Control
        if e_rot > 0:
            bot.rotateCCW(r=(1/25)/E)
        elif e_rot < 0:
            bot.rotateCW(r=(1/25)/E)

        # Kalman filtering
        x_pos, y_pos = pixel_2_grid(*bot.pos)
        bot.move(noise=[0,qerr])

        Z1 = np.arctan2(kf.y_S - y_pos, kf.x_S - x_pos) - bot.angle
        Z2 = np.sqrt((kf.x_S - x_pos)**2 + (kf.y_S - y_pos)**2)
        Z = np.array([Z1, Z2]).reshape(2,1) + np.random.multivariate_normal([0,0], cov=kf.R).reshape(2,1)
        # print('Meas: ' + str(rad_2_deg(Z)))

        print('Actual X: ', np.array([x_pos, y_pos, bot.angle]).reshape(3,1))
        print('Actual Z: ', np.array([Z1,Z2]).reshape(2,1))
        print('______________')

        kf.predict(E)
        x_pred = kf.update(Z)

        c1, c2 = grid_2_pixel(x_pred[0][0],x_pred[1][0])
        c.create_oval(c1-2, c2-2, c1+2, c2+2, fill='cyan')

        x_pos, y_pos = pixel_2_grid(*bot.pos)

        calc_error = np.sqrt((x_pred[0][0]-x_pos)**2 + (x_pred[1][0]-y_pos)**2)

        # print('X_true: ' + str(x_pos) + ', ' + str(y_pos) + ', ' + str(bot.angle))
        # print('X_pred: ' + str(x_pred[0][0]) + ', ' + str(x_pred[1][0]) + ', ' + str(x_pred[2][0]))

        sec_pass += 1000/speed
        # l.config(text='Simulation Time (s): ' + str(float(sec_pass/1000)))
        l.config(text='Z1: ' + str(rad_2_deg(Z1)))
        # l2.config(text='Predicted State / Actual State : ' + str(X_pred) + ' //// ' + str(bot.pos) + ', ' + str(bot.angle))
        l2.config(text='Error : ' + str(calc_error))
        root.after(int(1000/speed), Kalman_move)

def Kalman_move_2():

    global path_arr, bot, sec_pass, PID_ctrl, qerr, rerr, kf, x0, y0, inc

    if inc < 2:

        print('\n###################')
        print('# Increment ' + str(inc))
        print('###################\n')

        print('P: ', kf.P)
        print('Q: ', kf.Q)
        print('R: ', kf.R)
        print('X: ', kf.X)
        A, B, D = kf.update_ABD(kf.X[0][0], kf.X[1][0], kf.X[2][0])
        print('A: ', A)
        print('B: ', B)
        print('D: ', D)
        print('x_CN: ', kf.x_S)
        print('y_CN: ', kf.y_S)
        x_pos, y_pos = pixel_2_grid(*bot.pos)
        print('bot pos: ', [x_pos, y_pos])

        bot.move(noise=[0,0])

        Z = np.arctan2(kf.y_S - y_pos, kf.x_S - x_pos)
        print('Z sens: ', rad_2_deg(Z))
        kf.predict(0)

        print('W: ', kf.W)
        print('P k+1|k: ', kf.P)
        print('S k+1: ', kf.S)

        kf.update(Z)
        
        root.after(int(1000/speed), Kalman_move)
        inc += 1

    

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

# Add shapes for Kalman Filtering
r = 10
x0, y0 = 5, 5
x1, y1 = grid_2_pixel(x0,y0)
c.create_oval(x1-r, y1-r, x1+r, y1+r, fill='red')

# Setup Kalman variables and filter class
X_k = np.array([[0,0,bot.angle]]).T
perr, qerr, rerr = 1e-5, 5e-3, 1e-3
P_k, Q_k, R_k = np.eye(3) * perr, np.eye(3) * qerr, np.eye(2) * rerr
u_k = np.array([[1,0]]).T
kf = Kalman(X_0=X_k, dt=0.04, u=u_k, Q=Q_k, R=R_k, P_0=P_k, x_S=x0, y_S=y0)

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