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
glob_count = 0

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

    global path_arr, bot, sec_pass, PID_ctrl, glob_count

    # if np.abs(error) < 200:

    # Bang-bang Control: Rotate only depending on sign of error
        # if e_rot > 0:
        #     bot.rotateCCW(r=1.5)
        # elif e_rot < 0:
        #     bot.rotateCW(r=1.5)
    print('___')

    x_pos, y_pos = bot.pos[0], bot.pos[1]
    print('X_old: ', x_pos, y_pos)
    print('Angle: ', bot.angle)

    # Update gains
    PID_ctrl.compute_dist_dir(path_arr, bot.pos, bot.angle)
    E, e_rot = PID_ctrl.update_gains(speed)

    print('W: ', E)

    x_pred = np.round(x_pos + np.cos(bot.angle + E*0.04),6)
    x_pred_2 = np.round(x_pos + np.cos(bot.angle - E*0.04),6)
    y_pred = np.round(y_pos - np.sin(bot.angle + E*0.04),6)
    y_pred_2 = np.round(y_pos - np.sin(bot.angle - E*0.04),6)
    a_pred = np.round(bot.angle + E*0.04,6)
    a_pred_2 = np.round(bot.angle - E*0.04,6)
    print('X_pred_new: ', x_pred, y_pred, x_pred_2, y_pred_2)
    print('Angle_pred_new: ', a_pred, a_pred_2)
    
    # PID Control
    if e_rot > 0:
        bot.rotateCCW(r=(1/25)/E)
    elif e_rot < 0:
        bot.rotateCW(r=(1/25)/E)

    bot.move()

    x_pos, y_pos = np.round(bot.pos[0],6), np.round(bot.pos[1],6)
    print('X_new: ', x_pos, y_pos)
    print('Angle_new: ', bot.angle)

    if (x_pred != x_pos and x_pred_2 != x_pos) or (y_pred != y_pos and y_pred_2 != y_pos) or (a_pred != np.round(bot.angle,6) and a_pred_2 != np.round(bot.angle,6)):
        raise Exception
    
    if x_pred == x_pos and y_pred == y_pos and a_pred == np.round(bot.angle,6):
        use_type = 1
    elif x_pred_2 == x_pos and y_pred_2 == y_pos and a_pred_2 == np.round(bot.angle,6):
        use_type = 2
    else:
        use_type = 4444444
    
    # Update simulation time on screen
    sec_pass += 1000/speed
    l.config(text='Simulation Time (s): ' + str(float(sec_pass/1000)))
    l2.config(text='Use Type: ' + str(use_type) + '   ' + str(E))
    # root.after(int(1000/speed), PID_move)



def Kalman_move():

    global path_arr, bot, sec_pass, PID_ctrl, perr, rerr, kf, x0, y0, c

    if True:
        
        x_pos, y_pos = pixel_2_grid(*bot.pos)
        # Update gains
        PID_ctrl.compute_dist_dir(path_arr, bot.pos, bot.angle)
        E, e_rot = PID_ctrl.update_gains(speed)

        print(E, bot.angle, bot.pos[0])

        x_pred = np.round(bot.pos[0] + np.cos(bot.angle + E * 0.04),6)
        x_pred_2 = np.round(bot.pos[0] + np.cos(bot.angle - E * 0.04),6)
    

        # PID Control
        if e_rot > 0:
            bot.rotateCCW(r=(1/25)/E)
        elif e_rot < 0:
            bot.rotateCW(r=(1/25)/E)

        bot.move(noise=[0,qerr])

        # Kalman filtering
        x_CN, y_CN = pixel_2_grid(kf.x_S, kf.y_S)
        x_pos, y_pos = pixel_2_grid(*bot.pos)

        print(x_pred, x_pred_2, bot.MX[0])

        if x_pred == np.round(bot.MX[0],6):
            f = 1
        elif x_pred_2 == np.round(bot.MX[0],6):
            f = -1
        else:
            raise Exception

        Z1 = np.arctan2(y_CN - y_pos, x_CN - x_pos) - bot.angle
        Z2 = np.sqrt((kf.x_S - bot.pos[0])**2 + (kf.y_S - bot.pos[1])**2)
        Z = np.array([Z1, Z2]).reshape(2,1) + np.random.multivariate_normal([0,0], cov=kf.R).reshape(2,1)

        print('Actual X: ', np.array([bot.pos[0], bot.pos[1], bot.angle]).reshape(3,1))
        print('Actual Z: ', np.array([Z1,Z2]).reshape(2,1))
        print('f: ', f)
        print('______________')

        kf.predict(E, f)
        x_pred = kf.update(Z, Z2)

        c1, c2 = x_pred[0][0], x_pred[1][0]
        c.create_oval(c1-2, c2-2, c1+2, c2+2, fill='cyan')

        x_pos, y_pos = pixel_2_grid(*bot.pos)

        calc_error = np.sqrt((x_pred[0][0]-bot.pos[0])**2 + (x_pred[1][0]-bot.pos[1])**2)

        # print('X_true: ' + str(x_pos) + ', ' + str(y_pos) + ', ' + str(bot.angle))
        # print('X_pred: ' + str(x_pred[0][0]) + ', ' + str(x_pred[1][0]) + ', ' + str(x_pred[2][0]))

        sec_pass += 1000/speed
        # l.config(text='Simulation Time (s): ' + str(float(sec_pass/1000)))
        l.config(text='Z1: ' + str(rad_2_deg(Z1)))
        # l2.config(text='Predicted State / Actual State : ' + str(X_pred) + ' //// ' + str(bot.pos) + ', ' + str(bot.angle))
        l2.config(text='Error : ' + str(calc_error))
        root.after(int(1000/speed), Kalman_move)
    

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
x1, y1 = grid_2_pixel(10,1)
c.create_oval(x1-100, y1-100, x1+100, y1+100, fill='white')
c.create_oval(x1-r, y1-r, x1+r, y1+r, fill='red')

# Setup Kalman variables and filter class
X_k = np.array([[bot.pos[0],bot.pos[1],bot.angle]]).T
perr, qerr, rerr = 1e-5, 5e-3, 1e-3
P_k, Q_k, R_k = np.eye(3) * perr, np.eye(3) * qerr, np.eye(2) * rerr
u_k = np.array([[1,0]]).T
kf = Kalman(X_0=X_k, dt=0.04, u=u_k, Q=Q_k, R=R_k, P_0=P_k, x_S=x1, y_S=y1)

# Add buttons and labels
b = Button(root, text='Run', command=Kalman_move, font=('Helvetica', 16), fg='black')
b.place(x=58, y=10)
l = Label(root, text='Simulation Time (s): 0', fg='black')
l.place(x=58, y=60)
l2 = Label(root, text='PID Parameters: ', fg='black')
l2.place(x=58, y=110)

# d = Dubin(c)
# d.create_paths()

root.mainloop()
print(glob_count)