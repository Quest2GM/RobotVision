from tkinter import *
from class_main import *
from func_main import *

###################
# Global Variables
###################

sim_time = 0    # Simulation time
speed = 25      # Vehicle speed (pixels)
draw_arr = []   # Path array - the set of points that define a path, drawn by the user

############################
# Tkinter Canvas Properties
############################

w, h = 1400, 900
root = Tk()
root.title('RobotVision')
root.geometry(str(w) + 'x' + str(h))
root.state('zoomed')
c = Canvas(root, width=w, height=h, bg='white')
c.pack(padx=2, pady=2)


# Creates grid with separation of 25 pixels
def create_grid(event=None):
    
    # Creates all vertical grid lines
    for i in range(0, w, speed):
        c.create_line([(i, 0), (i, h)], tag='grid_line', fill='#D9E2E5')

    # Creates all horizontal grid lines
    for i in range(0, h, speed):
        c.create_line([(0, i), (w, i)], tag='grid_line', fill='#D9E2E5')

    c.create_oval(w/2-3, h/2-3, w/2+3, h/2+3, fill='black')

# Path drawing functions
def get_x_and_y(event):
    global lasx, lasy, draw_arr
    lasx, lasy = event.x, event.y
    draw_arr += [[lasx, lasy]]

def draw_path(event):
    global lasx, lasy, draw_arr
    c.create_line((lasx, lasy, event.x, event.y), fill='black', width=5)
    lasx, lasy = event.x, event.y
    draw_arr += [[lasx, lasy]]


#######################
# PID Control Function
#######################

def PID_move():

    global sim_time, c

    # Update PID gains
    PID_ctrl.compute_dist_dir(draw_arr, bot.pos, bot.angle)
    E, e_rot = PID_ctrl.update_gains(speed)
    
    # Rotate the bot CW or CCW depending on cross-product result and then incrementally move forward
    if e_rot > 0:
        bot.rotateCCW(r=(1/25)/E)
    elif e_rot < 0:
        bot.rotateCW(r=(1/25)/E)
    bot.move()

    # Update simulation time on screen
    sim_time += 1000/speed
    l.config(text='Simulation Time (s): ' + str(float(sim_time/1000)))
    l2.config(text='Use Type: ' + str(e_rot))
    root.after(int(1000/speed), PID_move)

##################################
# Extended Kalman Filter Function
##################################

def EKF_move():

    global sim_time, c

    # Update PID gains
    PID_ctrl.compute_dist_dir(draw_arr, bot.pos, bot.angle)
    E, e_rot = PID_ctrl.update_gains(speed)

    # Rotate the bot CW or CCW depending on cross-product result and then incrementally move forward (with error)
    if e_rot > 0:
        bot.rotateCCW(r=(1/25)/E)
    elif e_rot < 0:
        bot.rotateCW(r=(1/25)/E)
    bot.move(noise=[0,qerr])

    ### Extended Kalman Filter Algorithm: ###
    
    # Get coordinates of lighthouse and current position
    x_CN, y_CN = pixel_2_grid(ekf.x_S, ekf.y_S)
    x_pos, y_pos = pixel_2_grid(*bot.pos)

    # +/- Factor to identify whether to rotate CW or CCW
    f = e_rot/np.abs(e_rot)

    # Simulate noisy measurement. Z1 = angle to lighthouse, Z2 = distance to lighthouse
    Z1 = np.arctan2(y_CN - y_pos, x_CN - x_pos) - bot.angle
    Z2 = np.sqrt((ekf.x_S - bot.pos[0])**2 + (ekf.y_S - bot.pos[1])**2)
    Z = np.array([Z1, Z2]).reshape(2,1) + np.random.multivariate_normal([0,0], cov=ekf.R).reshape(2,1)

    # Predict and update
    ekf.predict(E, f)
    x_pred = ekf.update(Z, Z2, detect_range)

    # Visually track estimation on screen
    c1, c2 = x_pred[0][0], x_pred[1][0]
    c.create_oval(c1-2, c2-2, c1+2, c2+2, fill='cyan')

    # Absolute error between prediction and actual
    x_pos, y_pos = pixel_2_grid(*bot.pos)
    calc_error = np.sqrt((x_pred[0][0]-bot.pos[0])**2 + (x_pred[1][0]-bot.pos[1])**2)

    # Update screen text
    sim_time += 1000/speed
    l.config(text='Simulation Time (s): ' + str(float(sim_time/1000)))
    l2.config(text='Error : ' + str(calc_error))
    root.after(int(1000/speed), EKF_move)
    

if __name__ == '__main__':

    ### Create Bot ###
    bot = Car(c)
    bot.build([float(w/2), float(h/2)])
    c.bind('<Configure>', create_grid)
    c.bind('<Button-1>', get_x_and_y)
    c.bind('<B1-Motion>', draw_path)

    
    ### Setup PID Control ###
    kp, ki, kd = 5, 0.00001, 0.5
    PID_ctrl = PID(kp=kp, ki=ki, kd=kd, f_dist=0.1)

    ### Setup Kalman Filter ###

    # Create lighthouse and the detection range circle
    r, detect_range = 10, 100
    x1, y1 = grid_2_pixel(10, 1)
    c.create_oval(x1-detect_range, y1-detect_range, x1+detect_range, y1+detect_range, fill='white')
    c.create_oval(x1-r, y1-r, x1+r, y1+r, fill='red')

    # Setup Kalman variables and filter class
    X_k = np.array([[bot.pos[0],bot.pos[1],bot.angle]]).T                       # Initial pose
    perr, qerr, rerr = 1e-5, 5e-3, 1e-3                                         # Kalman filter error terms
    P_k, Q_k, R_k = np.eye(3) * perr, np.eye(3) * qerr, np.eye(2) * rerr        # State, Process, Measurement Covariances
    u_k = np.array([[1,0]]).T                                                   # Input: (velocity, angular velocity)
    ekf = EKF(X_0=X_k, dt=0.04, u=u_k, Q=Q_k, R=R_k, P_0=P_k, x_S=x1, y_S=y1)   # Initializaion of extended kalman filter

    ### Canvas Buttons and Labels ###
    b = Button(root, text='Run', command=EKF_move, font=('Helvetica', 16), fg='black')
    b.place(x=58, y=10)
    l = Label(root, text='Simulation Time (s): 0', fg='black')
    l.place(x=58, y=60)
    l2 = Label(root, text='PID Parameters: ', fg='black')
    l2.place(x=58, y=110)

    root.mainloop()