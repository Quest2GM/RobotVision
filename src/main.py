import random
import numpy as np
from tkinter import *
from PIL import Image, ImageTk
from scipy.spatial.distance import cdist

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
    E, e_rot = PID_ctrl.update_gains_PID(speed)
    
    # Rotate the bot CW or CCW depending on cross-product result and then incrementally move forward
    if e_rot > 0:
        bot.rotateCCW(r=(1/25)/E)
    elif e_rot < 0:
        bot.rotateCW(r=(1/25)/E)
    bot.move()

    # Update simulation time on screen
    sim_time += 1000/speed
    l.config(text='Simulation Time (s): ' + str(float(sim_time/1000)))
    # l2.config(text='PID Parameters: kp=' + str(kp) + str(', ki=') + str(ki) + str(', kd=') + str(kd))
    root.after(int(1000/speed), PID_move)



##################################
# Extended Kalman Filter Function
##################################

def EKF_move():

    global sim_time, c

    # Update PID gains
    A = range_2_pi(bot.angle)
    PID_ctrl.compute_dist_dir(draw_arr, bot.pos, A)
    E, e_rot = PID_ctrl.update_gains_PID(speed)

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
    Z11, Z12 = range_2_pi(np.arctan2(y_CN - y_pos, x_CN - x_pos)), range_2_pi(bot.angle)
    Z1 = Z11 - Z12
    Z2 = np.sqrt((ekf.x_S - bot.pos[0])**2 + (ekf.y_S - bot.pos[1])**2)
    Z = np.array([Z1, Z2]).reshape(2,1) + np.random.multivariate_normal([0,0], cov=ekf.R).reshape(2,1)

    # Predict and update
    ekf.predict(E, f)
    x_pred = ekf.update(Z, Z2, detect_range)

    # Visually track estimation on screen
    c1, c2 = x_pred[0][0], x_pred[1][0]
    c.create_oval(c1-2, c2-2, c1+2, c2+2, fill='cyan', outline='')

    # Absolute error between prediction and actual
    x_pos, y_pos = pixel_2_grid(*bot.pos)
    calc_error = np.sqrt((x_pred[0][0]-bot.pos[0])**2 + (x_pred[1][0]-bot.pos[1])**2)

    # Update screen text
    sim_time += 1000/speed
    l.config(text='Simulation Time (s): ' + str(float(sim_time/1000)))
    l2.config(text='Error : ' + str(calc_error))
    root.after(int(1000/speed), EKF_move)


##################################
# EKF Based Simul. Loc and Mapping
##################################

def SLAM_move():

    global sim_time, c

    # Update PID gains
    A = range_2_pi(bot.angle)
    PID_ctrl.compute_dist_dir(draw_arr, bot.pos, A)
    E, e_rot = PID_ctrl.update_gains_PID(speed)

    # Rotate the bot CW or CCW depending on cross-product result and then incrementally move forward (with error)
    if e_rot > 0:
        bot.rotateCCW(r=(1/25)/E)
    elif e_rot < 0:
        bot.rotateCW(r=(1/25)/E)
    bot.move(noise=[0,qerr])

    # +/- Factor to identify whether to rotate CW or CCW
    f = e_rot/np.abs(e_rot)

    ### SLAM Algorithm: ###

    # Determine which lighthouses are within detectable range of the bot
    dists = cdist([bot.pos], obs_arr, 'euclidean')[0]
    dists_cond = np.where(dists < detect_range)[0]

    # SLAM Predict
    slam.predict(E, f)
    
    # Iterate through detectable lighthouses
    for ind in dists_cond:

        # Get coordinates of lighthouse and current position
        x_CN, y_CN = pixel_2_grid(*obs_arr[ind])
        x_pos, y_pos = pixel_2_grid(*bot.pos)

        # Simulate noisy measurement. Z1 = angle to lighthouse, Z2 = distance to lighthouse
        Z11, Z12 = range_2_pi(np.arctan2(y_CN - y_pos, x_CN - x_pos)), range_2_pi(bot.angle)
        Z1 = Z11 - Z12
        Z2 = np.sqrt((obs_arr[ind][0] - bot.pos[0])**2 + (obs_arr[ind][1] - bot.pos[1])**2)
        Z = np.array([Z1, Z2]).reshape(2,1) + np.random.multivariate_normal([0,0], cov=slam.R).reshape(2,1)
        
        # Initialize the X_obj location if it has not been done already
        j1, j2 = 2*ind + 3, 2*ind + 4

        # Predict object location
        # if slam.X[j1][0] == 0 and slam.X[j2][0] == 0:
        slam.X[j1][0] = slam.X[0][0] + Z[1][0] * np.cos(Z[0][0] + Z12)
        slam.X[j2][0] = slam.X[1][0] - Z[1][0] * np.sin(Z[0][0] + Z12)

        # SLAM Update
        x_pred = slam.update(Z, obs_arr[ind], ind)

        # Draw predicted location of lighthouse_j on screen
        c.create_oval(x_pred[j1][0]-2, x_pred[j2][0]-2, x_pred[j1][0]+2, x_pred[j2][0]+2, fill='blue', outline='')

    # Do not update parameters if there is no lighthouse in range
    if len(dists_cond) == 0:
        x_pred = np.array([slam.X[0][0], slam.X[1][0]]).reshape(2,1)

    # Visually track estimation on screen
    c1, c2 = x_pred[0][0], x_pred[1][0]
    c.create_oval(c1-2, c2-2, c1+2, c2+2, fill='cyan', outline='')
    calc_error = np.sqrt((c1-bot.pos[0])**2 + (c2-bot.pos[1])**2)

    # Update screen text
    sim_time += 1000/speed
    l.config(text='Simulation Time (s): ' + str(float(sim_time/1000)))
    l2.config(text='Error : ' + str(calc_error))
    root.after(int(1000/speed), SLAM_move)

    
if __name__ == '__main__':

    ### Create Bot ###
    bot = Car(c)
    bot.build([float(w/2), float(h/2)])
    c.bind('<Configure>', create_grid)
    c.bind('<Button-1>', get_x_and_y)
    c.bind('<B1-Motion>', draw_path)

    
    ### Setup PID Control ###
    kp, ki, kd = 5, 0.00001, 0.5
    # kp, ki, kd = 4, 0.000001, 1.1
    PID_ctrl = PID(kp=kp, ki=ki, kd=kd, f_dist=0.1)

    ### Setup Lead-Lag Control ###
    # kp, ki, kd = 5, 0.001, 0.0008
    # PID_ctrl = PID(kp=kp, ki=ki, kd=kd, f_dist=0.1)

    ### Setup Extended Kalman Filter ###

    ## Create lighthouse and the detection range circle
    # r, detect_range = 10, 200
    # x1, y1 = grid_2_pixel(10, 1)
    # c.create_oval(x1-detect_range, y1-detect_range, x1+detect_range, y1+detect_range, fill='white')
    # c.create_oval(x1-r, y1-r, x1+r, y1+r, fill='red')

    # # Setup Kalman variables and filter class
    # X_k = np.array([[bot.pos[0],bot.pos[1],bot.angle]]).T                       # Initial pose
    # perr, qerr, rerr = 100e-5, 500e-3, 10e-3                                     # Kalman filter error terms
    # P_k, Q_k, R_k = np.eye(3) * perr, np.eye(3) * qerr, np.eye(2) * rerr        # State, Process, Measurement Covariances
    # u_k = np.array([[1,0]]).T                                                   # Input: (velocity, angular velocity)
    # ekf = EKF(X_0=X_k, dt=0.04, u=u_k, Q=Q_k, R=R_k, P_0=P_k, x_S=x1, y_S=y1)   # Initialization of extended kalman filter

    ### Setup SLAM ###
    
    # Create obstacles to detect and store their locations
    # num_obs = 5
    # detect_range = 100
    # obs_arr = []
    # for i in range(num_obs):
    #     n1, n2 = random.randint(0,w), random.randint(0,h)
    #     obs_arr += [[n1,n2]]
    #     c.create_oval(n1-detect_range, n2-detect_range, n1+detect_range, n2+detect_range, fill='white')
    #     c.create_oval(n1-5, n2-5, n1+5, n2+5, fill='red')

    # # Setup Kalman variables and SLAM class
    # X_k = np.concatenate((np.array([[bot.pos[0],bot.pos[1],bot.angle]]).reshape(3,1), np.zeros((2*num_obs,1))), axis=0)
    # Fx = np.concatenate((np.eye(3), np.zeros((3, 2*num_obs))), axis=1)
    # perr, qerr, rerr = 1e-5, 50e-3, 1e-3  
    # P_k, Q_k, R_k = np.eye(3) * perr, np.eye(3) * qerr, np.eye(2) * rerr
    # P_k = Fx.T @ P_k @ Fx
    # slam = SLAM(X_0=X_k, Fx=Fx, dt=0.04, Q=Q_k, R=R_k, P_0=P_k, num_obs=num_obs, obs_arr=obs_arr)


    ### Setup Unscented Kalman Filter ###

    # Create lighthouse and the detection range circle
    r, detect_range = 10, 200
    x1, y1 = grid_2_pixel(10, 1)
    c.create_oval(x1-detect_range, y1-detect_range, x1+detect_range, y1+detect_range, fill='white')
    c.create_oval(x1-r, y1-r, x1+r, y1+r, fill='red')

    # Setup Kalman variables and filter class
    X_k = np.array([[bot.pos[0],bot.pos[1],bot.angle]]).T                       # Initial pose
    perr, qerr, rerr = 1e-5, 5e-3, 1e-3                                     # Kalman filter error terms
    P_k, Q_k, R_k = np.eye(3) * perr, np.eye(3) * qerr, np.eye(2) * rerr        # State, Process, Measurement Covariances
    ekf = UKF(X_0=X_k, dt=0.04, Q=Q_k, R=R_k, P_0=P_k, x_S=x1, y_S=y1)          # Initialization of extended kalman filter

    ### Canvas Buttons and Labels ###
    b = Button(root, text='Run', command=EKF_move, font=('Helvetica',16), fg='black')
    b.place(x=58, y=10)
    l = Label(root, text='Simulation Time (s): 0', fg='black')
    l.place(x=58, y=60)
    l2 = Label(root, text='PID Parameters: ', fg='black')
    l2.place(x=58, y=110)

    root.mainloop()