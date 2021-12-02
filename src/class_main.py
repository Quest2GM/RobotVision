import numpy as np
import time
from func_main import *

import numpy as np
from scipy.spatial.distance import cdist

# Avoid division by zero warning
np.seterr(divide='ignore')

class Car:

    def __init__(self, canvas):

        # Defines the Tkinter Canvas
        self.canvas = canvas

        # Canvas coordinates used to build car
        self.body, self.body_points = None, None
        self.indic, self.indic_points = None, None
        self.w1, self.w1_points = None, None
        self.w2, self.w2_points = None, None
        self.w3, self.w3_points = None, None
        self.w4, self.w4_points = None, None

        # Rotation Increment
        self.speed = 1
        self.rot_inc = deg_2_rad(0)

        # Other properties
        self.mass = 1   # Unit: kg
        self.turn_radius = 1

        # Pose - self.pos[0] = x-coordinate, self.pos[1] = y-coordinate, self.angle = orientation
        self.pos, self.angle = np.zeros(2), deg_2_rad(0)
        self.MX = None

    # Builds the Car in the Canvas
    def build(self, S):

        # Initialize Pose and Body Points
        self.pos[0], self.pos[1] = S[0], S[1]
        self.body_points =  [S[0]-10, S[1]-10, S[0]+10, S[1]-10, S[0]+10, S[1]+10, S[0]-10, S[1]+10]
        self.indic_points = [S[0], S[1]-8, S[0]+8, S[1]-8, S[0]+8, S[1]+8, S[0], S[1]+8]
        self.w1_points =    [S[0]-12, S[1]-12, S[0]-4, S[1]-12, S[0]-4, S[1]-6, S[0]-12, S[1]-6]
        self.w2_points =    [S[0]-12, S[1]+12, S[0]-4, S[1]+12, S[0]-4, S[1]+6, S[0]-12, S[1]+6]
        self.w3_points =    [S[0]+12, S[1]+12, S[0]+4, S[1]+12, S[0]+4, S[1]+6, S[0]+12, S[1]+6]
        self.w4_points =    [S[0]+12, S[1]-12, S[0]+4, S[1]-12, S[0]+4, S[1]-6, S[0]+12, S[1]-6]

        # Build Car
        self.body = self.canvas.create_polygon(self.body_points, fill='blue')
        self.indic = self.canvas.create_polygon(self.indic_points, fill='red')
        self.w1 = self.canvas.create_polygon(self.w1_points, fill='black')
        self.w2 = self.canvas.create_polygon(self.w2_points, fill='black')
        self.w3 = self.canvas.create_polygon(self.w3_points, fill='black')
        self.w4 = self.canvas.create_polygon(self.w4_points, fill='black')

        # Rotate if necessary
        self.rotate_main(dir=1)


    # Car movement
    def move(self, noise=[0,0]):

        # Store current points with shorter variable names
        BP, IP, W1, W2, W3, W4 = self.body_points, self.indic_points, self.w1_points, self.w2_points, self.w3_points, self.w4_points
        
        # Compute new coordinates (add Gaussian noise if needed)
        self.MX = self.pos + self.speed * np.array([np.cos(self.angle), -np.sin(self.angle)])
        M = (self.speed + np.random.normal(noise[0], noise[1], 1)[0]) * np.array([np.cos(self.angle), -np.sin(self.angle)])

        # Move body in the direction
        self.canvas.move(self.body, M[0], M[1])
        self.canvas.move(self.indic, M[0], M[1])
        self.canvas.move(self.w1, M[0], M[1])
        self.canvas.move(self.w2, M[0], M[1])
        self.canvas.move(self.w3, M[0], M[1])
        self.canvas.move(self.w4, M[0], M[1])

        # Line to visually track the car's path
        self.canvas.create_line(self.pos[0], self.pos[1], self.pos[0]+M[0], self.pos[1]+M[1])

        # Update position and body_points
        self.pos += M
        self.body_points  = [BP[0]+M[0], BP[1]+M[1], BP[2]+M[0], BP[3]+M[1], BP[4]+M[0], BP[5]+M[1], BP[6]+M[0], BP[7]+M[1]]
        self.indic_points = [IP[0]+M[0], IP[1]+M[1], IP[2]+M[0], IP[3]+M[1], IP[4]+M[0], IP[5]+M[1], IP[6]+M[0], IP[7]+M[1]]
        self.w1_points =    [W1[0]+M[0], W1[1]+M[1], W1[2]+M[0], W1[3]+M[1], W1[4]+M[0], W1[5]+M[1], W1[6]+M[0], W1[7]+M[1]]
        self.w2_points =    [W2[0]+M[0], W2[1]+M[1], W2[2]+M[0], W2[3]+M[1], W2[4]+M[0], W2[5]+M[1], W2[6]+M[0], W2[7]+M[1]]
        self.w3_points =    [W3[0]+M[0], W3[1]+M[1], W3[2]+M[0], W3[3]+M[1], W3[4]+M[0], W3[5]+M[1], W3[6]+M[0], W3[7]+M[1]]
        self.w4_points =    [W4[0]+M[0], W4[1]+M[1], W4[2]+M[0], W4[3]+M[1], W4[4]+M[0], W4[5]+M[1], W4[6]+M[0], W4[7]+M[1]]

    
    # Performs rotation on car. dir=1 => CCW, dir=-1 => CW
    def rotate_main(self, dir):

        # Sequentially delete Car and reestablish with Car, post rotation
        self.canvas.delete(self.body)
        self.body_points = self.rot_helper(self.body_points, dir)
        self.body = self.canvas.create_polygon(self.body_points, fill='blue')

        self.canvas.delete(self.indic)
        self.indic_points = self.rot_helper(self.indic_points, dir)
        self.indic = self.canvas.create_polygon(self.indic_points, fill='red')

        self.canvas.delete(self.w1)
        self.w1_points = self.rot_helper(self.w1_points, dir)
        self.w1 = self.canvas.create_polygon(self.w1_points, fill='black')

        self.canvas.delete(self.w2)
        self.w2_points = self.rot_helper(self.w2_points, dir)
        self.w2 = self.canvas.create_polygon(self.w2_points, fill='black')

        self.canvas.delete(self.w3)
        self.w3_points = self.rot_helper(self.w3_points, dir)
        self.w3 = self.canvas.create_polygon(self.w3_points, fill='black')

        self.canvas.delete(self.w4)
        self.w4_points = self.rot_helper(self.w4_points, dir)
        self.w4 = self.canvas.create_polygon(self.w4_points, fill='black')

        # Update orientation in radians
        self.angle += dir * self.rot_inc
    

    # Rotation functions
    def rotateCCW(self, r):
        self.rot_inc = 1 / (25 * r) * (1/25)
        self.rotate_main(dir=1)

    def rotateCW(self, r):
        self.rot_inc = 1 / (25 * r) * (1/25)
        self.rotate_main(dir=-1)
        
    def rot_helper(self, A, dir):
        P1 = self.rot_matrix(np.array([[A[0]-self.pos[0], A[1]-self.pos[1]]]), dir)[0]
        P2 = self.rot_matrix(np.array([[A[2]-self.pos[0], A[3]-self.pos[1]]]), dir)[0]
        P3 = self.rot_matrix(np.array([[A[4]-self.pos[0], A[5]-self.pos[1]]]), dir)[0]
        P4 = self.rot_matrix(np.array([[A[6]-self.pos[0], A[7]-self.pos[1]]]), dir)[0]
        return [P1[0], P1[1], P2[0], P2[1], P3[0], P3[1], P4[0], P4[1]]

    # Defines rotation matrix to rotate body points
    def rot_matrix(self, V, dir):
        ang = float(self.rot_inc)
        C = np.array([[np.cos(ang), dir*np.sin(ang)], [dir * -1 * np.sin(ang), np.cos(ang)]])
        X = np.matmul(C, V.reshape(2,-1))
        return X.reshape(1,2) + self.pos


class Dubin:

    def __init__(self, canvas):

        # Canvas
        self.canvas = canvas

        # Start and end positions of vehicle
        self.x_start, self.y_start = 10, 10
        self.x_end, self.y_end = 0, 0
        self.a_start, self.a_end = deg_2_rad(45), deg_2_rad(270)

        # Minimum turn radius of Car
        self.tr = 1

    def create_paths(self):
        
        # Start and end min-radius circles
        x1, y1, x2, y2 = self.build_start_circles(self.tr, self.a_start, self.x_start, self.y_start)
        x3, y3, x4, y4 = self.build_start_circles(self.tr, self.a_end, self.x_end, self.y_end)

        # Get outer/inner circles
        outer113, outer213, inner113, inner213 = self.circle_POIs(x1, y1, x3, y3)
        outer114, outer214, inner114, inner214 = self.circle_POIs(x1, y1, x4, y4)
        outer123, outer223, inner123, inner223 = self.circle_POIs(x2, y2, x3, y3)
        outer124, outer224, inner124, inner224 = self.circle_POIs(x2, y2, x4, y4)

        ds1 = self.cw_or_ccw(x1, y1, self.x_start, self.y_start, self.a_start)
        ds2 = self.cw_or_ccw(x2, y2, self.x_start, self.y_start, self.a_start)
        de1 = self.cw_or_ccw(x3, y3, self.x_end, self.y_end, self.a_end)
        de2 = self.cw_or_ccw(x4, y4, self.x_end, self.y_end, self.a_end)

        S_list = []

        print(ds1, ds2, de1, de2)

        # Circle 1 to Circle 3:
        if ds1 == 1 and de1 == 1:
            S_list += [outer213]
        elif ds1 == 1 and de1 == -1:
            S_list += [inner113]
        elif ds1 == -1 and de1 == 1:
            S_list += [inner213]
        elif ds1 == -1 and de1 == -1:
            S_list += [outer113]
                
        # Circle 1 to Circle 4:
        if ds1 == 1 and de2 == 1:
            S_list += [outer214]
        elif ds1 == 1 and de2 == -1:
            S_list += [inner114]
        elif ds1 == -1 and de2 == 1:
            S_list += [inner214]
        elif ds1 == -1 and de2 == -1:
            S_list += [outer114]
        
        # Circle 2 to Circle 3:
        if ds2 == 1 and de1 == 1:
            S_list += [outer223]
        elif ds2 == 1 and de1 == -1:
            S_list += [inner123]
        elif ds2 == -1 and de1 == 1:
            S_list += [inner223]
        elif ds2 == -1 and de1 == -1:
            S_list += [outer123]

        # Circle 2 to Circle 4:
        if ds2 == 1 and de2 == 1:
            S_list += [outer224]
        elif ds2 == 1 and de2 == -1:
            S_list += [inner124]
        elif ds2 == -1 and de2 == 1:
            S_list += [inner224]
        elif ds2 == -1 and de2 == -1:
            S_list += [outer124]
        
        self.canvas.create_line(S_list[0], fill='blue', width=2)
        self.canvas.create_line(S_list[1], fill='red', width=2)
        self.canvas.create_line(S_list[2], fill='blue', width=2)
        self.canvas.create_line(S_list[3], fill='red', width=2)

        dir_list = [(ds1,de1), (ds1,de2), (ds2,de1), (ds2,de2)]
        cen_list = [(x1,y1,x3,y3), (x1,y1,x4,y4), (x2,y2,x3,y3), (x2,y2,x4,y4)]
        dist_list = []

        # Compute Path Distances
        for i, S in enumerate(S_list):
            a, b, c, d = S
            (a,b), (c,d) = pixel_2_grid(a,b), pixel_2_grid(c,d)
            
            X1 = np.arctan2(b-cen_list[i][1], a-cen_list[i][0]) - np.arctan2(self.y_start-cen_list[i][1], self.x_start-cen_list[i][0])
            X2 = np.arctan2(d-cen_list[i][3], c-cen_list[i][2]) - np.arctan2(self.y_end-cen_list[i][3], self.x_end-cen_list[i][2])
            if X1 < 0 and dir_list[i][0] == 1:
                X1 = X1 + 2*np.pi
            elif X1 > 0 and dir_list[i][0] == -1:
                X1 = X1 - 2*np.pi
            
            if X2 < 0 and dir_list[i][1] == -1:
                X2 = X2 + 2*np.pi
            elif X2 > 0 and dir_list[i][1] == 1:
                X2 = X2 - 2*np.pi

            # print(X1, dir_list[i][0], dir_list[i][1])
            # print(dist(a,b,c,d), np.abs(X1*self.tr), np.abs(X2*self.tr))    
            dist_list += [dist(a,b,c,d) + np.abs(X1*self.tr) + np.abs(X2*self.tr)]

        
        print(dist_list)

    def build_start_circles(self, r, a, x, y):

        # Compute circle's positions
        x1 = -r/np.sqrt(1 + np.square(1/np.tan(a))) + x
        x2 = r/np.sqrt(1 + np.square(1/np.tan(a))) + x

        if np.tan(a) == 0:
            y1 = y - r
            y2 = y + r
        else:
            y1 = -1/np.tan(a) * (x1 - x) + y
            y2 = -1/np.tan(a) * (x2 - x) + y

        # Display circles on canvas
        xa, ya = grid_2_pixel(x1, y1)
        xb, yb = grid_2_pixel(x2, y2)
        self.canvas.create_oval(xa-self.tr*25, ya-self.tr*25, xa+self.tr*25, ya+self.tr*25, width=2, fill='gray')
        self.canvas.create_oval(xb-self.tr*25, yb-self.tr*25, xb+self.tr*25, yb+self.tr*25, width=2, fill='cyan')
        
        return x1, y1, x2, y2

    def circle_POIs(self, x1, y1, x2, y2):

        # Relative angle of inclination/declination of second circle vs first circle
        theta = np.arctan((y2-y1)/(x2-x1))

        # Combined tangent mid-point
        xp, yp = (x1 + x2)/2, (y1 + y2)/2

        # Outer Tangent 1 - CW departure / CW arrival
        a1, b1 = x1 - self.tr * np.sin(theta), y1 + self.tr * np.cos(theta)
        a2, b2 = x2 - self.tr * np.sin(theta), y2 + self.tr * np.cos(theta)

        # Outer Tangent 2 - CCW departure / CCW arrival
        c1, d1 = x1 + self.tr * np.sin(theta), y1 - self.tr * np.cos(theta)
        c2, d2 = x2 + self.tr * np.sin(theta), y2 - self.tr * np.cos(theta)

        # Inner Tangent 1 - CCW departure / CW arrival
        e1 = (np.square(self.tr)*(xp-x1) + self.tr*(yp-y1) * np.sqrt(np.square(xp-x1)+np.square(yp-y1)-np.square(self.tr))) \
             / (np.square(xp-x1) + np.square(yp-y1)) + x1
        f1 = (np.square(self.tr)*(yp-y1) - self.tr*(xp-x1) * np.sqrt(np.square(xp-x1)+np.square(yp-y1)-np.square(self.tr))) \
             / (np.square(xp-x1) + np.square(yp-y1)) + y1
        e2 = (np.square(self.tr)*(xp-x2) + self.tr*(yp-y2) * np.sqrt(np.square(xp-x2)+np.square(yp-y2)-np.square(self.tr))) \
             / (np.square(xp-x2) + np.square(yp-y2)) + x2
        f2 = (np.square(self.tr)*(yp-y2) - self.tr*(xp-x2) * np.sqrt(np.square(xp-x2)+np.square(yp-y2)-np.square(self.tr))) \
             / (np.square(xp-x2) + np.square(yp-y2)) + y2
        
        # Inner Tangent 2 - CW departure / CCW arrival
        g1 = (np.square(self.tr)*(xp-x1) - self.tr*(yp-y1) * np.sqrt(np.square(xp-x1)+np.square(yp-y1)-np.square(self.tr))) \
             / (np.square(xp-x1) + np.square(yp-y1)) + x1
        h1 = (np.square(self.tr)*(yp-y1) + self.tr*(xp-x1) * np.sqrt(np.square(xp-x1)+np.square(yp-y1)-np.square(self.tr))) \
             / (np.square(xp-x1) + np.square(yp-y1)) + y1
        g2 = (np.square(self.tr)*(xp-x2) - self.tr*(yp-y2) * np.sqrt(np.square(xp-x2)+np.square(yp-y2)-np.square(self.tr))) \
             / (np.square(xp-x2) + np.square(yp-y2)) + x2
        h2 = (np.square(self.tr)*(yp-y2) + self.tr*(xp-x2) * np.sqrt(np.square(xp-x2)+np.square(yp-y2)-np.square(self.tr))) \
             / (np.square(xp-x2) + np.square(yp-y2)) + y2

        # Convert to pixel coordinates
        (a1, b1), (a2, b2) = grid_2_pixel(a1,b1), grid_2_pixel(a2,b2)
        (c1, d1), (c2, d2) = grid_2_pixel(c1,d1), grid_2_pixel(c2,d2)
        (e1, f1), (e2, f2) = grid_2_pixel(e1,f1), grid_2_pixel(e2,f2)
        (g1, h1), (g2, h2) = grid_2_pixel(g1,h1), grid_2_pixel(g2,h2)

        return ([a1,b1,a2,b2], [c1,d1,c2,d2], [e1,f1,e2,f2], [g1,h1,g2,h2])

    # Determines whether circle needs to be entered ccw or cw. (x_c, y_c) = circle center, (x, y) = destination point
    def cw_or_ccw(self, x_c, y_c, x, y, a):
        ccw = [[90, 180], [180, 270], [270, 360], [0, 90]]
        cw  = [[270, 360], [0, 90], [90, 180], [180, 270]]

        S = (x_c - x)/(y - y_c)
        A = np.arctan2(y - y_c, x_c - x)
        a = rad_2_deg(a)

        # Return 1 if CCW, Return -1 if CW
        if (S == np.inf and A == 0) or (S <= 0 and A > 0):
            if a >= ccw[0][0] and a <= ccw[0][1]:
                return 1
            elif a >= cw[0][0] and a <= cw[0][1]:
                return -1
        elif (S >= 0 and A > 0):
            if a >= ccw[1][0] and a <= ccw[1][1]:
                return 1
            elif a >= cw[1][0] and a <= cw[1][1]:
                return -1
        elif (S == np.inf and A > 0) or (S <= 0 and A < 0):
            if a >= ccw[2][0] and a <= ccw[2][1]:
                return 1
            elif a >= cw[2][0] and a <= cw[2][1]:
                return -1
        elif (S == np.inf and A == 0) or (S >= 0 and A < 0):
            if a >= ccw[3][0] and a <= ccw[3][1]:
                return 1
            elif a >= cw[3][0] and a <= cw[3][1]:
                return -1
        
        return None


class PID:

    def __init__(self, kp, ki, kd, f_dist):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.P, self.I, self.D, self.LE = 0, 0, 0, 0
        self.e_rot = None
        self.E = None
        self.f_dist = f_dist

    def compute_dist_dir(self, path_arr, bot_pos, bot_angle):

        # Calculate minimum distance        
        dists = cdist([bot_pos], path_arr, 'euclidean')[0]
        ind_min = np.argmin(dists)
        min_dist_point = path_arr[ind_min]
        min_dist_point_2 = path_arr[ind_min + 5]

        # Convert points into grid coordinates
        bp = pixel_2_grid(*bot_pos)
        md = pixel_2_grid(*min_dist_point)

        # Compute the vectors that join the closest point and the bot, and the direction of the bot
        bot_line_pos = list([bp[0] - md[0], bp[1] - md[1]])
        bot_dir_vec = [np.cos(bot_angle), np.sin(bot_angle)]

        # Compute distance error
        self.P = dist(*bp, *md) - self.f_dist
        
        # Compute cross-product of two vectors to determine whether the bot should rotate CW or CCW to follow path
        self.e_rot = np.cross(bot_line_pos + [0], bot_dir_vec + [0])[2]


    def update_gains(self, speed):

        self.E = self.kp * self.P + self.ki * self.I + self.kd * self.D

        # Correct I and D terms
        self.I += self.P * (1/speed)
        self.D = (self.P - self.LE)/(1/speed)
        self.LE = self.P

        return self.E, self.e_rot


class Kalman:

    def __init__(self, X_0, u, dt, Q, R, P_0, x_S, y_S):
        
        # Initialize Scalars
        self.dt = dt    # Time increment
        self.x_S = x_S  # Sensor X Value
        self.y_S = y_S  # Sensor Y Value

        # Initialize Matrix Variables
        self.X = X_0        # State                  [3x1]
        self.Z = None       # Measurement            [1x1]
        self.u = u          # Control Input (v,w)    [2x1]
        self.Q = Q          # Process Noise          [3x3]
        self.R = R          # Measurement Noise      [1x1]              
        self.P = P_0        # State Covariance       [3x3]
        self.S = None       # Measurement Covariance [1x1]
        self.W = None       # Kalman Gain            [3x1]
        self.A = None       # 'A' Jacobian           [3x3]
        self.B = None       # 'B' Jacobian           [3x2]
        self.D = None       # 'D' Jacobian           [1x3]


    def predict(self, w, f):
        
         # Determine A, B, D
        self.A, self.B, self.D = self.update_ABD(self.X[0][0], self.X[1][0], self.X[2][0], w, f)

        # Predict apriori state covariance
        self.P = self.A @ self.P @ self.A.T + self.Q

        # Add control input w (angular velocity) - from PID control algorithm
        self.u = np.array([1,w]).reshape(2,1)

        # Predict apriori state estimate
        self.X = self.A @ self.X + self.B @ self.u


    def update(self, Z_meas, curr_dist):

        if curr_dist > 100:
            return self.X

         # Predict apriori measurement covariance
        self.S = self.D @ self.P @ self.D.T + self.R

        # Determine Kalman Gain
        self.W = self.P @ self.D.T @ np.linalg.inv(self.S)

        # Predict apriori measurement
        x_CN, y_CN = pixel_2_grid(self.x_S, self.y_S)
        x_pos, y_pos = pixel_2_grid(self.X[0][0], self.X[1][0])
        Z1 = np.arctan2(y_CN-y_pos, x_CN-x_pos) - self.X[2][0]
        Z2 = np.sqrt((self.x_S-self.X[0][0])**2 + (self.y_S-self.X[1][0])**2)
        self.Z = np.array([Z1, Z2]).reshape(2,1)

        print('Predicted X: ', self.X)
        print('Predicted Z: ', np.array([Z1,Z2]).reshape(2,1))
        
        # Update to aposteriori state covariance
        self.P -= self.W @ self.S @ self.W.T

        # Update to aposteriori state estimate
        self.X += self.W @ (Z_meas-self.Z)

        return self.X

    def update_ABD(self, xk, yk, tk, wk, f):
        Ak = np.array([[1, 0, -f*self.dt * np.sin(tk + f*wk*self.dt)], 
                       [0, 1, -f*self.dt * np.cos(tk + f*wk*self.dt)],
                       [0, 0, 1]])
        Bk = np.array([[np.cos(tk + f*wk*self.dt), -f*self.dt * np.sin(tk + f*wk*self.dt)],
                       [-np.sin(tk + f*wk*self.dt), -f*self.dt * np.cos(tk + f*wk*self.dt)],
                       [0, f*self.dt]])

        d = np.sqrt((self.x_S-xk)**2 + (self.y_S-yk)**2)
        Dk = np.array([[(self.y_S-yk)/(d**2), (xk-self.x_S)/(d**2), -1],
                       [(xk-self.x_S)/d, (yk-self.y_S)/d, 0]])
        
        return Ak, Bk, Dk

    