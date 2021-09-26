import numpy as np
import time
from func_main import *

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
        self.rot_inc = None

        # Other properties
        self.mass = 1   # Unit: kg
        self.turn_radius = 1

        # Pose - self.pos[0] = x-coordinate, self.pos[1] = y-coordinate, self.angle = orientation
        self.pos, self.angle = np.zeros(2), 0

    # Builds the Car in the Canvas
    def build(self, S):

        # Initialize Pose and Body Points
        self.pos[0], self.pos[1] = S[0], S[1]
        self.body_points = [S[0]-10, S[1]-10, S[0]+10, S[1]-10, S[0]+10, S[1]+10, S[0]-10, S[1]+10]
        self.indic_points = [S[0], S[1]-8, S[0]+8, S[1]-8, S[0]+8, S[1]+8, S[0], S[1]+8]
        self.w1_points = [S[0]-12, S[1]-12, S[0]-4, S[1]-12, S[0]-4, S[1]-6, S[0]-12, S[1]-6]
        self.w2_points = [S[0]-12, S[1]+12, S[0]-4, S[1]+12, S[0]-4, S[1]+6, S[0]-12, S[1]+6]
        self.w3_points = [S[0]+12, S[1]+12, S[0]+4, S[1]+12, S[0]+4, S[1]+6, S[0]+12, S[1]+6]
        self.w4_points = [S[0]+12, S[1]-12, S[0]+4, S[1]-12, S[0]+4, S[1]-6, S[0]+12, S[1]-6]

        # Build Car
        self.body = self.canvas.create_polygon(self.body_points, fill='blue')
        self.indic = self.canvas.create_polygon(self.indic_points, fill='red')
        self.w1 = self.canvas.create_polygon(self.w1_points, fill='black')
        self.w2 = self.canvas.create_polygon(self.w2_points, fill='black')
        self.w3 = self.canvas.create_polygon(self.w3_points, fill='black')
        self.w4 = self.canvas.create_polygon(self.w4_points, fill='black')

    # Car movement
    def move(self):

        # Store current points with shorter variable names
        BP, IP, W1, W2, W3, W4 = self.body_points, self.indic_points, self.w1_points, self.w2_points, self.w3_points, self.w4_points
        
        # Compute new coordinates
        M = self.speed * np.array([np.cos(self.angle), -np.sin(self.angle)])

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
        self.angle += dir * deg_2_rad(self.rot_inc)
    

    # Rotation functions
    def rotateCCW(self, r):
        self.rot_inc = 180/(np.pi * r * 25)
        self.rotate_main(dir=1)

    def rotateCW(self, r):
        self.rot_inc = 180/(np.pi * r * 25)
        self.rotate_main(dir=-1)
        
    def rot_helper(self, A, dir):
        P1 = self.rot_matrix(np.array([[A[0]-self.pos[0], A[1]-self.pos[1]]]), dir)[0]
        P2 = self.rot_matrix(np.array([[A[2]-self.pos[0], A[3]-self.pos[1]]]), dir)[0]
        P3 = self.rot_matrix(np.array([[A[4]-self.pos[0], A[5]-self.pos[1]]]), dir)[0]
        P4 = self.rot_matrix(np.array([[A[6]-self.pos[0], A[7]-self.pos[1]]]), dir)[0]
        return [P1[0], P1[1], P2[0], P2[1], P3[0], P3[1], P4[0], P4[1]]

    # Defines rotation matrix to rotate body points
    def rot_matrix(self, V, dir):
        ang = float(deg_2_rad(self.rot_inc))
        C = np.array([[np.cos(ang), dir*np.sin(ang)], [dir * -1 * np.sin(ang), np.cos(ang)]])
        X = np.matmul(C, V.reshape(2,-1))
        return X.reshape(1,2) + self.pos


class Dubin:

    def __init__(self, canvas):

        # Canvas
        self.canvas = canvas

        # Start and end positions of vehicle
        self.x_start = 0
        self.y_start = 0
        self.x_end = 10
        self.y_end = 10
        self.a_start = deg_2_rad(90)
        self.a_end = deg_2_rad(45)

        # Minimum turn radius of Car
        self.tr = 1

    def create_circles(self):
        
        x1, y1, x2, y2 = self.build_start_circles(self.tr, self.a_start, self.x_start, self.y_start)
        x3, y3, x4, y4 = self.build_start_circles(self.tr, self.a_end, self.x_end, self.y_end)
        

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

        # Print circles on canvas
        xa, ya = grid_2_pixel(x1, y1)
        self.canvas.create_oval(xa-self.tr*25, ya-self.tr*25, xa+self.tr*25, ya+self.tr*25)
        xb, yb = grid_2_pixel(x2, y2)
        self.canvas.create_oval(xb-self.tr*25, yb-self.tr*25, xb+self.tr*25, yb+self.tr*25)
        
        return x1, y1, x2, y2


    