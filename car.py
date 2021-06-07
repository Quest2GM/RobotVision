import numpy as np

class Car:
    def __init__(self, root, canvas):
        self.root = root
        self.canvas = canvas
        self.w1, self.w2, self.w3, self.w4, self.body, self.indic = None, None, None, None, None, None
        self.w1_points, self.w2_points, self.w3_points, self.w4_points = None, None, None, None
        self.body_points, self.indic_points = None, None
        self.angle = (0,0)
        self.pos = None

    def build(self, S):
        self.pos = np.array([S[0], S[1]])
        self.w1_points = (S[0]-12, S[1]-12, S[0]-4, S[1]-12, S[0]-4, S[1]-6, S[0]-12, S[1]-6)
        self.w2_points = (S[0]-12, S[1]+12, S[0]-4, S[1]+12, S[0]-4, S[1]+6, S[0]-12, S[1]+6)
        self.w3_points = (S[0]+12, S[1]+12, S[0]+4, S[1]+12, S[0]+4, S[1]+6, S[0]+12, S[1]+6)
        self.w4_points = (S[0]+12, S[1]-12, S[0]+4, S[1]-12, S[0]+4, S[1]-6, S[0]+12, S[1]-6)
        self.body_points = (S[0]-10, S[1]-10, S[0]+10, S[1]-10, S[0]+10, S[1]+10, S[0]-10, S[1]+10)
        self.indic_points = (S[0], S[1]-8, S[0]+8, S[1]-8, S[0]+8, S[1]+8, S[0], S[1]+8)
        self.w1 = self.canvas.create_polygon(self.w1_points, fill='black')
        self.w2 = self.canvas.create_polygon(self.w2_points, fill='black')
        self.w3 = self.canvas.create_polygon(self.w3_points, fill='black')
        self.w4 = self.canvas.create_polygon(self.w4_points, fill='black')
        self.body = self.canvas.create_polygon(self.body_points, fill='blue')
        self.indic = self.canvas.create_polygon(self.indic_points, fill='red')

    def move(self, event):
        move_xy = (2*np.cos(self.angle[0]), 2*np.sin(self.angle[1]))
        self.canvas.move(self.w1, move_xy[0], move_xy[1])
        self.canvas.move(self.w2, move_xy[0], move_xy[1])
        self.canvas.move(self.w3, move_xy[0], move_xy[1])
        self.canvas.move(self.w4, move_xy[0], move_xy[1])
        self.canvas.move(self.body, move_xy[0], move_xy[1])
        self.canvas.move(self.indic, move_xy[0], move_xy[1])
        new_pos = np.array([move_xy[0], move_xy[1]])
        self.pos = self.pos + new_pos

    def rotate(self, event):
        self.w1_points = self.rot_help(self.w1_points, self.pos)
        self.w2_points = self.rot_help(self.w2_points, self.pos)
        self.w3_points = self.rot_help(self.w3_points, self.pos)
        self.w4_points = self.rot_help(self.w4_points, self.pos)
        self.body_points = self.rot_help(self.body_points, self.pos)
        self.indic_points = self.rot_help(self.indic_points, self.pos)
        self.canvas.delete(self.w1)
        self.canvas.delete(self.w2)
        self.canvas.delete(self.w3)
        self.canvas.delete(self.w4)
        self.canvas.delete(self.body)
        self.canvas.delete(self.indic)
        self.w1 = self.canvas.create_polygon(self.w1_points, fill='black')
        self.w2 = self.canvas.create_polygon(self.w2_points, fill='black')
        self.w3 = self.canvas.create_polygon(self.w3_points, fill='black')
        self.w4 = self.canvas.create_polygon(self.w4_points, fill='black')
        self.body = self.canvas.create_polygon(self.body_points, fill='blue')
        self.indic = self.canvas.create_polygon(self.indic_points, fill='red')
        self.angle = (self.angle[0]+np.cos(np.deg2rad(1)), self.angle[1]-np.sin(np.deg2rad(1)))



    def rot_help(self, points, center, angle=1):
        cos_val = np.cos(np.deg2rad(angle))
        sin_val = np.sin(np.deg2rad(angle))
        points = np.array(points)
        points = tuple(points.reshape(-1,2))
        print(center)
        cx, cy = center
        new_points = []
        for x_old, y_old in points:
            x_old -= cx
            y_old -= cy
            x_new = x_old * cos_val - y_old * sin_val
            y_new = x_old * sin_val + y_old * cos_val
            new_points.append([x_new + cx, y_new + cy])

        x = np.array(new_points)
        y = tuple(x.reshape(-1, ))
        return y

