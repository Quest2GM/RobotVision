import numpy as np

class Car:
    def __init__(self, root, canvas):
        self.root = root
        self.canvas = canvas
        self.body, self.body_points = None, None
        self.indic, self.indic_points = None, None
        self.w1, self.w2, self.w3, self.w4 = None, None, None, None
        self.w1_points, self.w2_points, self.w3_points, self.w4_points = None, None, None, None
        self.rot_inc = 45
        self.pose = np.array([[0,0],[0,0]]).astype(np.float64)

    def build(self, S):
        self.pose[0][0], self.pose[0][1] = S[0], S[1]
        self.body_points = [S[0]-10, S[1]-10, S[0]+10, S[1]-10, S[0]+10, S[1]+10, S[0]-10, S[1]+10]
        self.indic_points = [S[0],S[1]-8, S[0]+8, S[1]-8, S[0]+8, S[1]+8, S[0], S[1]+8]
        self.w1_points = [S[0] - 12, S[1] - 12, S[0] - 4, S[1] - 12, S[0] - 4, S[1] - 6, S[0] - 12, S[1] - 6]
        self.w2_points = [S[0] - 12, S[1] + 12, S[0] - 4, S[1] + 12, S[0] - 4, S[1] + 6, S[0] - 12, S[1] + 6]
        self.w3_points = [S[0] + 12, S[1] + 12, S[0] + 4, S[1] + 12, S[0] + 4, S[1] + 6, S[0] + 12, S[1] + 6]
        self.w4_points = [S[0] + 12, S[1] - 12, S[0] + 4, S[1] - 12, S[0] + 4, S[1] - 6, S[0] + 12, S[1] - 6]
        self.body = self.canvas.create_polygon(self.body_points, fill='blue')
        self.indic = self.canvas.create_polygon(self.indic_points, fill='red')
        self.w1 = self.canvas.create_polygon(self.w1_points, fill='black')
        self.w2 = self.canvas.create_polygon(self.w2_points, fill='black')
        self.w3 = self.canvas.create_polygon(self.w3_points, fill='black')
        self.w4 = self.canvas.create_polygon(self.w4_points, fill='black')

    def move(self, event):
        M = np.array([2 * np.cos(self.pose[1][0]), -2 * np.sin(self.pose[1][0])])
        BP = self.body_points
        IP = self.indic_points
        W1 = self.w1_points
        W2 = self.w2_points
        W3 = self.w3_points
        W4 = self.w4_points
        self.canvas.move(self.body, M[0], M[1])
        self.canvas.move(self.indic, M[0], M[1])
        self.canvas.move(self.w1, M[0], M[1])
        self.canvas.move(self.w2, M[0], M[1])
        self.canvas.move(self.w3, M[0], M[1])
        self.canvas.move(self.w4, M[0], M[1])
        self.canvas.create_line(self.pose[0][0], self.pose[0][1], self.pose[0][0] + M[0], self.pose[0][1] + M[1])

        self.pose[0] += M
        self.body_points = [BP[0]+M[0],BP[1]+M[1],BP[2]+M[0],BP[3]+M[1],BP[4]+M[0],BP[5]+M[1],BP[6]+M[0],BP[7]+M[1]]
        self.indic_points = [IP[0]+M[0],IP[1]+M[1],IP[2]+M[0],IP[3]+M[1],IP[4]+M[0],IP[5]+M[1],IP[6]+M[0],IP[7]+M[1]]
        self.w1_points = [W1[0] + M[0], W1[1] + M[1], W1[2] + M[0], W1[3] + M[1], W1[4] + M[0], W1[5] + M[1],
                             W1[6] + M[0], W1[7] + M[1]]
        self.w2_points = [W2[0] + M[0], W2[1] + M[1], W2[2] + M[0], W2[3] + M[1], W2[4] + M[0], W2[5] + M[1],
                             W2[6] + M[0], W2[7] + M[1]]
        self.w3_points = [W3[0] + M[0], W3[1] + M[1], W3[2] + M[0], W3[3] + M[1], W3[4] + M[0], W3[5] + M[1],
                             W3[6] + M[0], W3[7] + M[1]]
        self.w4_points = [W4[0] + M[0], W4[1] + M[1], W4[2] + M[0], W4[3] + M[1], W4[4] + M[0], W4[5] + M[1],
                             W4[6] + M[0], W4[7] + M[1]]

    def rotateCCW(self, event):
        N1, N2, N3, N4 = self.rot_helper(self.body_points,1)
        self.canvas.delete(self.body)
        self.body_points = [N1[0], N1[1], N2[0], N2[1], N3[0], N3[1], N4[0], N4[1]]
        self.body = self.canvas.create_polygon(self.body_points, fill='blue')

        N1, N2, N3, N4 = self.rot_helper(self.indic_points,1)
        self.canvas.delete(self.indic)
        self.indic_points = [N1[0], N1[1], N2[0], N2[1], N3[0], N3[1], N4[0], N4[1]]
        self.indic = self.canvas.create_polygon(self.indic_points, fill='red')

        N1, N2, N3, N4 = self.rot_helper(self.w1_points,1)
        self.canvas.delete(self.w1)
        self.w1_points = [N1[0], N1[1], N2[0], N2[1], N3[0], N3[1], N4[0], N4[1]]
        self.w1 = self.canvas.create_polygon(self.w1_points, fill='black')

        N1, N2, N3, N4 = self.rot_helper(self.w2_points,1)
        self.canvas.delete(self.w2)
        self.w2_points = [N1[0], N1[1], N2[0], N2[1], N3[0], N3[1], N4[0], N4[1]]
        self.w2 = self.canvas.create_polygon(self.w2_points, fill='black')

        N1, N2, N3, N4 = self.rot_helper(self.w3_points,1)
        self.canvas.delete(self.w3)
        self.w3_points = [N1[0], N1[1], N2[0], N2[1], N3[0], N3[1], N4[0], N4[1]]
        self.w3 = self.canvas.create_polygon(self.w3_points, fill='black')

        N1, N2, N3, N4 = self.rot_helper(self.w4_points,1)
        self.canvas.delete(self.w4)
        self.w4_points = [N1[0], N1[1], N2[0], N2[1], N3[0], N3[1], N4[0], N4[1]]
        self.w4 = self.canvas.create_polygon(self.w4_points, fill='black')

        self.pose[1] += np.array([self.rot_inc / 360 * 2 * np.pi, 0])

    def rotateCW(self, event):
        N1, N2, N3, N4 = self.rot_helper(self.body_points,-1)
        self.canvas.delete(self.body)
        self.body_points = [N1[0], N1[1], N2[0], N2[1], N3[0], N3[1], N4[0], N4[1]]
        self.body = self.canvas.create_polygon(self.body_points, fill='blue')

        N1, N2, N3, N4 = self.rot_helper(self.indic_points,-1)
        self.canvas.delete(self.indic)
        self.indic_points = [N1[0], N1[1], N2[0], N2[1], N3[0], N3[1], N4[0], N4[1]]
        self.indic = self.canvas.create_polygon(self.indic_points, fill='red')

        N1, N2, N3, N4 = self.rot_helper(self.w1_points,-1)
        self.canvas.delete(self.w1)
        self.w1_points = [N1[0], N1[1], N2[0], N2[1], N3[0], N3[1], N4[0], N4[1]]
        self.w1 = self.canvas.create_polygon(self.w1_points, fill='black')

        N1, N2, N3, N4 = self.rot_helper(self.w2_points,-1)
        self.canvas.delete(self.w2)
        self.w2_points = [N1[0], N1[1], N2[0], N2[1], N3[0], N3[1], N4[0], N4[1]]
        self.w2 = self.canvas.create_polygon(self.w2_points, fill='black')

        N1, N2, N3, N4 = self.rot_helper(self.w3_points,-1)
        self.canvas.delete(self.w3)
        self.w3_points = [N1[0], N1[1], N2[0], N2[1], N3[0], N3[1], N4[0], N4[1]]
        self.w3 = self.canvas.create_polygon(self.w3_points, fill='black')

        N1, N2, N3, N4 = self.rot_helper(self.w4_points,-1)
        self.canvas.delete(self.w4)
        self.w4_points = [N1[0], N1[1], N2[0], N2[1], N3[0], N3[1], N4[0], N4[1]]
        self.w4 = self.canvas.create_polygon(self.w4_points, fill='black')

        self.pose[1] += np.array([-self.rot_inc / 360 * 2 * np.pi, 0])


    def rot_helper(self, A, dir):
        P1 = self.rot_matrix(np.array([[A[0]-self.pose[0][0],A[1]-self.pose[0][1]]]), dir)[0]
        P2 = self.rot_matrix(np.array([[A[2]-self.pose[0][0],A[3]-self.pose[0][1]]]), dir )[0]
        P3 = self.rot_matrix(np.array([[A[4]-self.pose[0][0],A[5]-self.pose[0][1]]]), dir)[0]
        P4 = self.rot_matrix(np.array([[A[6]-self.pose[0][0],A[7]-self.pose[0][1]]]), dir)[0]

        return P1, P2, P3, P4

    def rot_matrix(self, V, dir):
        a = float(self.rot_inc/360 * 2 * np.pi)
        C = np.array([[np.cos(a),dir * np.sin(a)],[dir * -1 * np.sin(a),np.cos(a)]])
        X = np.matmul(C, V.reshape(2,-1))
        return X.reshape(1,2) + self.pose[0]