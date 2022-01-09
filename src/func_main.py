import numpy as np
from numpy import linalg as la

# Canvas properties
w, h = 1400, 900

# Converts grid coordinates to pixel coordinates and vice-versa
def grid_2_pixel(x, y):
    return (25*x + w/2, -25*y + h/2)

def pixel_2_grid(x, y):
    return ((1/25) * (x-w/2), (-1/25) * (y-h/2))

# Radians to degrees and vice-versa
def rad_2_deg(a):
    return a / (2 * np.pi) * 360

def deg_2_rad(a):
    return a / 360 * (2 * np.pi)

# Calculate Distance
def dist(x1, y1, x2, y2):
    return np.sqrt(np.power(x2-x1,2) + np.power(y2-y1,2))

# Convert angles between -pi and pi
def range_2_pi(val):
    return np.where(val > np.pi, val - 2*np.pi, val)


# Finds nearest positive definite matrix for UKF algorithm
def nearestPD(A):
    """Find the nearest positive-definite matrix to input
    A Python/Numpy port of John D'Errico's `nearestSPD` MATLAB code [1], which
    credits [2].
    [1] https://www.mathworks.com/matlabcentral/fileexchange/42885-nearestspd
    [2] N.J. Higham, "Computing a nearest symmetric positive semidefinite
    matrix" (1988): https://doi.org/10.1016/0024-3795(88)90223-6
    """

    B = (A + A.T) / 2
    _, s, V = la.svd(B)

    H = np.dot(V.T, np.dot(np.diag(s), V))

    A2 = (B + H) / 2

    A3 = (A2 + A2.T) / 2

    if isPD(A3):
        return A3

    spacing = np.spacing(la.norm(A))
    I = np.eye(A.shape[0])
    k = 1
    while not isPD(A3):
        mineig = np.min(np.real(la.eigvals(A3)))
        A3 += I * (-mineig * k**2 + spacing)
        k += 1

    return A3

def isPD(B):
    """Returns true when input is positive-definite, via Cholesky"""
    try:
        _ = la.cholesky(B)
        return True
    except la.LinAlgError:
        return False