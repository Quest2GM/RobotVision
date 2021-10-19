import numpy as np

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
