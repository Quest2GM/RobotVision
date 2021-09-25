import numpy as np

# Converts grid coordinates to pixel coordinates and vice-versa
def grid_2_pixel(x, y):
    return (25*x + w/2, -25*y + h/2)
    
def pixel_2_grid(x, y):
    return ((1/25) * (x-w/2), (-1/25) * (y-h/2))