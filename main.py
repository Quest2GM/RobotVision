from tkinter import *
from class_main import *
from func_main import *
import numpy as np
from scipy.spatial.distance import cdist

# Global Vars
sec_pass, inc, radius = 0, 0, 1
speed = 25
path_arr = []
error = 0

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
        bot.rotateCW(r=radius)
        sec_pass += 1000/speed
        inc += 1
        l.config(text='Simulation Time (s): ' + str(float(sec_pass/1000)))
        root.after(int(1000/speed), auto_move)

def PID_move():

    global path_arr, bot, sec_pass, inc, error

    if np.abs(error) < 200:

        # Calculate minimum distance        
        dists = cdist([bot.pos], path_arr, 'euclidean')[0]
        ind_min = np.argmin(dists)
        min_dist_point = path_arr[ind_min]
        min_dist_point_2 = path_arr[ind_min + 2]

        # Convert points into grid coordinates
        bp = pixel_2_grid(*bot.pos)
        md = pixel_2_grid(*min_dist_point)

        # Compute the vectors that join the closest point and the bot, and the direction of the bot
        bot_line_pos = list([bp[0] - md[0], bp[1] - md[1]])
        bot_dir_vec = [np.cos(bot.angle), np.sin(bot.angle)]

        # Compute cross-product of two vectors to determine whether the bot should rotate CW or CCW to follow path
        error = np.cross(bot_line_pos + [0], bot_dir_vec + [0])[2]

        # Rotate depending on sign of error
        if error > 0:
            bot.rotateCCW(r=1.5)
        elif error < 0:
            bot.rotateCW(r=1.5)

        bot.move()

        # Update simulation time on screen
        sec_pass += 1000/speed
        inc += 1
        l.config(text='Simulation Time (s): ' + str(float(sec_pass/1000)))
        root.after(int(1000/speed), PID_move)



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
bot.build([float(w/2)+250, float(h/2)-250])
c.bind('<Configure>', create_grid)
c.bind('<Button-1>', get_x_and_y)
c.bind('<B1-Motion>', draw_path)

# Add buttons and labels
b = Button(root, text='Move', command=PID_move, font=('Helvetica', 16), fg='black')
b.place(x=58, y=10)
l = Label(root, text='Simulation Time (s): 0', fg='black')
l.place(x=58, y=60)

# d = Dubin(c)
# d.create_paths()

root.mainloop()

print(len(path_arr))
