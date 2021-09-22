from tkinter import *
from car import Car
import time
import numpy as np

# Global Vars
sec_pass, inc, radius = 0, 0, 2.5

speed = 25


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

def auto_move():
    
    global sec_pass, inc

    if inc < speed * 100:
        bot.move()
        bot.rotateCW(r=radius)
        sec_pass += 1000/speed
        inc += 1
        l.config(text=str(float(sec_pass/1000)))
        root.after(int(1000/speed), auto_move)

# Create TurtleBot
bot = Car(c)
bot.build([float(w/2), float(h/2)])
c.bind('<Configure>', create_grid)

# Add buttons and labels
b = Button(root, text='Move', command=auto_move, font=('Helvetica', 24), fg='red')
b.pack(side='right')
l = Label(root, text = '0', fg='red')
l.pack(side='top')



# root.bind('<Up>', bot.rotateCCW)
# root.bind('<Down>', bot.rotateCW)

root.mainloop()
