from tkinter import *
from car import Car

# Define Canvas and its Properties
width, height = 1400, 900
root = Tk()
root.title('RobotVision')
root.geometry(str(width) + 'x' + str(height))
my_canvas = Canvas(root, width=width, height=height, bg='white')
my_canvas.pack(padx=2, pady=2)

# Create TurtleBot
bot = Car(root, my_canvas)
bot.build((width/2, height/2))

root.bind('<Right>', bot.move)
root.bind('<Up>', bot.rotate)

root.mainloop()

