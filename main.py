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
bot = Car(my_canvas)
bot.build([float(40), float(40)])

root.bind('<Right>', bot.move)
root.bind('<Up>', bot.rotateCCW)
root.bind('<Down>', bot.rotateCW)

root.mainloop()