from tkinter import *

width, height = 1400, 1000

root = Tk()
root.title('RobotVision')
root.geometry('500x500')

my_canvas = Canvas(root, width=width, height=height, bg='white')
my_canvas.pack(padx=2, pady=2)


r1 = my_canvas.create_rectangle(width/2-12, height/2-12, width/2-4, height/2-6, fill='black')
r2 = my_canvas.create_rectangle(width/2-12, height/2+12, width/2-4, height/2+6, fill='black')
r3 = my_canvas.create_rectangle(width/2+12, height/2+12, width/2+4, height/2+6, fill='black')
r4 = my_canvas.create_rectangle(width/2+12, height/2-12, width/2+4, height/2-6, fill='black')
r5 = my_canvas.create_rectangle(width/2-10, height/2-10, width/2+10, height/2+10, fill='blue')
r6 = my_canvas.create_rectangle(width/2, height/2-8, width/2+8, height/2+8, fill='red')


def rot_cw(event):
    my_canvas.move(r1, 10, 0)
    my_canvas.move(r2, 10, 0)
    my_canvas.move(r3, 10, 0)
    my_canvas.move(r4, 10, 0)
    my_canvas.move(r5, 10, 0)
    my_canvas.move(r6, 10, 0)

def rot_ccw(event):
    my_canvas.move(r1, 0, 10)
    my_canvas.move(r2, 0, 10)
    my_canvas.move(r3, 0, 10)
    my_canvas.move(r4, 0, 10)
    my_canvas.move(r5, 0, 10)
    my_canvas.move(r6, 0, 10)

root.bind('<Left>', rot_cw)
root.bind('<Right>', rot_ccw)


root.mainloop()

