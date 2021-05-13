import turtle

bot = turtle.Turtle()
botsc = bot.getscreen()
bot.shape("square")
bot.color("blue")
botsc.bgcolor("white")


def move():
    bot.forward(20)


botsc.onkey(move, "space")
botsc.listen()

turtle.done()