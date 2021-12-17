# RobotVision

The goal of RobotVision is to simulate path-planning, control, localization and mapping of a non-holonomic robot in a 2D world. The topics explored here are well known in the robotics industry but I figured that this is a good opportunity for me to explore them in detail. Here, I attempt to provide the mathematical framework behind each algorithm, just enough to setup each problem in robot's reference frame, while explaining some interesting discoveries I made along the way.

The project was built from scratch using Python's Tkinter library. I found this the most versatile library in Python to create animations and it very much exceeded expectations.

As a side note, I adopt Prof. Gabriele D'Eleuterio's math symbol scheme for clarity and stylistic elegance.

## How do I move?

### Defining the Car's model
The vehicle that we will work with has three notable manuevers; rotate clockwise, rotate counter-clockwise and drive straight. Thus, the most logical control input <img src="https://latex.codecogs.com/gif.latex?u=\begin{bmatrix}%20v%20&%20\omega%20\end{bmatrix}" />  is a vector with velocity, <img src="https://latex.codecogs.com/gif.latex?v" />, and angular velocity, <img src="https://latex.codecogs.com/gif.latex?\omega" />. For simplicity, we will traverse with constant velocity (25 pixels/sec = 1 unit/sec).

The trick to achieving angular control when the car's foreign metric system is in pixels is in the rotation matrix. The Tkinter grid is positioned such that positive is to the right and down, whereas the conventional form is to the right and up. Thus, the conversion to the pixel reference frame comes down to a vertical flip. From this, the CW and CCW 2D rotation matrices can be deduced as:

<img src="https://latex.codecogs.com/gif.latex?\textbf{C}_{cw}=\begin{bmatrix}%20\cos{\theta}%20&%20-%20%20\sin{\theta}\\%20%20\sin{\theta}%20&%20\cos{\theta}%20\end{bmatrix}" />                    <img src="https://latex.codecogs.com/gif.latex?\textbf{C}_{ccw}=\begin{bmatrix}%20\cos{\theta}%20&%20\sin{\theta}\\%20%20-\sin{\theta}%20&%20\cos{\theta}%20\end{bmatrix}" />


### Dubin's Path Calculator

The first thing I wanted to do was build a Dubin's path calculator. The idea behind Dubin's path is very simple; what is the shortest distance path between any two points on a 2D grid, given your vehicle has a minimum turning radius. Given that there are only six types of curves to consider according to Dubin, the problem really comes down to circle geometry. There are two types of curves; CSC = Curve, Straight, Curve and CCC = Curve, Curve, Curve. As such, I set out to handle this graphically in Desmos before attempting to program it; [CSC](https://www.desmos.com/calculator/dqbshvxzmd), [CCC](https://www.desmos.com/calculator/xfw2mw9dti). I don't really understand why, but the amount of time I spent on these Desmos visualizations is probably about half in comparison to the amount of time I spent on this entire project.



## PID Control
![](images/pid_ctrl_demo.gif)

You can basically draw a path for the robot to follow and the robot will follow it. The PID constants are tunable, and the  error is computed as the straight line distance between the robot and the closest point to the path.

## Progress
- [x] Dubin's Path calculator
- [x] PID Control
- [x] Lead-Lag Control
- [x] Kalman Filtering (Normal, Extended, Unscented)
- [x] SLAM
