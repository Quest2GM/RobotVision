# RobotVision

<p  align="justify">
The goal of RobotVision is to simulate control, path-planning, localization and mapping of a non-holonomic robot in a 2D world. The topics explored here are well known in robotics but I figured that this is a good opportunity for me to explore them in detail. Here, I attempt to provide the mathematical framework behind each algorithm, just enough to setup each problem in robot's reference frame, while explaining some interesting discoveries I made along the way.

<p  align="justify">
The project was built from scratch using Python's Tkinter library. I found this the most versatile library in Python to create animations and it very much exceeded expectations.

<p  align="justify">
As a side note, I adopt Prof. Gabriele D'Eleuterio's math symbol scheme for clarity and stylistic elegance.


## What am I?
<p  align="justify">
I am a robot made of six rectangles; one body, one head and four wheels. This is what I look like.

<p  align="justify">
I can move forward and rotate about a point that is not my centroid, but not in reverse. With these maneuvers, I can traverse the entire 2D world. But I can't do it on my own; I only understand how to follow dark coloured markers on the ground.

<p  align="justify">
I am equipped with a LIDAR sensor, one that can measure distance to objects and the relative angle of myself to them, as well as a colour sensor that tells me how close I am to the dark markers.

<p  align="justify">
Let's help the robot achieve control, path planning, localization and mapping.

## How do I move?

### Defining the Robot's Model

<p  align="justify">
The vehicle has three manuevers; rotate clockwise, rotate counter-clockwise and drive straight. Thus, the most logical control input <img  src="https://latex.codecogs.com/gif.latex?u=\begin{bmatrix}%20v%20&%20\omega%20\end{bmatrix}"  /> is a vector with velocity, <img  src="https://latex.codecogs.com/gif.latex?v"  />, and angular velocity, <img  src="https://latex.codecogs.com/gif.latex?\omega"  />.

<p  align="justify">
The trick to achieving angular control when the vehicle's foreign metric system is in pixels is in the rotation matrix. The Tkinter grid is positioned such that positive is to the right and down, whereas the conventional form is to the right and up. Thus, the conversion to the pixel reference frame comes down to a vertical flip. From this, the CW and CCW 2D rotation matrices can be deduced as:

<p  align="center">
	<img  src="https://latex.codecogs.com/gif.latex?\textbf{C}_{cw}=\begin{bmatrix}%20\cos{\theta}%20&%20-%20%20\sin{\theta}\\%20%20\sin{\theta}%20&%20\cos{\theta}%20\end{bmatrix}"/>  &nbsp; &nbsp; &nbsp;
	<img  src="https://latex.codecogs.com/gif.latex?\textbf{C}_{ccw}=\begin{bmatrix}%20\cos{\theta}%20&%20\sin{\theta}\\%20%20-\sin{\theta}%20&%20\cos{\theta}%20\end{bmatrix}"/>

### PID Control
You can basically draw a path for the robot to follow and the robot will follow it. The PID constants are tunable, and the error is computed as the straight line distance between the robot and the closest point to the path.


### Lead-Lag Control



## How do I get from here to there?

### Dubin's Path Calculator
<p  align="justify">
The idea behind Dubin's path is very simple; what is the shortest distance path between any two points on a 2D grid, given your vehicle has a minimum turning radius. Given that there are only six types of curves to consider according to Dubin, the problem really comes down to circle geometry. There are two types of curves; CSC = Curve, Straight, Curve and CCC = Curve, Curve, Curve. As such, I set out to handle this graphically in Desmos before attempting to program it. I don't really understand why, but the amount of time I spent on these Desmos visualizations is probably about half in comparison to the amount of time I spent on this entire project.

[CSC Desmos](https://www.desmos.com/calculator/dqbshvxzmd), [CCC Desmos](https://www.desmos.com/calculator/xfw2mw9dti). 


## Where am I?

### Extended Kalman Filtering


### Unscented Kalman Filtering

### Simulatenous Localization and Mapping (SLAM)
  
  





## Progress
-  [x] Dubin's Path Calculator
- [ ] Rapidly Exploring Random Tree
-  [x] PID Control
-  [x] Lead-Lag Control
-  [x] Kalman Filtering (Extended, Unscented)
- [ ] Particle Filtering
-  [x] SLAM