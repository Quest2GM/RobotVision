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

<p  align="justify">
Note that although the math to follow from here will ignore the discrepancy from the vertical flip, we must be mindful that we use the negative y-direction in all of our equations when we are actually programming this.

### PID Control
<p  align="justify">
If we want to setup a PID controller, we need two things; a variable to control and a defined path for the robot to follow. Since the velocity of our robot is constant, the only variable that needs control is our angular velocity. Also, we can define the path for the bot to follow by drawing on the Tkinter canvas itself, and the path can be stored in an array as a set of points.

<p  align="justify">
There is one problem that we need to overcome - how do we know when to command the robot to turn clockwise or counterclockwise? In reality, the robot has a colour sensor on it so it can know whether to rotate right or left depending on where it detects the dark colour in its field of view. Doing this in simulation is very different since we don't actually use sensor readings. Here we make use of the cross product!

<p  align="justify">
At each iteration of our PID control loop, we find the point on the path that is the shortest distance away from the robot, and determine the vector that goes through this point and is tangent to the path. This is our 'b' vector. Then we find the vector that describes the heading of the robot, and this is our 'a' vector. To enable the use of the cross product, we add a third dimension to our vectors. If you can visualize the right hand rule, notice that computing <img  src="https://latex.codecogs.com/gif.latex?f=|\underrightarrow{a}\times\underrightarrow{b}|" > will always be negative if the robot is to the left of the path, it will be positive if it is to the right of the path, and perfectly aligned with the path if it is zero.

<p  align="justify">
Now, our update equations for our robot pose can be derived as:

<p  align="center">
<img  src="https://latex.codecogs.com/gif.latex?\textbf{x}_{k+1}%20=%20\begin{bmatrix}%20x_k%20+%20v_k\Delta%20t\cos(\theta_k+F\omega_k\Delta%20t)%20\\%20y_k%20+%20v_k\Delta%20t\sin(\theta_k+F\omega_k\Delta%20t)%20\\%20\theta_k%20+%20F\omega_k\Delta%20t%20\end{bmatrix}%20+%20\textbf{v}_k"/>

<p  align="justify">
We define <img src="https://latex.codecogs.com/gif.latex?F=\frac{f}{|f|}"/>, which basically is the correction factor (either +1 or -1) that identifies if the robot is moving clockwise or counterclockwise.

Implementing this in code is well known:
	
	# The PID Control Loop
	error = kp * error + ki * integral + kd * derivative
	integral = error * dt
	derivative = (error - last_error) / dt
	last_error = error

We can easily modify the code block to integrate using the trapezoidal rule or the midpoint rule, as opposed to the standard Riemann sum to get better approximations of the intergral (and likewise with the derivative, but using euler forward or Runge-Kutta, etc). 

<p  align="justify">
From here, after tuning our PID gains of course, we can draw any path on the screen and the robot should be able to follow it!


### Lead-Lag Control
<p  align="justify">
I remember in class that we were taught the lead-lag controller and how it can approximate a PI or PD controller, but is much more stable since it eliminates a lot of the problems that experience with the Integral or Derivative terms. But how do we actually implement this in code? So far, I haven't found any source code online that actually allow you to implement a lead-lag controller in a way similar to the PID controller. But, while a lead-lag controller is a more compact version of the PID controller, I think I discovered why it isn't so simple to implement and I THINK I've found a way to actually implement this in code. Since a novice like myself has claimed to have discovered something in potentially well-known theoretical territory, take anything I say here with a grain of salt.

## How do I get from here to there?

### Dubin's Path Calculator
<p  align="justify">
The idea behind Dubin's path is very simple; what is the shortest distance path between any two points on a 2D grid, given your vehicle has a minimum turning radius. Given that there are only six types of curves to consider according to Dubin, the problem really comes down to circle geometry. There are two types of curves; CSC = Curve, Straight, Curve and CCC = Curve, Curve, Curve. As such, I set out to handle this graphically in Desmos before attempting to program it. I don't really understand why, but the amount of time I spent on these Desmos visualizations is probably about half in comparison to the amount of time I spent on this entire project.

[CSC Desmos](https://www.desmos.com/calculator/dqbshvxzmd), [CCC Desmos](https://www.desmos.com/calculator/xfw2mw9dti). 


## Where am I?

### Extended Kalman Filtering
<p  align="justify">
The state model, <img src="https://latex.codecogs.com/gif.latex?\textbf{x}%20=%20\begin{bmatrix}%20x%20&%20%20y%20&%20\theta%20\end{bmatrix}"/> for our system given our control input can be intuitively defined as the following:



<p  align="justify">
Since we are working in 2D, we require a 2D measurement model. Luckily, our LIDAR sensor can measure relative angle and distance, leading to the following measurement model:

<p  align="center">
<img  src="https://latex.codecogs.com/gif.latex?\textbf{z}_{k+1}%20=%20\begin{bmatrix}%20\arctan\left(\frac{y_L%20-%20y_k}{x_L-x_k}\right)%20-%20\theta_k%20\\%20\sqrt{(x_L-x_k)^2%20+%20(y_L-y_k)^2}%20\end{bmatrix}%20+%20\textbf{w}_k"/>

<p  align="justify">
Note that in implementation we use the atan2 function instead of arctan to account for directionality. Given this, the Jacobian matrices with respect to state space can be derived:


### Simulatenous Localization and Mapping (SLAM)

### Unscented Kalman Filtering
Since we have a highly non-linear model, the EKF tends to linearize in unwanted places and in general, the Jacobian matrix tends to under approximate the true state of the robot. As such, it only makes sense to explore the UKF since it eliminates the need for Jacobians and is almost always used in practice.
  
  
## What's Next?

### Progress
-  [x] Dubin's Path Calculator
- [ ] Rapidly Exploring Random Trees Algorithm
-  [x] PID Control
-  [x] Lead-Lag Control
-  [x] Kalman Filtering (Extended, Unscented)
- [ ] Particle Filtering
-  [x] SLAM