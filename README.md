# RobotVision

<p  align="justify">
The goal of RobotVision is to simulate control, path-planning, localization and mapping of a non-holonomic robot in a 2D world. The topics explored here are well known in the robotics industry but I figured that this is a good opportunity explore them myself in more detail. Here, I attempt to provide the mathematical framework behind RobotVision.

<p  align="justify">
The project was built from scratch using Python's Tkinter library. I found this the most versatile library in Python to create animations and it very much exceeded expectations.

<p  align="justify">
As a side note, I adopt Prof. Gabriele D'Eleuterio's math symbol scheme for consistency and stylistic elegance.


## What am I?
<p  align="justify">
I am a robot made of six rectangles; one body, one head and four wheels. This is what I look like.

<p  align="center">
![Bot](images/bot.png)

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
The trick to achieving angular control when the vehicle's foreign metric system is in pixels is in the rotation matrix. The Tkinter grid is positioned such that positive is to the right and down, whereas the conventional form is to the right and up. Thus, the conversion to the pixel reference frame comes down to a vertical flip. The CW and CCW 2D rotation matrices can be deduced as:

<p  align="center">
	<img  src="https://latex.codecogs.com/gif.latex?\textbf{C}_{cw}=\begin{bmatrix}%20\cos{\theta}%20&%20-%20%20\sin{\theta}\\%20%20\sin{\theta}%20&%20\cos{\theta}%20\end{bmatrix}"/>  &nbsp; &nbsp; &nbsp;
	<img  src="https://latex.codecogs.com/gif.latex?\textbf{C}_{ccw}=\begin{bmatrix}%20\cos{\theta}%20&%20\sin{\theta}\\%20%20-\sin{\theta}%20&%20\cos{\theta}%20\end{bmatrix}"/>

<p  align="justify">
These rotation matrices can be applied to each of the "six rectangles" that the robot composes of to achieve rotation! Note that although the math to follow from here will ignore the discrepancy from the vertical flip, we must be mindful that we use the negative y-direction in all of our equations when we are actually programming this.

### PID Control
<p  align="justify">
If we want to setup a PID controller, we need two things; a variable to control and a defined path for the robot to follow. Since the velocity of our robot is constant, the only variable that needs control is our angular velocity. Also, we can define the path for the bot to follow by drawing on the Tkinter canvas itself, and the path can be stored in an array as a set of points.

<p  align="justify">
There is one problem that we need to overcome - how do we know when to command the robot to turn clockwise or counterclockwise? In reality, the robot has a colour sensor on it so it can know whether to rotate right or left depending on where it detects the dark colour in its field of view. Doing this in simulation is very different since we don't actually use sensor readings. Here we make use of the cross product!

<p  align="center">
![PID_cross_prod](images/pid_cross_product.png)

<p  align="justify">
At each iteration of our PID control loop, we find the point on the path that is the shortest distance away from the robot, and determine the vector that goes through this point and is tangent to the path. This is our 'b' vector. Then we find the vector that describes the heading of the robot, and this is our 'a' vector. To enable the use of the cross product, we add a third dimension to our vectors. If you can visualize the right hand rule, notice that computing <img  src="https://latex.codecogs.com/gif.latex?f=|\underrightarrow{a}\times\underrightarrow{b}|" > will always be negative if the robot is to the left of the path, positive if it is to the right of the path, and zero if it is perfectly aligned with the path.

<p  align="justify">
The state model with state, <img src="https://latex.codecogs.com/gif.latex?\textbf{x}%20=%20\begin{bmatrix}%20x%20&%20%20y%20&%20\theta%20\end{bmatrix}"/>, for our system given our control input can be defined as the following:

<p  align="center">
<img  src="https://latex.codecogs.com/gif.latex?\textbf{x}_{k+1}%20=%20\begin{bmatrix}%20x_k%20+%20v_k\Delta%20t\cos(\theta_k+F\omega_k\Delta%20t)%20\\%20y_k%20+%20v_k\Delta%20t\sin(\theta_k+F\omega_k\Delta%20t)%20\\%20\theta_k%20+%20F\omega_k\Delta%20t%20\end{bmatrix}%20"/>

<p  align="justify">
We define <img src="https://latex.codecogs.com/gif.latex?F=\frac{f}{|f|}"/>, which basically is the correction factor (either +1 or -1) that identifies if the robot is moving clockwise or counterclockwise.

Implementing this in code is well known:
	
```python
# The PID Control Loop
error = kp * error + ki * integral + kd * derivative
integral += error * dt
derivative = (error - last_error) / dt
last_error = error
```
<p  align="justify">
We can easily modify the code block to integrate using the trapezoidal rule or the midpoint rule, as opposed to the standard Riemann sum to get better approximations of the integral (and likewise with the derivative, but using euler forward or Runge-Kutta, etc). 

<p  align="justify">
From here, after tuning our PID gains sufficiently enough of course, we can draw any path on the screen and the robot should be able to follow it!

<p  align="center">
<a href="https://www.youtube.com/embed/df07yQ8M38M">
         <img alt="PID" src="images/pid_thumbnail.png">
</a>

### Lead-Lag Control
<p  align="justify">
I remember in my Control Systems class that we were taught the lead-lag controller and how it can approximate a PI or PD controller, but is much more stable since it eliminates a lot of the problems that experience with the Integral or Derivative terms. But I never really understood how to actually implement this in code. So far, I haven't found any source code online that actually allow you to implement a lead-lag controller in a way similar to the PID controller. But, while a lead-lag controller is a more compact version of the PID controller, I think I understand why it isn't so simple and I THINK I've found a way to actually implement this in code. Since a novice like myself has claimed to have discovered something in potentially well-known theoretical territory, take anything I say in this section with a grain of salt.

<p  align="justify">
The PID controller can be described in the Laplace domain as:

<p  align="center">
<img  src="https://latex.codecogs.com/gif.latex?U(s)%20=%20E(s)%20\left(k_p%20+%20\frac{k_I}{s}%20+%20sk_D\right)"/>

<p  align="justify">
The <img  src="https://latex.codecogs.com/gif.latex?\frac{k_I}{s}E(s)"/> term means that we are integrating over the error, and this is made clear once we convert this back to the time domain using our Laplace transform tables. But let's do this in a slightly different way, using the convolution theorem. It tells us:

<p  align="center">
<img  src="https://latex.codecogs.com/gif.latex?f(t)%20*%20g(t)%20=%20\int_0^t%20f(\tau)g(t-\tau)%20d\tau%20\Rightarrow%20\mathcal{L}\{f(t)*g(t)\}%20=%20F(s)G(s)=Y(s)" />

In the case of our PID integral term, lets set <img  src="https://latex.codecogs.com/gif.latex?F(s)=E(s)"/> and <img  src="https://latex.codecogs.com/gif.latex?G(s)=\frac{1}{s}"/>. We also know that <img  src="https://latex.codecogs.com/gif.latex?\mathcal{L}^{-1}\{E(s)\}=f(t)=e(t)"/> and <img  src="https://latex.codecogs.com/gif.latex?\mathcal{L}^{-1}{\frac{1}{s}}=g(t)=g(t-\tau)=1"/>. This leads to the following equation for our output:

<p  align="center">
<img  src="https://latex.codecogs.com/gif.latex?y(t)=f(t)%20*%20g(t)%20=%20\int_0^t%20e(\tau)%20d\tau%20\approx%20\sum_{t}e_t\Delta%20t" />

<p  align="justify">
Thus, all we need to do for our integral term is sum up the error over all time. As you may have noticed from the PID code block earlier, this is precisely why we continue to add the current integral term onto our previous integral term.

<p  align="justify">
Let's try to apply the same idea to our lead lag controller, which can be described by the following in the Laplace domain:

<p  align="center">
<img  src="https://latex.codecogs.com/gif.latex?U(s)%20=%20E(s)%20\frac{K(s+a)}{s+b}%20=%20E(s)%20\left(\frac{K(a-b)}{s+b}%20+%20K\right)" />

<p  align="justify">
This time, the <img  src="https://latex.codecogs.com/gif.latex?KE(s)"/> term is our proportional controller, and <img  src="https://latex.codecogs.com/gif.latex?E(s)%20\frac{K(a-b)}{s+b}" /> becomes our integral controller. Now, <img  src="https://latex.codecogs.com/gif.latex?F(s)=E(s)"/> and <img  src="https://latex.codecogs.com/gif.latex?G(s)=\frac{K(a-b)}{s+b}"/>, and hence, <img  src="https://latex.codecogs.com/gif.latex?\mathcal{L}^{-1}\{E(s)\}=f(t)=e(t)"/> and <img  src="https://latex.codecogs.com/gif.latex?\mathcal{L}^{-1}{\frac{K(a-b)}{s+b}}=g(t)=K(a-b)\exp(-bt)"/> where <img  src="https://latex.codecogs.com/gif.latex?t>0"/>. This leads to the following equation for our output:

<p  align="center">
<img  src="https://latex.codecogs.com/gif.latex?y(t)=f(t)%20*%20g(t)%20=%20\int_0^t%20e(\tau)K(a-b)\exp(-b(t-\tau))%20d\tau"  />
<img  src="https://latex.codecogs.com/gif.latex?\approx%20K(a-b)\sum_{t}e_t\exp(-b(t-\tau))\Delta%20t"  />

<p  align="justify">
But, as you may have noticed, there's a small problem. Because we are multiplying two functions now and we are also time shifting g(t), the past multiplication terms will also change! This wasn't a problem earlier because in the PID case we had g(t) = g(t-tau) = 1 and we didn't have to consider this. Now, instead of adding an integral term continuously, we need to keep a continuous history of all errors and integral term calculations. Now, we can implement this in code:

```python
# The Lead Lag Control loop
error = K * (error +  integral)
times = [times[0] + dt] + times
exp_list = (a-b) * np.exp(-b * times)
err_list += [error]
integral = np.sum(exp_list * err_list) * dt
```
<p  align="justify">
A potential issue with this implementation is that our space complexity becomes infinite as t -> infinity. To solve this, since we are dealing with a decaying exponential, the leading terms in np.sum(exp_list * err_list) approach zero for large t. We could therefore just cap our array size to some finite number (can be tuned depending on how quickly the exponential decays) and it should still give us a good approximation.

<p  align="center">
<a href="https://www.youtube.com/embed/GudSzbR40U0">
         <img alt="LL" src="images/ll_thumbnail.png">
</a>

<p  align="justify">
More investigation into how to properly tune the gains for the lead-lag controller needs to be done.

## How do I get from here to there?

### Dubin's Path Calculator
<p  align="justify">
The idea behind Dubin's path is very simple; what is the shortest distance path between any two points on a 2D grid, given your vehicle has a minimum turning radius. Given that there are only six types of curves to consider according to Dubin, the problem really comes down to circle geometry. There are two types of curves; CSC = Curve, Straight, Curve and CCC = Curve, Curve, Curve. I set out to handle this graphically in Desmos before attempting to program it. I don't really understand why, but the amount of time I spent on these Desmos visualizations is probably about half in comparison to the amount of time I spent on this entire project.

<p  align="center">
<a href="https://www.desmos.com/calculator/dqbshvxzmd">
         <img alt="CSC" src="images/csc_thumbnail.png">
</a> <a href="https://www.desmos.com/calculator/xfw2mw9dti">
         <img alt="CCC" src="images/ccc_thumbnail.png">
</a>

I implemented this in three steps:
1. Draw all "pseudo-legal" Dubin's paths (ones do not consider vehicle's initial orientation) between start and end point.
2. Eliminate illegal paths.
3. Compute distance for all paths and identify minimum.

### Rapidly Exploring Random Trees
TBD

## Where am I?

### Extended Kalman Filter (EKF)
<p  align="justify">
As defined in the PID control section, our state update equation can be given as the following, this time with a state Gaussian noise term:

<p  align="center">
<img  src="https://latex.codecogs.com/gif.latex?f(\textbf{x}_k,\textbf{u}_k)%20=%20\begin{bmatrix}%20x_k%20+%20v_k\Delta%20t\cos(\theta_k+F\omega_k\Delta%20t)%20\\%20y_k%20+%20v_k\Delta%20t\sin(\theta_k+F\omega_k\Delta%20t)%20\\%20\theta_k%20+%20F\omega_k\Delta%20t%20\end{bmatrix}%20+%20\textbf{v}_k"/>

<p  align="justify">
Since we are working in 2D, we require a 2D measurement model. Luckily, our LIDAR sensor can measure relative angle and distance, leading to the following measurement model:

<p  align="center">
<img  src="https://latex.codecogs.com/gif.latex?h(\textbf{x}_k)%20=%20\begin{bmatrix}%20\arctan\left(\frac{y_L%20-%20y_k}{x_L-x_k}\right)%20-%20\theta_k%20\\%20\sqrt{(x_L-x_k)^2%20+%20(y_L-y_k)^2}%20\end{bmatrix}%20+%20\textbf{w}_k"/>

<p  align="justify">
where <img  src="https://latex.codecogs.com/gif.latex?(x_L,y_L)"/>  is the position of a LIDAR detectable object. Note that in implementation we use the atan2 function instead of atan to account for directionality. Given this, the Jacobian matrices with respect to state space and measurement space can be derived:

<p  align="center">
<img src="https://latex.codecogs.com/gif.latex?\textbf{A}_k%20=%20\begin{bmatrix}%201%20&%200%20&%20-v_k\Delta%20t\sin(\theta_k+F\omega_k\Delta%20t)%20%20\\%200%20&%201%20&%20v_k\Delta%20t\cos(\theta_k+F\omega_k\Delta%20t)%20\\%200%20&%200%20&%201%20\end{bmatrix}" />

<p  align="center">
<img src="https://latex.codecogs.com/gif.latex?\textbf{D}_k%20=%20\begin{bmatrix}%20\frac{y_L-y_k}{d^2}%20&%20\frac{x_k-x_L}{d^2}%20&%20-1%20%20\\%20\frac{x_k-x_L}{d}%20&%20\frac{y_k-y_L}{d}%20&%200%20\end{bmatrix}%20\hspace{0.4cm}\text{where}\hspace{0.4cm}%20d%20=%20\sqrt{(x_L-x_k)^2+(y_L-y_k)^2}" />

<p  align="justify">
Finally, given an initial state covariance, <img src="https://latex.codecogs.com/gif.latex?\textbf{P}_{k|k}"/>, process noise, <img src="https://latex.codecogs.com/gif.latex?\textbf{Q}_{k}"/>, and measurement noise, <img src="https://latex.codecogs.com/gif.latex?\textbf{R}_{k}"/>, we can apply the EKF algorithm:

<p  align="center">
![EKF_alg](images/ekf_alg.png)

<p  align="center">
<a href="https://www.youtube.com/embed/xIxnYnVbS8k">
         <img alt="EKF" src="images/ekf_thumbnail.png">
</a>

<p  align="justify">
In the demo, the red circle represents a LIDAR detectable station, the black circular outline represents the detectable range of the station and the cyan trace is the predicted state. When the robot is outside of the detectable range, it cannot update the state with the Kalman gain since it doesn't receive any sensor readings, and so it predicts normally with <img  src="https://latex.codecogs.com/gif.latex?f(\textbf{x}_k,\textbf{u}_k)"/> without accounting for error. Notice how, when the robot enters the detectable range, it very visibly starts correcting the state and it eventually converges quite amazingly at the end, despite large deviations during traversal outside of the detectable range.

### Simultaneous Localization and Mapping (SLAM)

<p  align="justify">
The conversion from EKF to SLAM isn't a difficult one, so long as we are continuing to filter and not smooth. Pose graph optimization based SLAM, falling under the smoothing category, is more widely used in practice because it is more robust and less error prone to the limitations of the Kalman filter. Regardless, I decided to implement EKF based SLAM, because understanding it will help me grasp the fundamentals, and more importantly, better appreciate the need for something better.

<p  align="justify">
The biggest difference in implementation from the EKF is the fact that we need to additionally track the positions of the objects we observe. In the case of a robot exploring an unknown environment, we wouldn't know the number of objects to keep track of beforehand and so we would need to constantly update the sizes of our matrices. For simplicity, let's assume that we know the number of objects that require tracking in our map. We need to keep track of 2N additional things - the x and y coordinates for each of our N objects. Our conversion matrix from our EKF state space to this higher order space can be defined as the following:

<p  align="center">
<img src="https://latex.codecogs.com/gif.latex?\textbf{F}_x%20=%20\overbrace{\begin{bmatrix}%201%20&%200%20&%200%20&%200%20&%20\dots%20&%200\\%200%20&%201%20&%200%20&%200%20&%20\dots%20&%200\\%200%20&%200%20&%201%20&%200%20&%20\dots%20&%200%20\end{bmatrix}}^\text{2N+3}" />

<p  align="justify">
Next, we need some way of defining <img src="https://latex.codecogs.com/gif.latex?\textbf{D}_{k}"/>, but only to allow updates for objects in our current field of view. As such, we define the following matrices:

<p  align="center">
<img src="https://latex.codecogs.com/gif.latex?\textbf{D}_{low}%20=%20\begin{bmatrix}%20\frac{y_L-y_k}{d^2}%20&%20\frac{x_k-x_L}{d^2}%20&%20-1%20&%20%20\frac{y_k-y_L}{d^2}%20&%20\frac{x_L-x_k}{d^2}%20\\%20\frac{x_k-x_L}{d}%20&%20\frac{y_k-y_L}{d}%20&%200%20&%20\frac{x_L-x_k}{d}%20&%20\frac{y_L-y_k}{d}%20\end{bmatrix}%20\hspace{1cm}%20\textbf{F}_D%20=%20\overbrace{\begin{bmatrix}%201%20&%200%20&%200%20&%200%20&%20\dots%20&%200%20&%200%20&%20\dots%20&%200%20\\%200%20&%201%20&%200%20&%200%20&%20\dots%20&%200%20&%200%20&%20\dots%20&%200%20\\%200%20&%200%20&%201%20&%200%20&%20\dots%20&%200%20&%200%20&%20\dots%20&%200%20\\%200%20&%200%20&%200%20&%200%20&%20\dots%20&%201%20&%200%20&%20\dots%20&%200%20\\%200%20&%200%20&%200%20&%200%20&%20\dots%20&%200%20&%201%20&%20\dots%20&%200%20\end{bmatrix}}^\text{2N+3}" />

This is enough to implement EKF-SLAM:

<p  align="center">
![SLAM_alg](images/slam_alg.png)

<p  align="center">
<a href="https://www.youtube.com/embed/o0ACDtnDxwk">
         <img alt="SLAM" src="images/slam_thumbnail.png">
</a>

<p  align="justify">
In the demo, the cyan trace represent the predicted state and the blue dots represent the predicted state of the landmarks. Despite large errors towards the end caused by no measurement update, the robot is able to quickly localize and converge on the landmark locations with reasonable accuracy.


### Unscented Kalman Filter (UKF)

<p  align="justify">
Since we have a highly non-linear model, the EKF tends to linearize in unwanted places and in general, the Jacobian matrix tends to under approximate the true state of the robot. As such, it only makes sense to explore the UKF since it eliminates the need for Jacobians and is almost always beats the EKF.

Here is the algorithm:

<p  align="center">
![UKF_alg](images/ukf_alg.png)


<p  align="justify">
There are still one or two bugs that need to be fixed with my current implementation, but I hope to have a demo out soon!

### Particle Filter
TBD
  
  
## What's Next?

I've put the remaining items on our list on hold for now, and I may come back to completing them in the future.

### Progress
- [x] Dubin's Path Calculator
- [ ] Rapidly Exploring Random Trees Algorithm
- [x] PID Control
- [x] Lead-Lag Control
- [x] Kalman Filter (Extended, Unscented)
- [ ] Particle Filter
- [x] SLAM

## Acknowledgements
- Professor Gabriele D'Eleuterio: ROB301 lectures and primers
- Professor Cyrill Stachniss: YouTube series on Kalman Filtering and SLAM
- Professor Angela Shoellig: ROB310 lectures and class demos