# RobotVision

![](images/Robot_Vision_logo.gif)

The goal of RobotVision is to be able to simulate the control of a robot. The app will be a combination of many aspects of control theory, filtering and Bayesian techniques and will feature a supposed 'TurtleBot' trying to navigate through obstacles in a stable and efficient manner. The experience in building this app will have great implications on getting a strong grasp of very powerful modern robotic controllers, and may give rise to new discoveries.

The project will be prototyped in Python and will be later converted to C++. For this reason, the prototype will be written from scratch with minimal use of complex libraries. It is also planned to first start with a 2D simulator, and later on move to 3D through the help of OpenGL.

## PID Control
![](images/pid_ctrl_demo.gif)

You can basically draw a path for the robot to follow and the robot will follow it. The PID constants are tunable, and the  error is computed as the straight line distance between the robot and the closest point to the path.

# Progress
- [x] Dubin's Path calculator
- [x] PID Control
- [ ] Lead-Lag Control
- [ ] Kalman Filtering (Normal, Extended, Unscented)
- [ ] Bayesian Localization
- [ ] SLAM
