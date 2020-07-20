# Instruction to use the proxy :
Requirements:
Python3 environment that has the geometry and geometry2 package ros correctly installed 
- library : 
cvxpy
sympy 
numpy
math

#How to use:
- launch a GAZEBO simulation with a map of a robot 
- launch a navigation algorithm in order to provide the robot a map_metadata topic (amcl_demo for example)
- launch the proxy that will listen to the initial pose and goal that you can provide for example in rviz

#Details:
the algoritmh is a service that can be called with an EMPTY srv by the global planner plugin.
this service will receive the request, will subcrive the initial position and the goal position and then calculate the trajectory by the convex optimization method.
THe parameters that can be setted are the number of Successive approssimation of convex optimization and the resolution of the path.
It can be selected also the optimization parameters by selecting the boolean parameters of the function, the velocity of convergence and the power consumption optimization
