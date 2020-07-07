# Instruction to use the proxy :
Requirements:
Python3 environment that works well
- library : 
cvxpy
sympy 
numpy
math

How to use:
- launch a GAZEBO simulation with a map of a robot 
- launch a mapping algorithm in order to provide the robot a map_metadata topic
- launch the proxy that will listen to the initial pose and goal that you can provide for example in rviz
