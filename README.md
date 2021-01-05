# LEGO-EV3-n-DOF-Manipulator has two classes, actuator and robot, to implement a range of LEGO Mindstorms robots. This software is at the prototype stage.

# The short main program at the end is specifically for this six-axis plus end effector articulator:  https://photos.app.goo.gl/Z64PNGZADpMZKLYn9

# Before using this code, it's necessary to perform a kinematics analysis using Denavin-Hartenberg notation. Here's the diagram the author created for this initial code example: https://i1.wp.com/tomwilson.com/wp-content/uploads/2020/12/6-axis-angles-1.jpg?w=763&ssl=1

# Future: The robot go_to method will interface to an inverse kinematics library. The code will determine the x, y, z of an object to be picked up, eventually using machine vision. Inputs to the kinematics library will include that coordinate, plus other parameters, including desired end effector orientation and (yet-to-be-supplied) limb distances (a's).
