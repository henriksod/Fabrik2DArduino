FABRIK based 2D Inverse kinematics solver
=====

***************************************************************
* FABRIK 2D inverse kinematics solver - Version 1.0.1
* By Henrik Söderlund <henrik.a.soderlund@gmail.com>
* This Library is licensed under a GPLv3 License
***************************************************************

A FABRIK based inverse kinematics solver for Arduino.

This library lets you specify (x,y) coordinates in the 2-dimensional plane and it will compute the joint angles required for the end of the manipulator (end effector) to reach the desired position. It works for N number of joints up to 3DOF, but with support for 4DOF (3D) with the addition of a rotating base.
You can also specify a desired angle for the end effector to approach the desired position, which allows picking up objects from different orientations. Moreover, you can adjust the gripping offset to compensate for your chosen gripper and you can also solve IK with a rotating base which yields movement in the z-axis.
This library has protection against unsolvability by simply not computing unsolvable inverse kinematics problems for unreachable positions.

The library is based on an iterative inverse kinematics algorithm called FABRIK:
http://www.andreasaristidou.com/FABRIK.html

Usage
-----

```C++
#include <FABRIK2D.h>
#include <Servo.h>

// For a 2DOF arm, we have 2 links and 2+1 joints, 
// where the end effector counts as one joint in this case.
int lengths[] = {225, 150}; // Length of shoulder and elbow in mm.
Fabrik2D fabrik2D(3, lengths); // 3 Joints in total

// Servos should be positioned so that when all joint angles are
// equal to 0, the manipulator should point straight up.
Servo shoulder;
Servo elbow;

void setup() {
  shoulder.attach(9);
  elbow.attach(10);
  
  // Tolerance determines how much error is allowed for solving
  // the inverse kinematics for the end effector to reach the
  // desired point.
  fabrik2D.setTolerance(0.5);
}

void loop() {
  // Solve IK, move up to x=200, y=50
  fabrik2D.solve(200,50,lengths);
  
  // Get the angles (in radians [-pi,pi]) and convert them to degrees [-180,180]
  int shoulderAngle = fabrik2D.getAngle(0)* 57296 / 1000; // In degrees
  int elbowAngle = fabrik2D.getAngle(1)* 57296 / 1000; // In degrees
  
  // Write to the servos with limits, these will probably not be the same
  // for your manipulator and will have to be changed depending on your
  // setup.
  shoulder.write(min(180, max(0, shoulderAngle + 180/2)));
  elbow.write(min(180, max(0, elbowAngle + 180/2)));
  
  delay(1000);
  
  // Solve IK, move down to x=150, y=10
  fabrik2D.solve(150,10,lengths);
  
  // Get the angles (in radians [-pi,pi]) and convert them to degrees [-180,180]
  shoulderAngle = fabrik2D.getAngle(0)* 57296 / 1000; // In degrees
  elbowAngle = fabrik2D.getAngle(1)* 57296 / 1000; // In degrees
  
  // Write to the servos with limits, these will probably not be the same
  // for your manipulator and will have to be changed depending on your
  // setup.
  shoulder.write(min(180, max(0, shoulderAngle + 180/2)));
  elbow.write(min(180, max(0, elbowAngle + 180/2)));
  
  delay(1000);
}
```

Five usage examples are included which give more in-depth information:
* example_2DOFMoveUpAndDown creates a 2DOF arm and moves it up and down
* example_3DOFMoveCircle creates a 3DOF arm and moves it in a circle
* example_3DOFToolAngle creates a 3DOF arm and moves it in a circle with given tool angle
* example_3DOFGrippingOffset creates a 3DOF arm and moves it in a horizontal line with given tool angle and varying gripping offset
* example_4DOF creates a 3DOF arm and solves for a rotating base which yields movement in the z-axis (offset from the chain plane)

**&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; Example 2DOF chain moving up and down &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;**                                                                                                                     |  **&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; Example 3DOF chain moving in a circle &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;**  
:------------------------------------------------------------------------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------------------------------------------------------------:
![Example2DOF](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_2DOFMoveUpAndDown/preview.gif)                                       |  ![Example3DOF](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFMoveCircle/preview.gif)

**Example 3DOF chain moving in a circle with given tool angle at -45 degrees**                                                                                |  **Example 3DOF chain moving in a circle with given tool angle at -90 degrees**                                                                           
:------------------------------------------------------------------------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------------------------------------------------------------:
![Example3DOFToolAng45](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFToolAngle/preview1.gif)                                 |  ![Example3DOFToolAng90](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFToolAngle/preview2.gif)

**Example 3DOF chain moving in a horizontal line with varying gripping offset and with tool angle at -90 degrees**                                            |  **Example 4DOF chain moving in a plane in the x-z axes with given tool angle at -72 degrees**                                                                           
:------------------------------------------------------------------------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------------------------------------------------------------:
![Example3DOFGrippingOffset](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFGrippingOffset/preview.gif)                                 |  ![Example4DOF](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_4DOF/preview.gif)

Installation
------------
Clone this repository to your local machine, and place it in your Arduino libraries folder as 'Fabrik2DArduino'.

Class methods of Fabrik2D class
-----------------------------
* ```Fabrik2D(int numJoints, int* lengths)``` - The constructor of the class. Here you specify the number of joints (which cannot be changed) and the array of lengths which is always one less than the number of joints.
* ```bool solve(float x, float y, int* lengths)``` - Solves inverse kinematics for the end effector to reach (x,y). Returns false if IK could not be solved, will not update joints in this case.
* ```bool solve(float x, float y, float toolAngle, int* lengths)``` - Solves inverse kinematics for the end effector to reach (x,y) with a given tool angle. Returns false if IK could not be solved, will not update joints in this case.
* ```bool solve(float x, float y, float toolAngle, float grippingOffset, int* lengths)``` - Solves inverse kinematics for the end effector to reach (x,y) with a given tool angle and a gripping offset. Returns false if IK could not be solved, will not update joints in this case.
* ```bool solve2(float x, float y, float z, int* lengths)``` - Solves inverse kinematics for the end effector to reach (x,y,z). Requires a rotating base. Returns false if IK could not be solved, will not update joints in this case.
* ```bool solve2(float x, float y, float z, float toolAngle, int* lengths)``` - Solves inverse kinematics for the end effector to reach (x,y,z) with a given tool angle. Requires a rotating base. Returns false if IK could not be solved, will not update joints in this case.
* ```bool solve2(float x, float y, float z, float toolAngle, float grippingOffset, int* lengths)``` - Solves inverse kinematics for the end effector to reach (x,y,z) with a given tool angle and a gripping offset. Requires a rotating base. Returns false if IK could not be solved, will not update joints in this case.
* ```float getX(int n)``` - Current x coordinate of joint n.
* ```float getY(int n)``` - Current y coordinate of joint n.
* ```float getZ()``` - Current z offset of the chain end effector from the plane.
* ```float getAngle(int n)``` - Current angle on joint n.
* ```float getBaseAngle()``` - Current angle of the base of the chain (the angle in which the chain is pointing).
* ```void setTolerance(float val)``` - Set tolerance to a value. If reachable, the end effector will approach the target with this tolerance.
* ```void setJoints(int* angles, int* lengths)``` - Manually sets the joint angles and updates their position using forward kinematics.
* ```void setBaseAngle()``` - Manually set the angle of the base of the chain.

Notice
------------
It is recommended that you implement your own acceleration and velocity functions to make sure that your manipulator does not snap into the solved positions (which could cause breakage or slipping)! One way of doing this is to just increment the x and y positions and solving inverse kinematics over time until the manipulator has reached it's destination.
