FABRIK based 2D Inverse kinematics solver
=====

***************************************************************
* FABRIK 2D inverse kinematics solver - Version 0.6.6
* by Henrik Söderlund <henrik.a.soderlund@gmail.com>
* This Library is licensed under a GPLv3 License
***************************************************************

A FABRIK based inverse kinematics solver for Arduino.

This library lets you specify coordinates in the 2-dimensional plane and it will compute the joint angles required for the end of the manipulator (end effector) to reach the desired position. It works for N number of joints.
You can also specify a desired angle for the end effector to approach the desired position, which allows picking up objects from different orientations.
This library has protection against unsolvability by simply not computing unsolvable inverse kinematics problems for unreachable positions.

The library is based on an iterative inverse kinematics algorithm called FABRIK:
http://www.andreasaristidou.com/FABRIK.html

Usage
-----

```C++
#include <FABRIK2D.h>
#include <Servo.h>

// For a 2DOF arm
int lengths[] = {225, 150};
Fabrik2D fabrik2D(3, lengths);

Servo shoulder;
Servo elbow;

void setup() {
  shoulder.attach(9);
  elbow.attach(9);
  
  fabrik2D.setTolerance(0.5);
}

void loop() {
  // Move up to x=200, y=50
  fabrik2D.solve(200,50,lengths);
  
  // Get the angles (in radians) and convert them to degrees
  int shoulderAngle = fabrik2D.getAngle(0)* 57296 / 1000; // In degrees
  int elbowAngle = fabrik2D.getAngle(1)* 57296 / 1000; // In degrees
  
  // Write to the servos (with limits)
  shoulder.write(min(180, max(0, shoulderAngle)));
  elbow.write(min(180, max(0, elbowAngle)));
  
  delay(1000);
  
  // Move down to x=150, y=10
  fabrik2D.solve(150,10,lengths);
  
  // Get the angles (in radians) and convert them to degrees
  int shoulderAngle = fabrik2D.getAngle(0)* 57296 / 1000; // In degrees
  int elbowAngle = fabrik2D.getAngle(1)* 57296 / 1000; // In degrees
  
  // Write to the servos (with limits)
  shoulder.write(min(180, max(0, shoulderAngle)));
  elbow.write(min(180, max(0, elbowAngle)));
  
  delay(1000);
}
```

Three usage examples are included which give more in-depth information:
* example_2DOFMoveUpAndDown creates a 2DOF arm and moves it up and down
* example_3DOFMoveCircle creates a 3DOF arm and moves it in a circle
* example_3DOFToolAngle creates a 3DOF arm and moves it in a circle with given tool angle

**Example 2DOF chain moving up and down**                                                                                                                     |  **Example 3DOF chain moving in a circle**                                                                                              
:------------------------------------------------------------------------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------------------------------------------------------------:
![Example2DOF](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_2DOFMoveUpAndDown/preview.gif)                                       |  ![Example3DOF](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFMoveCircle/preview.gif)


**Example 3DOF chain moving in a circle with given tool angle at -45 degrees**                                                                                |  **Example 3DOF chain moving in a circle with given tool angle at -90 degrees**                                                                           
:------------------------------------------------------------------------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------------------------------------------------------------:
![Example3DOFToolAng45](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFToolAngle/preview1.gif)                                 |  ![Example3DOFToolAng90](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFToolAngle/preview2.gif)

Installation
------------
Clone this repository to your local machine, and place it in your Arduino libraries folder as 'Fabrik2DArduino'.

Class methods of Fabrik2D class
-----------------------------
* ```Fabrik2D(int numJoints, int* lengths)``` - The constructor of the class. Here you specify the number of joints (which cannot be changed) and the array of lengths which is always one less than the number of joints.
* ```bool solve(float x, float y, int* lengths)``` - Solves inverse kinematics for the end effector to reach (x,y). The length values can be changed which allows prismatic joints to be used. Returns false if IK could not be solved, will not update joints in this case.
* ```bool solve(float x, float y, float toolAngle, int* lengths)``` - Solves inverse kinematics for the end effector to reach (x,y) with a given tool angle. The length values can be changed which allows prismatic joints to be used. Returns false if IK could not be solved, will not update joints in this case.
* ```float getX(int n)``` - Current x coordinate of joint n
* ```float getY(int n)``` - Current y coordinate of joint n
* ```float getAngle(int n)``` - Current angle on joint n
* ```void setTolerance(float val)``` - Set tolerance to a value. If reachable, the end effector will approach the target with this tolerance.
* ```void setJoints(int* angles, int* lengths)``` - Manually sets the joint angles and updates their position using forward kinematics.

Notice
------------
It is recommended that you implement your own acceleration and velocity functions to make sure that your manipulator does not snap into the solved positions (which could cause breakage or slipping)! One way of doing this is to just increment the x and y positions and solving inverse kinematics over time until the manipulator has reached it's destination.
