FABRIK based 2D Inverse kinematics solver
=====

***************************************************************
* FABRIK 2D inverse kinematics solver - Version 0.1
* by Henrik Söderlund <henrik.a.soderlund@gmail.com>
* This Library is licensed under a GPLv3 License
***************************************************************

A FABRIK based inverse kinematics solver for Arduino.

This library lets you specify coordinates in the 2-dimensional plane and it will compute the servo angles for the end of the robot arm (end effector) to reach the desired position.

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

Two usage examples are included which give more in-depth information:
* example_2DOFMoveUpAndDown creates a 2DOF arm and moves it up and down
* example_2DOFMoveUpAndDown creates a 3DOF arm and moves it in a circle

**Example 2DOF chain moving up and down**                                                                                |  **Example 3DOF chain moving in a circle**
:-----------------------------------------------------------------------------------------------------------------------:|:--------------------------------------------------------------------------------------------------------------------:
![Example2DOF](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_2DOFMoveUpAndDown/preview.gif)  |  ![Example3DOF](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFMoveCircle/preview.gif)

Installation
------------
Clone this repository to your local machine, and place it in your Arduino libraries folder as 'Fabrik2DArduino'.

Class methods of Fabrik2D class
-----------------------------
* ```Fabrik2D(int numJoints, int* lengths)``` - The constructor of the class. Here you specify the number of joints (which cannot be changed) and the array of lengths which is always one less than the number of joints.
* ```void solve(float x, float y, int* lengths)``` - Solves inverse kinematics for the end effector to reach (x,y). The length values can be changed which allows prismatic joints to be used.
* ```float getX(int n)``` - Current x coordinate of joint n
* ```float getY(int n)``` - Current y coordinate of joint n
* ```float getAngle(int n)``` - Current angle on joint n
* ```void setTolerance(float val)``` - Set tolerance to a value. If reachable, the end effector will approach the target with this tolerance.