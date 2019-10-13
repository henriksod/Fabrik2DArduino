FABRIK Based 2D Inverse Kinematics Solver
=====

***************************************************************
* FABRIK 2D inverse kinematics solver - Version 1.0.3
* By Henrik Söderlund <henrik.a.soderlund@hotmail.com>
* This Library is licensed under a MIT License
***************************************************************

A FABRIK based inverse kinematics solver for Arduino.

This library lets you specify (x,y) coordinates in the 2-dimensional plane and it will compute the joint angles required for the end of the manipulator (end effector) to reach the desired position. It works for N number of joints up to 3DOF, but with support for 4DOF (3D movements) with the addition of a rotating base.
You can also specify a desired angle for the end effector to approach the desired position, which allows picking up objects from different orientations. Moreover, you can adjust the gripping offset to compensate for your chosen gripper. This library has protection against unsolvability by simply not computing unsolvable inverse kinematics problems for unreachable positions.

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

// Servos should be positioned so that when all servo angles are
// equal to 90 degrees, the manipulator should point straight up.
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
  // setup. Since the library may output negative angles, it is important
  // to apply limits before sending the angles to the servos!
  shoulder.write(min(180, max(0, shoulderAngle + 180/2)));
  elbow.write(min(180, max(0, elbowAngle + 180/2)));
  
  // The following delay is just a part of this example, remove it
  delay(1000);
  
  // Solve IK, move down to x=150, y=10
  fabrik2D.solve(150,10,lengths);
  
  // Get the angles (in radians [-pi,pi]) and convert them to degrees [-180,180]
  shoulderAngle = fabrik2D.getAngle(0)* 57296 / 1000; // In degrees
  elbowAngle = fabrik2D.getAngle(1)* 57296 / 1000; // In degrees
  
  // It is important that your servos are pointing straight up when set to
  // 90 degrees! This is because the servos are limited to 0 to 180 degrees
  // while the library is limited to -180 to 180 degrees. This is why the
  // servos have to be limited and the output from the library offset by
  // 90 degrees.
  shoulder.write(min(180, max(0, shoulderAngle + 180/2)));
  elbow.write(min(180, max(0, elbowAngle + 180/2)));
  
  // The following delay is just a part of this example, remove it
  delay(1000);
}
```

Five usage examples are included which give more in-depth information:
* __example_2DOFMoveUpAndDown__: creates a 2DOF arm and moves it up and down
* __example_3DOFMoveCircle__: creates a 3DOF arm and moves it in a circle
* __example_3DOFToolAngle__: creates a 3DOF arm and moves it in a circle with given tool angle
* __example_3DOFGrippingOffset__: creates a 3DOF arm and moves it in a horizontal line with given tool angle and varying gripping offset
* __example_4DOF__: creates a 3DOF arm and solves for a rotating base which yields movement in the z-axis (offset from the chain plane)

**&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; Example 2DOF chain moving in a vertical line &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;**                                                                                                                     |  **&nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; Example 3DOF chain moving in a circle &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp; &nbsp;**  
:------------------------------------------------------------------------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------------------------------------------------------------:
![Example2DOF](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_2DOFMoveUpAndDown/preview.gif)                                       |  ![Example3DOF](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFMoveCircle/preview.gif)

**Example 3DOF chain moving in a circle with given tool angle at -45 degrees**                                                                                |  **Example 3DOF chain moving in a vertical line with given tool angle at -90 degrees**                                                                           
:------------------------------------------------------------------------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------------------------------------------------------------:
![Example3DOFToolAng45](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFToolAngle/preview1.gif)                                 |  ![Example3DOFToolAng90](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFToolAngle/preview2.gif)

**Example 3DOF chain moving in a horizontal line with varying gripping offset and with tool angle at -90 degrees**                                            |  **Example 4DOF chain moving in a plane in the x-z axes with given tool angle at -72 degrees**                                                                           
:------------------------------------------------------------------------------------------------------------------------------------------------------------:|:-------------------------------------------------------------------------------------------------------------------------------:
![Example3DOFGrippingOffset](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFGrippingOffset/preview.gif)                                 |  ![Example4DOF](https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_4DOF/preview.gif)

Robot Arm Configuration
------------
Due to many requests by email, I have decided to provide you with figures illustrating the configuration of the robot arm that is necessary for this library to work as it is supposed to. Remember that these are only examples of how your arm could look like, but the same concept is applied to any arm that you use with this library. To use the examples, an additional joint is required. I did not include all joints in the configuration example images because the images would have been unnecessarily complex. What is important is that the servos are oriented correctly, as described in the images.

**Robot arm configuration in 2D:**

<p align="center">
<img src="https://github.com/henriksod/Fabrik2DArduino/blob/master/setup/3DOFSetup.png" width="500">
</p>

**Robot arm configuration in 3D:**

<p align="center">
<img src="https://github.com/henriksod/Fabrik2DArduino/blob/master/setup/4DOFSetup.png" width="500">
</p>

**DISCLAIMER: What is important, and can be seen in the figures, is that the servos' angles have to be 90 degrees when the link is parallel to the previous link. By setting up the servos in this manner, the arm will be pointing straight up when all joint angles are set to zero (in the library). This is because the library is limited between -180 to 180 degrees while the servos are in general between 0 to 180 degrees. Moreover, the library has it's zero angle along the y-axis so in order for the servos to move below this point, they have to be offset by 90 degrees. I will make a video in the future showing exactly what I mean. In the meantime, you will have to try to understand what I am saying here.**

Installation
------------
Clone this repository to your local machine, and place it in your Arduino libraries folder as 'Fabrik2DArduino'.

You can also download the library via Arduino IDE. Navigate to Sketch->Include Library->Manage Libraries... and search for "Fabrik2D", then press "install".

Methods of Fabrik2D class
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
It is recommended that you implement your own acceleration and velocity functions to make sure that your manipulator does not snap into the solved positions (which could cause breakage or slipping)! One way of doing this is to interpolate the joint angles over time until the manipulator has reached it's destination. I would reccoment using [RAMP](https://github.com/siteswapjuggler/RAMP), an Arduino interpolation library made by [siteswapjuggler](https://github.com/siteswapjuggler).
