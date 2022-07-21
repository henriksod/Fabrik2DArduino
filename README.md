# FABRIK Based 2D Inverse Kinematics Solver

[![Compilation Status](https://github.com/henriksod/Fabrik2DArduino/actions/workflows/compile_benchmark.yml/badge.svg)](https://github.com/henriksod/Fabrik2DArduino/actions/workflows/compile_benchmark.yml)
[![Latest Release](https://github.com/henriksod/Fabrik2DArduino/actions/workflows/release.yml/badge.svg)](https://github.com/henriksod/Fabrik2DArduino/actions/workflows/release.yml)

***************************************************************
* FABRIK 2D inverse kinematics solver - Version 1.0.4
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
  int shoulderAngle = fabrik2D.getAngle(0) * RAD_TO_DEG; // In degrees
  int elbowAngle = fabrik2D.getAngle(1) * RAD_TO_DEG; // In degrees
  
  // Compute servo angles based on the output (see explanation in README.md under "Servo Orientation")
  shoulder.write(min(180, max(0, shoulderAngle)));
  elbow.write(min(180, max(0, elbowAngle + 90)));
  
  // The following delay is just a part of this example, remove it
  delay(1000);
  
  // Solve IK, move down to x=150, y=10
  fabrik2D.solve(150,10,lengths);
  
  // Get the angles (in radians [-pi,pi]) and convert them to degrees [-180,180]
  shoulderAngle = fabrik2D.getAngle(0) * RAD_TO_DEG; // In degrees
  elbowAngle = fabrik2D.getAngle(1) * RAD_TO_DEG; // In degrees
  
  // Compute servo angles based on the output (see explanation in README.md under "Servo Orientation")
  shoulder.write(min(180, max(0, shoulderAngle)));
  elbow.write(min(180, max(0, elbowAngle + 90)));
  
  // The following delay is just a part of this example, remove it
  delay(1000);
}
```

Five usage examples are included which give more in-depth information:
| Example | Description |
| --- | --- |
| [example_2DOFMoveUpAndDown](/examples/example_2DOFMoveUpAndDown/example_2DOFMoveUpAndDown.ino) | Creates a 2DOF arm and moves it up and down.<br /><img src="https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_2DOFMoveUpAndDown/preview.gif" width="128" height="128" /> | 
| [example_3DOFMoveCircle](/examples/example_3DOFMoveCircle/example_3DOFMoveCircle.ino) | Creates a 3DOF arm and moves it in a circle.<br /><img src="https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFMoveCircle/preview.gif" width="128" height="128" /> |
| [example_3DOFToolAngle](/examples/example_3DOFToolAngle/example_3DOFToolAngle.ino) | Creates a 3DOF arm and moves it in a circle with given tool angle.<br /><img src="https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFToolAngle/preview1.gif" width="128" height="128" /><img src="https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFToolAngle/preview2.gif" width="128" height="128" /> |
| [example_3DOFGrippingOffset](/examples/example_3DOFGrippingOffset/example_3DOFGrippingOffset.ino) | Creates a 3DOF arm and moves it in a horizontal line with given tool angle and varying gripping offset.<br /><img src="https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFGrippingOffset/preview.gif" width="128" height="128" /> |
| [example_4DOF](/examples/example_4DOF/example_4DOF.ino) | Creates a 3DOF arm and solves for a rotating base which yields movement in the z-axis (offset from the chain plane).<br /><img src="https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_4DOF/preview.gif" width="128" height="128" /> |


Servo Orientation
------------
The library computes angles with respect to the x-axis. This means that all angles which makes a joint point down will be negative. Servos do not take negative angles. If we are using 180 degree servos, we will have to do some modificaitons to the angle outputs from the library, as shown below:
```
// Let's say we have a manipulator with 3 joints. A shoulder joint, an elbow joint, and a wrist joint (3 DOF).

// Solve IK
fabrik2D.solve(x,y,lengths);

// Get the angles (in radians [-pi,pi]) and convert them to degrees [-180,180]
shoulderAngle = fabrik2D.getAngle(0) * RAD_TO_DEG; // In degrees
elbowAngle = fabrik2D.getAngle(1) * RAD_TO_DEG; // In degrees
wristAngle = fabrik2D.getAngle(2) * RAD_TO_DEG // In degrees

// Compute servo angles
shoulder.write(min(180, max(0, shoulderAngle)));
elbow.write(min(180, max(0, elbowAngle + 90)));
wrist.write(min(180, max(0, wristAngle + 90)));
```
As you can see, the servos are clamped between 0 and 180 degrees. Moreover, the elbow and wrist servos are rotated 90 degrees. This makes it possible for the elbow and wrist joints to point down while keeping a positive angle. All angles under 90 degrees will make the joint point down while all angles above 90 degrees will make the joint point up. We are not doing this for the shoulder joint in this case, because we want it to be able to point backwards and forwards along the surface, e.g. a table.


Installation
------------
Clone this repository to your local machine, and place it in your Arduino libraries folder as 'Fabrik2DArduino'.

You can also download the library via Arduino IDE. Navigate to Sketch->Include Library->Manage Libraries... and search for "Fabrik2D", then press "install".


Notice
------------
It is recommended that you implement your own acceleration and velocity functions to make sure that your manipulator does not snap into the solved positions (which could cause breakage or slipping)! One way of doing this is to interpolate the joint angles over time until the manipulator has reached it's destination. I would reccoment using [RAMP](https://github.com/siteswapjuggler/RAMP), an Arduino interpolation library made by [siteswapjuggler](https://github.com/siteswapjuggler).
