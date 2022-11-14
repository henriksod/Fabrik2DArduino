# Iterative Inverse Kinematics Solver for Arduino

[![arduino-library-badge](https://www.ardu-badge.com/badge/Fabrik2D.svg?)](https://www.ardu-badge.com/Fabrik2D)
![C++](https://img.shields.io/badge/Langauge-C++-blue.svg)
[![License: MIT](https://img.shields.io/badge/License-MIT-brightgreen.svg)](https://opensource.org/licenses/MIT)
[![Tests](https://github.com/henriksod/Fabrik2DArduino/actions/workflows/test_runner.yml/badge.svg)](https://github.com/henriksod/Fabrik2DArduino/actions/workflows/test_runner.yml)
[![Spell Check](https://github.com/henriksod/Fabrik2DArduino/actions/workflows/spell-check.yml/badge.svg)](https://github.com/henriksod/Fabrik2DArduino/actions/workflows/spell-check.yml)

A fast forward- and inverse kinematics solver for Arduino based on the [FABRIK algorithm](http://www.andreasaristidou.com/FABRIK.html).

* This solver is simple to use.

* It is as fast as analytical IK solvers, but allows arbitrary joint configurations.

* It lets you specify (x, y) coordinates and it will compute the joint angles required for the end of the manipulator (end effector) to reach the desired position with up to 3 degrees of freedom (DOF).

* With the addition of a rotating base, you can move in (x, y, z) coordinates with up to 4 degrees of freedom (DOF).

* You can also specify a desired angle for the end effector to approach the desired position, which allows picking up objects from different orientations.

* Moreover, you can adjust the gripping offset to compensate for your chosen gripper or move towards objects from specific angles.

<p align="center">
  <img src="https://github.com/henriksod/Fabrik2DArduino/blob/master/images/exec_evaluation.png" width="512" height="auto" />
</p>


Usage
-----

```cpp
#include <FABRIK2D.h>
#include <Servo.h>

// A 2DOF arm, where we have 2 links and 2+1 joints, 
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
  shoulder.write(min(180, max(0, -shoulderAngle)));
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
| Example | Description | Visualization |
| --- | --- | --- |
| [example_2DOFMoveUpAndDown](/examples/example_2DOFMoveUpAndDown/example_2DOFMoveUpAndDown.ino) | Creates a 2DOF arm and moves it up and down. | <img src="https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_2DOFMoveUpAndDown/preview.gif" width="128" height="128" /> |
| [example_2DOFMoveCircle](/examples/example_2DOFMoveCircle/example_2DOFMoveCircle.ino) | Creates a 2DOF arm and moves it in a circle. | <img src="https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_2DOFMoveCircle/preview.gif" width="128" height="128" /> |
| [example_3DOFToolAngle](/examples/example_3DOFToolAngle/example_3DOFToolAngle.ino) | Creates a 3DOF arm and moves it in a circle with given tool angle. | <img src="https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFToolAngle/preview1.gif" width="128" height="128" /><img src="https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFToolAngle/preview2.gif" width="128" height="128" /> |
| [example_3DOFGrippingOffset](/examples/example_3DOFGrippingOffset/example_3DOFGrippingOffset.ino) | Creates a 3DOF arm and moves it in a horizontal line with given tool angle and varying gripping offset. | <img src="https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_3DOFGrippingOffset/preview.gif" width="128" height="128" /> |
| [example_4DOF](/examples/example_4DOF/example_4DOF.ino) | Creates a 4DOF arm which solves for a rotating base which yields movement in the z-axis (offset from the chain plane). | <img src="https://github.com/henriksod/Fabrik2DArduino/blob/master/examples/example_4DOF/preview.gif" width="128" height="128" /> |


Servo Orientation
------------
The library computes angles with respect to the x-axis. For the joints, the x-axis is always pointing in the direction of the attached arm. This means that all angles which makes a joint point down will be negative. Servos do not take negative angles. If we are using 180 degree servos, we will have to do some modifications to the angle outputs from the library, as shown below:
```cpp
// Let's say we have a manipulator with 3 joints. A shoulder joint, an elbow joint, and a wrist joint.

// Solve IK for 2 DOF (x and y)
fabrik2D.solve(x,y,lengths);

// Get the angles (in radians [-pi,pi]) and convert them to degrees [-180,180]
shoulderAngle = fabrik2D.getAngle(0) * RAD_TO_DEG; // In degrees
elbowAngle = fabrik2D.getAngle(1) * RAD_TO_DEG; // In degrees
wristAngle = fabrik2D.getAngle(2) * RAD_TO_DEG // In degrees

// Compute servo angles
shoulder.write(min(180, max(0, -shoulderAngle)));
elbow.write(min(180, max(0, elbowAngle + 90)));
wrist.write(min(180, max(0, wristAngle + 90)));
```
As you can see in the example above, the servos are clamped between 0 and 180 degrees. Moreover, the elbow and wrist servos are rotated 90 degrees. This makes it possible for the elbow and wrist joints to point down while keeping a positive angle. We assume that the servos for the elbow and wrist joints are oriented such that 0 degrees is pointing forwards and 180 degrees is pointing backwards.

All angles under 90 degrees will make the joint point down while all angles above 90 degrees will make the joint point up. We are not doing this for the shoulder joint in this case. Instead, we take the negative shoulder angle. We assume the servo is oriented with its 0 degrees pointing up and 180 degrees pointing down. This increases the range of motion in front of the robotic arm.

Here is an example of how a 4 DOF setup might look like:

<p align="center">
  <img src="https://github.com/henriksod/Fabrik2DArduino/blob/master/images/4DOFSetup.png" width="512" height="auto" />
</p>


Installation
------------
Clone this repository to your local machine, and place it in your Arduino libraries folder as 'Fabrik2DArduino'.

You can also download the library via Arduino IDE. Navigate to Sketch->Include Library->Manage Libraries... and search for "Fabrik2D", then press "install".


Notice
------------
It is recommended that you implement your own acceleration and velocity functions to make sure that your manipulator does not snap into the solved positions (which could cause breakage or slipping)! One way of doing this is to interpolate the joint angles over time until the manipulator has reached it's destination. I would recommend using [RAMP](https://github.com/siteswapjuggler/RAMP), an Arduino interpolation library made by [siteswapjuggler](https://github.com/siteswapjuggler).

Todo
------------
* Implement templates for all methods with default type as single precision float. Make calculations work for arbitrary-precision integers and floating point numbers.
* Add support for 6 DOF with a perpendicular rotating joint between the elbow and the wrist joints. Hint: Divide the problem into two IK chains with a rotating "base". In essence, attach a Fabrik2D object onto another Fabrik2D object.
