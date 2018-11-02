/********************************************************
 * FABRIK2D 4DOF example
 * Creating the FABRIK object and moving the end effector in a linear motion in x, y, z coordinates.
 * You can use whichever unit you want for coordinates, lengths and tolerances as long as you are consistent.
 * Default unit is millimeters.
 ********************************************************/

#include <FABRIK2D.h>

int lengths[] = {225, 150, 100}; // 3DOF arm where shoulder to elbow is 225mm, elbow to wrist is 150mm and wrist to end effector is 100mm.
Fabrik2D fabrik2D(4, lengths); // This arm has 4 joints; one in the origin, the elbow, the wrist and the end effector.

float x = 0;
int toggle_x = 0;

float z = 0;
int toggle_z = 0;

void setup() {
  Serial.begin(9600);

  Serial.print("ang0");
  Serial.print("\t");
  Serial.print("ang1");
  Serial.print("\t");
  Serial.print("ang2");
  Serial.print("\t");
  Serial.print("x0");
  Serial.print("\t");
  Serial.print("y0");
  Serial.print("\t");
  Serial.print("x1");
  Serial.print("\t");
  Serial.print("y1");
  Serial.print("\t");
  Serial.print("x2");
  Serial.print("\t");
  Serial.print("y2");
  Serial.print("\t");
  Serial.print("x3");
  Serial.print("\t");
  Serial.print("y3");
  Serial.print("\t");
  Serial.println("angB");

  // Set tolerance to 0.5mm. If reachable, the end effector will approach
  // the target with this tolerance
  fabrik2D.setTolerance(0.5);
}

void loop() {

  // Move from 0 to 50 in the z axis
  if (z < 0) {
    toggle_z = 0;
    z = 0;
  } else if (z > 50) {
    toggle_z = 1;
    z = 50;
  }

  // Move from 50 to 150 in the x axis
  if (x < 50) {
    toggle_x = 0;
    x = 50;
  } else if (x > 150) {
    toggle_x = 1;
    x = 150;
  }

  // Solve inverse kinematics given the coordinates x and y, z, the desired gripping offset, tool angle and the list of lengths for the arm.
  // Note that for 3D movements, we use the solve2 method instead of solve.
  fabrik2D.solve2(x, 100, z, -M_PI/2.5, 0, lengths);

  // Angles are printed in degrees.
  // The function calls below shows how easy it is to get the results from the inverse kinematics solution.
  Serial.print(fabrik2D.getAngle(0)* 57296 / 1000);
  Serial.print("\t");
  Serial.print(fabrik2D.getAngle(1)* 57296 / 1000);
  Serial.print("\t");
  Serial.print(fabrik2D.getAngle(2)* 57296 / 1000);
  Serial.print("\t");
  Serial.print(fabrik2D.getX(0));
  Serial.print("\t");
  Serial.print(fabrik2D.getY(0));
  Serial.print("\t");
  Serial.print(fabrik2D.getX(1));
  Serial.print("\t");
  Serial.print(fabrik2D.getY(1));
  Serial.print("\t");
  Serial.print(fabrik2D.getX(2));
  Serial.print("\t");
  Serial.print(fabrik2D.getY(2));
  Serial.print("\t");
  Serial.print(fabrik2D.getX(3));
  Serial.print("\t");
  Serial.print(fabrik2D.getY(3));
  Serial.print("\t");
  Serial.println(fabrik2D.getBaseAngle());

  if (toggle_z == 0) {
    z++;
  } else {
    z--;
  }

  if (toggle_x == 0) {
    x++;
  } else {
    x--;
  }

  //delay(50);
}
