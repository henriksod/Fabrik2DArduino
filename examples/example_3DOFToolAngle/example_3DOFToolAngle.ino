/********************************************************
 * FABRIK2D 3DOF example
 * Creating the FABRIK object and moving the end effector in a circular motion.
 * You can use whichever unit you want for coordinates, lengths and tolerances as long as you are consistent.
 * Default unit is millimeters.
 ********************************************************/

#include <FABRIK2D.h>

int lengths[] = {225, 150, 100}; // 3DOF arm where shoulder to elbow is 225mm, elbow to wrist is 150mm and wrist to end effector is 100mm.
Fabrik2D fabrik2D(4, lengths); // This arm has 4 joints; one in the origin, the elbow, the wrist and the end effector.

float ang = 0;
float radius = 30;
float x_offset = 100;
float y_offset = 150;

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
  Serial.println("y3");

  // Set tolerance to 0.5mm. If reachable, the end effector will approach
  // the target with this tolerance
  fabrik2D.setTolerance(0.5);
}

void loop() {

  // Move x and y in a circular motion
  float x = x_offset+radius*cos(ang * 1000 / 57296);
  float y = y_offset+radius*sin(ang * 1000 / 57296);

  ang = ((int)(ang + 1)) % 360;

  // Solve inverse kinematics given the coordinates x and y, the desired tool angle and the list of lengths for the arm.
  fabrik2D.solve(x, y, -M_PI/4.0, lengths);

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
  Serial.println(fabrik2D.getY(3));

  //delay(50);
}
