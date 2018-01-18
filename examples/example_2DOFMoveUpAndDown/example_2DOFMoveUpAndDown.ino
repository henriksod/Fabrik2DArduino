/********************************************************
 * FABRIK2D 2DOF example
 * Creating the FABRIK object and moving the end effector up and down.
 * You can use whichever unit you want for coordinates, lengths and tolerances as long as you are consistent.
 * Default unit is millimeters.
 ********************************************************/

#include <FABRIK2D.h>

int lengths[] = {50, 50}; // 2DOF arm where each limb is 50mm in length.
Fabrik2D fabrik2D(3, lengths); // This arm has 3 joints; one in the origin, the elbow and the end effector.

float y = 0;
int toggle_y = 0;

void setup() {
  Serial.begin(9600);

  Serial.print("ang1");
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
  Serial.println("y2");

  
  fabrik2D.setTolerance(0.5);
}

void loop() {

  if (y < 0) {
    toggle_y = 0;
    y = 0;
  } else if (y > 40) {
    toggle_y = 1;
    y = 40;
  }

  // Solve inverse kinematics given the coordinates x and y and the list of lengths for the arm.
  fabrik2D.solve(30,y,lengths);

  // Angle is printed in degrees, this is the angle between the origin and the end effector.
  // The function calls below shows how easy it is to get the results from the inverse kinematics solution.
  Serial.print(fabrik2D.getAngle(1)* 57296 / 1000);
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
  Serial.println(fabrik2D.getY(2));
  

  if (toggle_y == 0) {
    y++;
  } else {
    y--;
  }

  delay(50);
}
