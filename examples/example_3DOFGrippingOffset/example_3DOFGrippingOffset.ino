/********************************************************
 * FABRIK2D 3DOF With Gripping Offset example
 * Creating the FABRIK object and moving the end effector along a horizontal line while adjusting gripping offset.
 * You can use whichever unit you want for coordinates, lengths and tolerances as long as you are consistent.
 * Default unit is millimeters.
 ********************************************************/

#include <FABRIK2D.h>

int lengths[] = {225, 150, 100}; // 3DOF arm where shoulder to elbow is 225mm, elbow to wrist is 150mm and wrist to end effector is 100mm.
Fabrik2D fabrik2D(4, lengths); // This arm has 4 joints; one in the origin, the elbow, the wrist and the end effector.

float x = 0;
int toggle_x = 0;

float grippingOffset = 20;
int toggle_grip = 0;

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

  // Move from 50 to 150 in the x axis
  if (x < 50) {
    toggle_x = 0;
    x = 50;
  } else if (x > 150) {
    toggle_x = 1;
    x = 150;
  }

  // Adjust gripping angle as we go
  if (grippingOffset < 0) {
    toggle_grip = 0;
    grippingOffset = 0;
  } else if (grippingOffset > 20) {
    toggle_grip = 1;
    grippingOffset = 20;
  }

  // Solve inverse kinematics given the coordinates x and y, the desired gripping offset, tool angle and the list of lengths for the arm.
  fabrik2D.solve(x, 50, -M_PI/2.0, grippingOffset, lengths);

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

  if (toggle_x == 0) {
    x++;
  } else {
    x--;
  }

  if (toggle_grip == 0) {
    grippingOffset++;
  } else {
    grippingOffset--;
  }

  //delay(50);
}
