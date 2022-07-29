//
//    FILE: unit_tests.cpp
//  AUTHOR: Henrik SÖderlund
//    DATE: 2022-07-29
// PURPOSE: unit tests for the solve() methods of Fabrik2D
//          https://github.com/henriksod/Fabrik2DArduino
//          https://github.com/Arduino-CI/arduino_ci/blob/master/REFERENCE.md
//

// supported assertions
// ----------------------------
// assertEqual(expected, actual);               // a == b
// assertNotEqual(unwanted, actual);            // a != b
// assertComparativeEquivalent(expected, actual);    // abs(a - b) == 0 or (!(a > b) && !(a < b))
// assertComparativeNotEquivalent(unwanted, actual); // abs(a - b) > 0  or ((a > b) || (a < b))
// assertLess(upperBound, actual);              // a < b
// assertMore(lowerBound, actual);              // a > b
// assertLessOrEqual(upperBound, actual);       // a <= b
// assertMoreOrEqual(lowerBound, actual);       // a >= b
// assertTrue(actual);
// assertFalse(actual);
// assertNull(actual);

// // special cases for floats
// assertEqualFloat(expected, actual, epsilon);    // fabs(a - b) <= epsilon
// assertNotEqualFloat(unwanted, actual, epsilon); // fabs(a - b) >= epsilon
// assertInfinity(actual);                         // isinf(a)
// assertNotInfinity(actual);                      // !isinf(a)
// assertNAN(arg);                                 // isnan(a)
// assertNotNAN(arg);                              // !isnan(a)


#include <ArduinoUnitTests.h>

#include "Arduino.h"
#include "FABRIK2D.h"


unittest_setup()
{
}


unittest_teardown()
{
}


unittest(test_constructor)
{
  int lengths_3_joints[] = {190, 200};
  int lengths_4_joints[] = {190, 200, 270};
  
  Fabrik2D fabrik2D_4(4, lengths_4_joints);
  assertEqual(4, fabrik2D_4.numJoints());
  assertEqual(10, fabrik2D_4.getTolerance());
  assertEqual(0, fabrik2D_4.getChain()->angle);
  assertEqual(0, fabrik2D_4.getChain()->z);
  assertEqual(0, fabrik2D_4.getChain()->joints[0].x);
  assertEqual(0, fabrik2D_4.getChain()->joints[0].y);
  assertEqual(0, fabrik2D_4.getChain()->joints[0].angle);
  assertEqual(0, fabrik2D_4.getChain()->joints[1].x);
  assertEqual(190, fabrik2D_4.getChain()->joints[1].y);
  assertEqual(0, fabrik2D_4.getChain()->joints[1].angle);
  assertEqual(0, fabrik2D_4.getChain()->joints[2].x);
  assertEqual(200, fabrik2D_4.getChain()->joints[2].y);
  assertEqual(0, fabrik2D_4.getChain()->joints[2].angle);
  assertEqual(0, fabrik2D_4.getChain()->joints[3].x);
  assertEqual(270, fabrik2D_4.getChain()->joints[3].y);
  assertEqual(0, fabrik2D_4.getChain()->joints[3].angle);
  
  Fabrik2D fabrik2D_3(3, lengths_3_joints);
  assertEqual(3, fabrik2D_3.numJoints());
  assertEqual(10, fabrik2D_3.getTolerance());
  assertEqual(0, fabrik2D_3.getChain()->angle);
  assertEqual(0, fabrik2D_3.getChain()->z);
  assertEqual(0, fabrik2D_3.getChain()->joints[0].x);
  assertEqual(0, fabrik2D_3.getChain()->joints[0].y);
  assertEqual(0, fabrik2D_3.getChain()->joints[0].angle);
  assertEqual(0, fabrik2D_3.getChain()->joints[1].x);
  assertEqual(190, fabrik2D_3.getChain()->joints[1].y);
  assertEqual(0, fabrik2D_3.getChain()->joints[1].angle);
  assertEqual(0, fabrik2D_3.getChain()->joints[2].x);
  assertEqual(200, fabrik2D_3.getChain()->joints[2].y);
  assertEqual(0, fabrik2D_3.getChain()->joints[2].angle);
  
  Fabrik2D fabrik2D_4_tolerance_1(4, lengths_4_joints, 1);
  assertEqual(1, fabrik2D_4_tolerance_1.getTolerance());
  
  Fabrik2D fabrik2D_3_begin_init;
  fabrik2D_3_begin_init.begin(3, lengths_3_joints);
  assertEqual(3, fabrik2D_3_begin_init.numJoints());
  assertEqual(10, fabrik2D_3_begin_init.getTolerance());
  assertEqual(0, fabrik2D_3_begin_init.getChain()->angle);
  assertEqual(0, fabrik2D_3_begin_init.getChain()->z);
  assertEqual(0, fabrik2D_3_begin_init.getChain()->joints[0].x);
  assertEqual(0, fabrik2D_3_begin_init.getChain()->joints[0].y);
  assertEqual(0, fabrik2D_3_begin_init.getChain()->joints[0].angle);
  assertEqual(0, fabrik2D_3_begin_init.getChain()->joints[1].x);
  assertEqual(190, fabrik2D_3_begin_init.getChain()->joints[1].y);
  assertEqual(0, fabrik2D_3_begin_init.getChain()->joints[1].angle);
  assertEqual(0, fabrik2D_3_begin_init.getChain()->joints[2].x);
  assertEqual(200, fabrik2D_3_begin_init.getChain()->joints[2].y);
  assertEqual(0, fabrik2D_3_begin_init.getChain()->joints[2].angle);
}


unittest_main()


// -- END OF FILE --