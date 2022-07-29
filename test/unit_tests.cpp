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
  
  // Test 4 joints init
  fprintf(stderr, "Test 4 joints init\n");
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
  assertEqual(190+200, fabrik2D_4.getChain()->joints[2].y);
  assertEqual(0, fabrik2D_4.getChain()->joints[2].angle);
  assertEqual(0, fabrik2D_4.getChain()->joints[3].x);
  assertEqual(190+200+270, fabrik2D_4.getChain()->joints[3].y);
  assertEqual(0, fabrik2D_4.getChain()->joints[3].angle);
  
  // Test 3 joints init
  fprintf(stderr, "Test 3 joints init\n");
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
  assertEqual(190+200, fabrik2D_3.getChain()->joints[2].y);
  assertEqual(0, fabrik2D_3.getChain()->joints[2].angle);
  
  // Test different tolerance
  fprintf(stderr, "Test different tolerance\n");
  Fabrik2D fabrik2D_4_tolerance_1(4, lengths_4_joints, 1);
  assertEqual(1, fabrik2D_4_tolerance_1.getTolerance());
  fabrik2D_4_tolerance_1.setTolerance(20);
  assertEqual(20, fabrik2D_4_tolerance_1.getTolerance());
  
  // Test begin
  fprintf(stderr, "Test begin\n");
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
  assertEqual(190+200, fabrik2D_3_begin_init.getChain()->joints[2].y);
  assertEqual(0, fabrik2D_3_begin_init.getChain()->joints[2].angle);
}

unittest(test_solve)
{
    int success = 0;
    int lengths_3_joints[] = {200, 200};
    int lengths_4_joints[] = {200, 200, 200};
    
    // Solve 3 joints, 2DOF
    fprintf(stderr, "Solve 3 joints, 2DOF\n");
    Fabrik2D fabrik2D_3_2DOF(3, lengths_3_joints, 1);
    success = fabrik2D_3_2DOF.solve(100, 100, lengths_3_joints);
    assertEqual(1, success);
    
    float dist = sqrt(100*100 + 100*100);
    
    float c_phi = (
        dist*dist - lengths_3_joints[0]*lengths_3_joints[0]
        - lengths_3_joints[1]*lengths_3_joints[1]
    ) / (2*lengths_3_joints[0]*lengths_3_joints[1]);
    float s_phi = sqrt(1 - c_phi*c_phi);
    
    // Assign the angles for shoulder and elbow.
    float a1 = atan2(100, 100) + atan2(
        lengths_3_joints[1]*s_phi,
        lengths_3_joints[0]+lengths_3_joints[1]*c_phi);
    float a2 = -atan2(s_phi, c_phi);
    
    assertEqualFloat(a1, fabrik2D_3_2DOF.getAngle(0), fabrik2D_3_2DOF.getTolerance());
    assertEqualFloat(a2, fabrik2D_3_2DOF.getAngle(1), fabrik2D_3_2DOF.getTolerance());
    
    assertEqualFloat(100, fabrik2D_3_2DOF.getX(2), fabrik2D_3_2DOF.getTolerance());
    assertEqualFloat(100, fabrik2D_3_2DOF.getY(2), fabrik2D_3_2DOF.getTolerance());
    
    // Test Set Joints
    float angles[] = {a1, a2};
    fprintf(stderr, "Test Set Joints\n");
    fabrik2D_3_2DOF.setJoints(angles, lengths_3_joints);
    assertEqualFloat(100, fabrik2D_3_2DOF.getX(2), 1e-3);
    assertEqualFloat(100, fabrik2D_3_2DOF.getY(2), 1e-3);

    // Test Solve Fail
    fprintf(stderr, "Test Solve Fail\n");
    success = fabrik2D_3_2DOF.solve(1000, 1000, lengths_3_joints);
    assertEqual(0, success);

    // Test Solve tolerance too low
    fprintf(stderr, "Test Solve tolerance too low\n");
    fabrik2D_3_2DOF.setTolerance(0.001);
    success = fabrik2D_3_2DOF.solve(100, 100, lengths_3_joints);
    assertEqual(2, success);

    // Solve 4 joints, 3DOF
    fprintf(stderr, "Solve 4 joints, 3DOF (tool angle)\n");
    Fabrik2D fabrik2D_3_3DOF(4, lengths_4_joints, 10);
    success = fabrik2D_3_3DOF.solve(50, 50, -HALF_PI, lengths_4_joints);
    assertEqual(1, success);
    
    assertEqualFloat(50, fabrik2D_3_3DOF.getX(3), fabrik2D_3_3DOF.getTolerance());
    assertEqualFloat(50, fabrik2D_3_3DOF.getY(3), fabrik2D_3_3DOF.getTolerance());
    
    assertEqualFloat(50, fabrik2D_3_3DOF.getX(2), fabrik2D_3_3DOF.getTolerance());
    assertEqualFloat(250, fabrik2D_3_3DOF.getY(2), fabrik2D_3_3DOF.getTolerance());
    
    // Solve 4 joints, 3DOF (tool angle), Gripping offset
    fprintf(stderr, "Solve 4 joints, 3DOF, Gripping offset\n");
    Fabrik2D fabrik2D_3_3DOF_GO(4, lengths_4_joints, 1);
    success = fabrik2D_3_3DOF_GO.solve(50, 50, -HALF_PI, 10, lengths_4_joints);
    assertEqual(1, success);
    
    assertEqualFloat(50, fabrik2D_3_3DOF_GO.getX(3), fabrik2D_3_3DOF_GO.getTolerance());
    assertEqualFloat(60, fabrik2D_3_3DOF_GO.getY(3), fabrik2D_3_3DOF_GO.getTolerance());
    
    assertEqualFloat(50, fabrik2D_3_3DOF_GO.getX(2), fabrik2D_3_3DOF_GO.getTolerance());
    assertEqualFloat(260, fabrik2D_3_3DOF_GO.getY(2), fabrik2D_3_3DOF_GO.getTolerance());
    
    // Solve 4 joints, 3DOF
    fprintf(stderr, "Solve 4 joints, 3DOF\n");
    Fabrik2D fabrik2D_4_3DOF(4, lengths_4_joints, 1);
    success = fabrik2D_4_3DOF.solve2(100, 100, 100, lengths_4_joints);
    assertEqual(1, success);
    
    float base_angle = atan2(100, 100);
    
    assertEqualFloat(base_angle, fabrik2D_4_3DOF.getBaseAngle(), 1e-3);
    
    assertEqualFloat(100, fabrik2D_4_3DOF.getX(3), fabrik2D_4_3DOF.getTolerance());
    assertEqualFloat(100, fabrik2D_4_3DOF.getY(3), fabrik2D_4_3DOF.getTolerance());
    assertEqualFloat(100, fabrik2D_4_3DOF.getZ(3), fabrik2D_4_3DOF.getTolerance());
    
    // Solve 4 joints, 4DOF
    fprintf(stderr, "Solve 4 joints, 4DOF\n");
    Fabrik2D fabrik2D_4_4DOF(4, lengths_4_joints, 1);
    success = fabrik2D_4_4DOF.solve2(150, 50, 100, -HALF_PI, lengths_4_joints);
    assertEqual(1, success);
    
    assertEqualFloat(150, fabrik2D_4_4DOF.getX(3), fabrik2D_4_4DOF.getTolerance());
    assertEqualFloat(50, fabrik2D_4_4DOF.getY(3), fabrik2D_4_4DOF.getTolerance());
    assertEqualFloat(100, fabrik2D_4_4DOF.getZ(3), fabrik2D_4_4DOF.getTolerance());
    
    assertEqualFloat(150, fabrik2D_4_4DOF.getX(2), fabrik2D_4_4DOF.getTolerance());
    assertEqualFloat(250, fabrik2D_4_4DOF.getY(2), fabrik2D_4_4DOF.getTolerance());
    
    // Solve 4 joints, 4DOF, Gripping offset
    fprintf(stderr, "Solve 4 joints, 4DOF, Gripping offset\n");
    Fabrik2D fabrik2D_4_4DOF_GO(4, lengths_4_joints, 1);
    success = fabrik2D_4_4DOF_GO.solve2(150, 50, 0, -HALF_PI, 10, lengths_4_joints);
    assertEqual(1, success);
    
    assertEqualFloat(150, fabrik2D_4_4DOF_GO.getX(3), fabrik2D_4_4DOF_GO.getTolerance());
    assertEqualFloat(60, fabrik2D_4_4DOF_GO.getY(3), fabrik2D_4_4DOF_GO.getTolerance());
    assertEqualFloat(0, fabrik2D_4_4DOF_GO.getZ(3), fabrik2D_4_4DOF_GO.getTolerance());
    
    assertEqualFloat(150, fabrik2D_4_4DOF_GO.getX(2), fabrik2D_4_4DOF_GO.getTolerance());
    assertEqualFloat(260, fabrik2D_4_4DOF_GO.getY(2), fabrik2D_4_4DOF_GO.getTolerance());
    
    // Test Set Base Angle
    fprintf(stderr, "Test Set Base Angle\n");
    fprintf(stderr, "Test Set Base Angle %f\n", HALF_PI - fabrik2D_4_4DOF_GO.getChain()->angle);
    fabrik2D_4_4DOF_GO.setBaseAngle(HALF_PI);
    assertEqualFloat(0, fabrik2D_4_4DOF_GO.getX(3), fabrik2D_4_4DOF_GO.getTolerance());
    assertEqualFloat(60, fabrik2D_4_4DOF_GO.getY(3), fabrik2D_4_4DOF_GO.getTolerance());
    assertEqualFloat(150, fabrik2D_4_4DOF_GO.getZ(3), fabrik2D_4_4DOF_GO.getTolerance());
    
}

unittest(test_getters_setters)
{
    int lengths[] = {200, 200};
    Fabrik2D fabrik2D(3, lengths);

    fabrik2D.setTolerance(20);
    assertEqual(20, fabrik2D.getTolerance());
    
    fabrik2D.setBaseAngle(HALF_PI);
    assertEqualFloat(HALF_PI, fabrik2D.getBaseAngle(), 1e-3);
    
    assertEqual(0, fabrik2D.getZ());
    assertEqual(0, fabrik2D.getAngle(0));
    assertEqual(0, fabrik2D.getAngle(1));
    assertEqual(200, fabrik2D.getY(1));
    assertEqual(400, fabrik2D.getY(2));
    assertEqual(0, fabrik2D.getX(0));
    assertEqual(0, fabrik2D.getX(1));
}

unittest_main()


// -- END OF FILE --