/**********************************************************************************************
 * FABRIK 2D inverse kinematics solver - Version 0.1
 * by Henrik Söderlund <henrik.a.soderlund@gmail.com>
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/


#ifndef FABRIK2D_h
#define FABRIK2D_h

#include "Arduino.h"

class Fabrik2D
{
  public:
    /* Fabrik2D(numJoints, lengths)
     * inputs: numJoints, lengths
     *
     * creates the chain to be used for the inverse kinematics solver
     */
    Fabrik2D(int numJoints, int* lengths);

    /* solve(x, y, lengths)
     * inputs: x and y positions of target, lengths between each joint
     * outputs: True if solvable, false if not solvable
     *
     * solves the inverse kinematics of the stored chain to reach the target
     */
    bool solve(float x, float y, int* lengths);
    
    /* solve(x, y, angle, lengths)
     * inputs: x and y positions of target, desired tool angle and lengths between each joint
     * outputs: True if solvable, false if not solvable
     *
     * !!! tool angle is in radians !!!
     *
     * solves the inverse kinematics of the stored chain to reach the target with tool angle
     *
     * will only work for 3DOF or more
     */
    bool solve(float x, float y, float toolAngle, int* lengths);
    
    /* getX(joint)
     * inputs: joint number
     * outputs: x position of joint
     */
    float getX(int joint);
    
    /* getY(joint)
     * inputs: joint number
     * outputs: y position of joint
     */
    float getY(int joint);
    
    /* getAngle(joint)
     * inputs: joint number
     * outputs: angle (radians) of joint
     */
    float getAngle(int joint);
    
    /* setTolerance(tolerance)
     * inputs: tolerance value
     * 
     * sets the tolerance of the distance between the end effector and the target
     */
    void setTolerance(float tolerance);

  private:
    
    // Joint struct
    typedef struct 
    {
        float x; // x position of joint relative to origin
        float y; // y position of joint relative to origin
        float angle; // angle of joint (if the joint has adjacent joints or origin)
    } Joint;
    
    // Chain struct
    typedef struct 
    {
      Joint* joints; // list of joints
    } Chain;
    
    // Number of joints in the chain
    int numJoints;
    // Tolerance of distance between end effector and target
    float tolerance;
    // The chain containing joints
    Chain* chain;
    
    /* createChain(lengths)
     * inputs: lengths
     *
     * length size should always be one lesser than the number of joints
     */
    void createChain(int* lengths);
    
    /* distance(x1,y1,x2,y2)
     * inputs: coordinates
     * outputs: distance between points
     *
     * Uses euclidean distance
     */
    float distance(float x1, float y1, float x2, float y2);
};

#endif