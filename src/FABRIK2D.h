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
     *
     * solves the inverse kinematics of the stored chain to reach the target
     */
    void solve(float x, float y, int* lengths);
    
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
};

#endif