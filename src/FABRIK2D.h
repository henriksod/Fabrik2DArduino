/**********************************************************************************************
 * FABRIK 2D inverse kinematics solver - Version 1.0.6
 * by Henrik Söderlund <henrik.a.soderlund@gmail.com>
 *
 * Copyright (c) 2018 Henrik Söderlund

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 **********************************************************************************************/


#ifndef SRC_FABRIK2D_H_
#define SRC_FABRIK2D_H_

#include "Arduino.h"

class Fabrik2D {
 public:
    // Joint struct
    typedef struct {
        float x = 0;  // x position of joint relative to origin
        float y = 0;  // y position of joint relative to origin
        float angle = 0;  // angle of joint
    } Joint;

    // Chain struct
    typedef struct {
      Joint* joints = nullptr;  // list of joints
      float z = 0;  // z position defining the chain offset from the plane
      float angle = 0;  // base (plane) rotation
    } Chain;

    /* Fabrik2D()
     *  
     *  Default constructor. Call begin(numJoints, lengths) to initialize
     */
    Fabrik2D() {}

    /* Fabrik2D(numJoints, lengths, tolerance)
     * inputs: numJoints, lengths, (optional) tolerance
     *
     * Calls begin(numJoints, lengths, tolerance)
     *
     * tolerance is optional, will be set to 10 by default
     */
    Fabrik2D(int numJoints, int lengths[], float tolerance = 10);

    /* Fabrik2D destructor */
    ~Fabrik2D();

    /*
     * begin(numJoints, lengths, tolerance)
     * inputs: numJoints, lengths, (optional) tolerance 
     * 
     * Initializes the library
     * creates the chain to be used for the inverse kinematics solver
     * 
     * tolerance is optional, will be set to 10 by default
     */
     void begin(int numJoints, int lengths[], float tolerance = 10);

    /* solve(x, y, lengths)
     *  
     * Inputs: x and y positions of target, lengths between each joint
     * Returns:
     *  0 if FABRIK could not converge
     *  1 if FABRIK converged to the set threshold
     *  2 if FABRIK converged with a higher tolerance value
     *
     * solves the inverse kinematics of the stored chain to reach the target
     * 
     * Returns:
     *  0 if FABRIK could not converge
     *  1 if FABRIK converged to the set threshold
     *  2 if FABRIK converged with a higher tolerance value
     */
    uint8_t solve(float x, float y, int lengths[]);

    /* solve(x, y, angle, lengths)
     *  
     * Inputs: x and y positions of target, desired tool angle and lengths
     *         between each joint
     * Returns:
     *  0 if FABRIK could not converge
     *  1 if FABRIK converged to the set threshold
     *  2 if FABRIK converged with a higher tolerance value
     *
     * !!! tool angle is in radians !!!
     *
     * solves the inverse kinematics of the stored chain to reach the target
     * with tool angle
     *
     * will only work for 3DOF
     */
    uint8_t solve(float x, float y, float toolAngle, int lengths[]);

    /* solve(x, y, angle, offset, lengths)
     *  
     * Inputs: x and y positions of target, desired tool angle and lengths
     *         between each joint
     * Returns:
     *  0 if FABRIK could not converge
     *  1 if FABRIK converged to the set threshold
     *  2 if FABRIK converged with a higher tolerance value
     *
     * !!! tool angle is in radians !!!
     *
     * solves the inverse kinematics of the stored chain to reach the target
     * with tool angle 
     * and gripping offset
     *
     * will only work for 3DOF
     */
    uint8_t solve(
        float x, float y,
        float toolAngle,
        float grippingOffset,
        int lengths[]);

    /* solve2(x, y, z, lengths)
     *  
     * Inputs: x, y and z positions of target, desired tool angle and lengths
     *         between each joint
     * Returns:
     *  0 if FABRIK could not converge
     *  1 if FABRIK converged to the set threshold
     *  2 if FABRIK converged with a higher tolerance value
     *
     * !!! tool angle is in radians !!!
     *
     * solves the inverse kinematics of the stored chain to reach the target
     * introducing the z-axis, which allows a rotational base of the manipulator
     *
     * angle of the chain defines the base rotation
     *
     * the x- and y-axes define the plane and the z-axis defines the offset
     * from the plane
     *
     * will only work for 4DOF, i.e. 4 joints or more and a rotational base
     */
    uint8_t solve2(float x, float y, float z, int lengths[]);

    /* solve2(x, y, z, toolAngle, lengths)
     *  
     * Inputs: x, y and z positions of target, desired tool angle and lengths
     *         between each joint
     * Returns:
     *  0 if FABRIK could not converge
     *  1 if FABRIK converged to the set threshold
     *  2 if FABRIK converged with a higher tolerance value
     *
     * !!! tool angle is in radians !!!
     *
     * solves the inverse kinematics of the stored chain to reach the target
     * with tool angle introducing the z-axis, which allows a rotational base
     * of the manipulator
     *
     * angle of the chain defines the base rotation
     *
     * the x- and y-axes define the plane and the z-axis defines the offset
     * from the plane
     *
     * will only work for 4DOF, i.e. 4 joints or more and a rotational base
     * 
     */
    uint8_t solve2(float x, float y, float z, float toolAngle, int lengths[]);

    /* solve2(x, y, z, angle, offset, lengths)
     *  
     * Inputs: x, y and z positions of target, desired tool angle, gripping
               offset and lengths between each joint
     * Returns:
     *  0 if FABRIK could not converge
     *  1 if FABRIK converged to the set threshold
     *  2 if FABRIK converged with a higher tolerance value
     *
     * !!! tool angle is in radians !!!
     *
     * solves the inverse kinematics of the stored chain to reach the target 
     * with tool angle and gripping offset introducing the z-axis, which
     * allows a rotational base of the manipulator
     *
     * angle of the chain defines the base rotation
     *
     * the x- and y-axes define the plane and the z-axis defines the offset
     * from the plane
     *
     * will only work for 4DOF, i.e. 4 joints or more and a rotational base
     */
    uint8_t solve2(
        float x, float y, float z,
        float toolAngle,
        float grippingOffset,
        int lengths[]);

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

    /* getZ(joint)
     * inputs: (optional) joint number
     * outputs: z offset of the chain from the plane
     * 
     * Passing the joint argument will not do anything.
     * It is just there for consistency with getX and getY.
     */
    float getZ(int joint = 0);

    /* getAngle(joint)
     * inputs: joint number
     * outputs: angle (radians) of joint
     */
    float getAngle(int joint);

    /* getBaseAngle()
     * outputs: base angle (radians) of chain
     */
    float getBaseAngle();

    /* setBaseAngle()
     * inputs: base angle (radians) of chain to set
     */
    void setBaseAngle(float baseAngle);

    /* getTolerance()
     * outputs: end effector tolerance of chain
     */
    float getTolerance();

    /* setTolerance(tolerance)
     * inputs: tolerance value
     * 
     * sets the tolerance of the distance between the end effector
     * and the target
     */
    void setTolerance(float tolerance);

    /* setJoints(angles, lengths)
     * inputs: New joint angles (in radians) and list of lengths
     *         between each joint
     *
     * Angles must be equal to the number of joints - 1
     * 
     * manually sets the joint angles and updates their position using
     * forward kinematics
     */
    void setJoints(float angles[], int lengths[]);

    // Gets the number of joints of the chain
    int numJoints();

    // Gets the IK chain object
    Chain* getChain();

 private:
    // Number of joints in the chain
    int _numJoints;
    // Tolerance of distance between end effector and target
    float _tolerance;
    // The chain containing joints
    Chain* _chain = nullptr;
    // Number of iterations to converge for last run (debugging only)
    int _num_iterations = 0;

    /* _createChain(lengths)
     * inputs: lengths
     *
     * Creates a new chain and attaches it to the Fabrik2D object
     *
     * length size should always be one lesser than the number of joints
     */
    void _createChain(int* lengths);

    /* _resetChain(lengths)
     * inputs: lengths
     *
     * Resets the chain to initial configuration
     *
     * length size should always be one lesser than the number of joints
     */
    void _resetChain(int lengths[]);

    /* _deleteChain()
     *
     * Deallocates _chain from memory
     *
     */
    void _deleteChain();

    /* distance(x1,y1,x2,y2)
     * inputs: coordinates
     * outputs: distance between points
     *
     * Uses euclidean distance
     */
    float _distance(float x1, float y1, float x2, float y2);
};

#endif  // SRC_FABRIK2D_H_
