/**********************************************************************************************
 * FABRIK 2D inverse kinematics solver - Version 1.0.5
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
     * inputs: x and y positions of target, desired tool angle and lengths
     *         between each joint
     * outputs: True if solvable, false if not solvable
     *
     * !!! tool angle is in radians !!!
     *
     * solves the inverse kinematics of the stored chain to reach the target
     * with tool angle
     *
     * will only work for 3DOF
     */
    bool solve(float x, float y, float toolAngle, int* lengths);

    /* solve(x, y, angle, offset, lengths)
     * inputs: x and y positions of target, desired tool angle and lengths
     *         between each joint
     * outputs: True if solvable, false if not solvable
     *
     * !!! tool angle is in radians !!!
     *
     * solves the inverse kinematics of the stored chain to reach the target
     * with tool angle 
     * and gripping offset
     *
     * will only work for 3DOF
     */
    bool solve(
        float x, float y,
        float toolAngle,
        float grippingOffset,
        int* lengths);

    /* solve2(x, y, z, lengths)
     * inputs: x, y and z positions of target, desired tool angle and lengths
     *         between each joint
     * outputs: True if solvable, false if not solvable
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
    bool solve2(float x, float y, float z, int* lengths);

    /* solve2(x, y, z, toolAngle, lengths)
     * inputs: x, y and z positions of target, desired tool angle and lengths
     *         between each joint
     * outputs: True if solvable, false if not solvable
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
     */
    bool solve2(float x, float y, float z, float toolAngle, int* lengths);

    /* solve2(x, y, z, angle, offset, lengths)
     * inputs: x, y and z positions of target, desired tool angle, gripping
               offset and lengths between each joint
     * outputs: True if solvable, false if not solvable
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
    bool solve2(
        float x, float y, float z,
        float toolAngle,
        float grippingOffset,
        int* lengths);

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

    /* getZ()
     * outputs: z offset of the chain from the plane
     */
    float getZ();

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

    /* setTolerance(tolerance)
     * inputs: tolerance value
     * 
     * sets the tolerance of the distance between the end effector
     * and the target
     */
    void setTolerance(float tolerance);

    /* setJoints(angles, lengths)
     * inputs: New joint angles(in radians) and list of lengths
     *         between each joint
     * 
     * manually sets the joint angles and updates their position using
     * forward kinematics
     */
    void setJoints(float* angles, int* lengths);

 private:
    // Joint struct
    typedef struct {
        float x;  // x position of joint relative to origin
        float y;  // y position of joint relative to origin
        float angle;  // angle of joint
    } Joint;

    // Chain struct
    typedef struct {
      Joint* joints;  // list of joints
      float z;  // z position defining the offset of the chain from the plane
      float angle;  // base (plane) rotation
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

#endif  // SRC_FABRIK2D_H_
