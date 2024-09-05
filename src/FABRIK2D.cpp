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


#include "Arduino.h"
#include "FABRIK2D.h"

// Defined epsilon value which is considered as 0
#define EPSILON_VALUE 0.001


Fabrik2D::Fabrik2D(int numJoints, int lengths[], float tolerance) {
    begin(numJoints, lengths, tolerance);
}

Fabrik2D::~Fabrik2D() {
    _deleteChain();
}

void Fabrik2D::begin(int numJoints, int lengths[], float tolerance) {
    this->_numJoints = numJoints;
    _createChain(lengths);

    this->_tolerance = tolerance;
}

void Fabrik2D::_createChain(int lengths[]) {
    Chain* chain = new Chain();
    chain->joints = new Joint[this->_numJoints];

    this->_chain = chain;

    _resetChain(lengths);
}

void Fabrik2D::_resetChain(int lengths[]) {
    this->_chain->joints[0].x = 0;
    this->_chain->joints[0].y = 0;
    this->_chain->joints[0].angle = 0;

    int sumLengths = 0;
    for (int i = 1; i < this->_numJoints; i++) {
        sumLengths = sumLengths + lengths[i-1];
        this->_chain->joints[i].x = 0;
        this->_chain->joints[i].y = sumLengths;
        this->_chain->joints[i].angle = 0;
    }
}

void Fabrik2D::_deleteChain() {
    if (this->_chain->joints != nullptr) {
        delete[] this->_chain->joints;
        this->_chain->joints = nullptr;
    }

    if (this->_chain != nullptr) {
        delete this->_chain;
        this->_chain = nullptr;
    }
}

uint8_t Fabrik2D::solve(float x, float y, int lengths[]) {
    uint8_t result_status = 1;
    _num_iterations = 0;  // Used for debugging

    // Distance between root and target (root is always 0,0)
    int origin_to_target = sqrt(x*x+y*y);

    // Total length of chain
    int totalLength = 0;
    for (int i = 0; i < this->_numJoints-1; i++) {
        totalLength = totalLength + lengths[i];
    }

    // Check whether the target is within reach
    if (origin_to_target > totalLength) {
        // The target is unreachable

        for (int i = 0; i < this->_numJoints-1; i++) {
            // Find the distance r_i between the target (x,y) and the
            // joint i position (jx,jy)
            float jx = this->_chain->joints[i].x;
            float jy = this->_chain->joints[i].y;
            float r_i = _distance(jx, jy, x, y);
            float lambda_i = static_cast<float>(lengths[i])/r_i;

            // Find the new joint positions
            this->_chain->joints[i+1].x = static_cast<float>(
                (1-lambda_i)*jx + lambda_i*x);
            this->_chain->joints[i+1].y = static_cast<float>(
                (1-lambda_i)*jy + lambda_i*y);
        }

       _resetChain(lengths);
       return 0;
    } else {
        // The target is reachable; this, set as (bx,by) the initial
        // position of the joint i
        float bx = this->_chain->joints[0].x;
        float by = this->_chain->joints[0].y;

        // Check whether the distance between the end effector
        // joint n (ex,ey) and the target is greater than a tolerance
        float ex = this->_chain->joints[this->_numJoints-1].x;
        float ey = this->_chain->joints[this->_numJoints-1].y;
        float dist = _distance(ex, ey, x, y);

        float prevDist = 0;
        float tolerance = this->_tolerance;
        while (dist > tolerance) {
            _num_iterations++;
            // If previous error is the same as current error,
            // Tolerances might be too low. Increase tolerances and try again
            if (abs(prevDist - dist) < EPSILON_VALUE) {
                result_status = 2;
                // Increase tolerance by 10%
                tolerance *= 1.1;
                // If increased tolerance is higher than 2x the desired
                // tolerance report failed to converge
                if (tolerance > this->_tolerance*2) {
                    _resetChain(lengths);
                    return 0;
                }
            }

            prevDist = dist;

            // STAGE 1: FORWARD REACHING
            // Set the end effector as target
            this->_chain->joints[this->_numJoints-1].x = x;
            this->_chain->joints[this->_numJoints-1].y = y;

            for (int i = this->_numJoints-2; i >= 0; i--) {
                // Find the distance r_i between the new joint position
                // i+1 (nx,ny) and the joint i (jx,jy)
                float jx = this->_chain->joints[i].x;
                float jy = this->_chain->joints[i].y;
                float nx = this->_chain->joints[i+1].x;
                float ny = this->_chain->joints[i+1].y;
                float r_i = _distance(jx, jy, nx, ny);
                float lambda_i = static_cast<float>(lengths[i])/r_i;

                // Find the new joint positions
                this->_chain->joints[i].x = static_cast<float>(
                    (1-lambda_i)*nx + lambda_i*jx);
                this->_chain->joints[i].y = static_cast<float>(
                    (1-lambda_i)*ny + lambda_i*jy);
            }

            // STAGE 2: BACKWARD REACHING
            // Set the root at its initial position
            this->_chain->joints[0].x = bx;
            this->_chain->joints[0].y = by;

            for (int i = 0; i < this->_numJoints-1; i++) {
                // Find the distance r_i between the new joint position
                // i (nx,ny) and the joint i+1 (jx,jy)
                float jx = this->_chain->joints[i+1].x;
                float jy = this->_chain->joints[i+1].y;
                float nx = this->_chain->joints[i].x;
                float ny = this->_chain->joints[i].y;
                float r_i = _distance(jx, jy, nx, ny);
                float lambda_i = static_cast<float>(lengths[i])/r_i;

                // Find the new joint positions
                this->_chain->joints[i+1].x = static_cast<float>(
                    (1-lambda_i)*nx + lambda_i*jx);
                this->_chain->joints[i+1].y = static_cast<float>(
                    (1-lambda_i)*ny + lambda_i*jy);
            }

            // Update distance between end effector and target
            ex = this->_chain->joints[this->_numJoints-1].x;
            ey = this->_chain->joints[this->_numJoints-1].y;
            dist = _distance(ex, ey, x, y);
        }
    }

    this->_chain->joints[0].angle = atan2(
        this->_chain->joints[1].y, this->_chain->joints[1].x);

    float prevAngle = this->_chain->joints[0].angle;

    for (int i = 2; i <= this->_numJoints-1; i++) {
        float ax = this->_chain->joints[i-1].x;
        float ay = this->_chain->joints[i-1].y;
        float bx = this->_chain->joints[i].x;
        float by = this->_chain->joints[i].y;

        float aAngle = atan2(by-ay, bx-ax);

        this->_chain->joints[i-1].angle = aAngle - prevAngle;

        prevAngle = aAngle;
    }

    return result_status;
}

uint8_t Fabrik2D::solve2(
    float x, float y, float z,
    float toolAngle,
    float grippingOffset,
    int lengths[]
) {
    uint8_t result_status = 0;

    if (this->_numJoints >= 4) {
        // Solve in 2D plane
        float r = _distance(0, 0, x, z);

        // Find wrist center by moving from the desired position with
        // tool angle and link length
        float oc_r = r - (
            lengths[this->_numJoints-2]+grippingOffset)*cos(toolAngle);

        float oc_y = y - (
            lengths[this->_numJoints-2]+grippingOffset)*sin(toolAngle);

        // We solve IK from first joint to wrist center
        int tmp = this->_numJoints;
        this->_numJoints = this->_numJoints-1;

        result_status = solve(oc_r, oc_y, lengths);

        this->_numJoints = tmp;

        if (result_status == 1) {
            // Update the end effector position to preserve tool angle
            this->_chain->joints[this->_numJoints-1].x =
                this->_chain->joints[this->_numJoints-2].x
                + lengths[this->_numJoints-2]*cos(toolAngle);

            this->_chain->joints[this->_numJoints-1].y =
                this->_chain->joints[this->_numJoints-2].y
                + lengths[this->_numJoints-2]*sin(toolAngle);

            // Update angle of last joint
            this->_chain->joints[0].angle = atan2(
                this->_chain->joints[1].y,
                this->_chain->joints[1].x);

            float prevAngle = this->_chain->joints[0].angle;
            for (int i = 2; i <= this->_numJoints-1; i++) {
                float ax = this->_chain->joints[i-1].x;
                float ay = this->_chain->joints[i-1].y;
                float bx = this->_chain->joints[i].x;
                float by = this->_chain->joints[i].y;

                float aAngle = atan2(by-ay, bx-ax);

                this->_chain->joints[i-1].angle = aAngle - prevAngle;

                prevAngle = aAngle;
            }

            // Save tool angle
            this->_chain->joints[this->_numJoints-1].angle = toolAngle;

            // Save base angle
            this->_chain->z = z;
            this->_chain->angle = atan2(z, x);

            // Update joint X values based on base rotation
            for (int i = 0; i <= this->_numJoints-1; i++) {
                this->_chain->joints[i].x =
                    this->_chain->joints[i].x * cos(-this->_chain->angle);
            }
        }
    }

    return result_status;
}

uint8_t Fabrik2D::solve(float x, float y, float toolAngle, int lengths[]) {
    return solve2(x, y, 0, toolAngle, 0, lengths);
}

uint8_t Fabrik2D::solve(
    float x, float y,
    float toolAngle,
    float grippingOffset,
    int lengths[]
) {
    return solve2(x, y, 0, toolAngle, grippingOffset, lengths);
}

uint8_t Fabrik2D::solve2(float x, float y, float z, int lengths[]) {
    float r = _distance(0, 0, x, z);

    uint8_t result_status =  solve(r, y, lengths);
    if (result_status == 1) {
        this->_chain->z = z;
        this->_chain->angle = atan2(z, x);

        // Update joint X values based on base rotation
        for (int i = 0; i <= this->_numJoints-1; i++) {
            this->_chain->joints[i].x =
                this->_chain->joints[i].x * cos(-this->_chain->angle);
        }
    }

    return result_status;
}

uint8_t Fabrik2D::solve2(
    float x, float y, float z,
    float toolAngle,
    int lengths[]
) {
    return solve2(x, y, z, toolAngle, 0, lengths);
}

float Fabrik2D::getX(int joint) {
  if (joint >= 0 && joint < this->_numJoints) {
      return this->_chain->joints[joint].x;
  }
  return 0;
}

float Fabrik2D::getY(int joint) {
  if (joint >= 0 && joint < this->_numJoints) {
      return this->_chain->joints[joint].y;
  }
  return 0;
}

float Fabrik2D::getAngle(int joint) {
  if (joint >= 0 && joint < this->_numJoints) {
      return this->_chain->joints[joint].angle;
  }
  return 0;
}

float Fabrik2D::getZ(int joint) {
    return this->_chain->z;
}

float Fabrik2D::getBaseAngle() {
    return this->_chain->angle;
}

void Fabrik2D::setBaseAngle(float baseAngle) {
    float angle_diff = baseAngle - this->_chain->angle;
    this->_chain->angle = baseAngle;

    if (this->_numJoints >= 4) {
        // Update end effector Z value based on base rotation
        this->_chain->z =
            this->_chain->joints[this->_numJoints-1].x * sin(angle_diff);
        // Update joint X values based on base rotation
        for (int i = 0; i <= this->_numJoints-1; i++) {
            this->_chain->joints[i].x =
                this->_chain->joints[i].x * cos(angle_diff);
        }
    }
}

float Fabrik2D::getTolerance() {
    return this->_tolerance;
}

void Fabrik2D::setTolerance(float tolerance) {
    this->_tolerance = tolerance;
}

void Fabrik2D::setJoints(float angles[], int lengths[]) {
    float accAng = angles[0];
    float accX = 0;
    float accY = 0;
    this->_chain->joints[0].angle = angles[0];

    for (int i = 1; i < this->_numJoints-1; i++) {
        this->_chain->joints[i].x = accX + lengths[i-1]*cos(accAng);
        this->_chain->joints[i].y = accY + lengths[i-1]*sin(accAng);
        this->_chain->joints[i].angle = angles[i];
        accAng += angles[i];
        accX = this->_chain->joints[i].x;
        accY = this->_chain->joints[i].y;
    }

    // Update end effector x and y
    this->_chain->joints[this->_numJoints-1].x =
        accX + lengths[this->_numJoints-2]*cos(accAng);
    this->_chain->joints[this->_numJoints-1].y =
        accY + lengths[this->_numJoints-2]*sin(accAng);
    this->_chain->joints[this->_numJoints-1].angle = 0;
}

int Fabrik2D::numJoints() {
    return this->_numJoints;
}

Fabrik2D::Chain* Fabrik2D::getChain() {
    return this->_chain;
}

float Fabrik2D::_distance(float x1, float y1, float x2, float y2) {
    float xDiff = x2 - x1;
    float yDiff = y2 - y1;
    return sqrt(xDiff*xDiff + yDiff*yDiff);
}
