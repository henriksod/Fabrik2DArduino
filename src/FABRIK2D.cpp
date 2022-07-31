/*******************************************************************************
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
 ******************************************************************************/


#include "FABRIK2D.h"
#include "helper_3dmath.h"  // NOLINT

// Defined epsilon value which is considered as 0
#define EPSILON_VALUE 0.001


template<typename T = VectorFloat>
Fabrik2D<T>::Fabrik2D(int numJoints, int lengths[], float tolerance) {
    begin(numJoints, lengths, tolerance);
}

template<typename T = VectorFloat>
Fabrik2D<T>::~Fabrik2D() {
    _deleteChain();
}

template<typename T = VectorFloat>
void Fabrik2D<T>::begin(int numJoints, int lengths[], float tolerance) {
    this->_numJoints = numJoints;
    _createChain(lengths);

    this->_tolerance = tolerance;
}

template<typename T = VectorFloat>
void Fabrik2D<T>::_createChain(int lengths[]) {
    Chain* chain = new Chain();
    chain->joints = new Joint[this->_numJoints];

    this->_chain = chain;

    this->_chain->p = new T();
    this->_chain->q = new Quaternion();

    this->_chain->joints[0].p = new T();
    this->_chain->joints[0].q = new Quaternion();

    int sumLengths = 0;
    for (int i = 1; i < this->_numJoints; i++) {
        sumLengths = sumLengths + lengths[i-1];
        this->_chain->joints[i].p = new T(0, sumLengths, 0);
        this->_chain->joints[i].q = new Quaternion();
    }
}

template<typename T = VectorFloat>
void Fabrik2D<T>::_resetChain(int lengths[]) {
    *(this->_chain->p) = T();
    *(this->_chain->q) = Quaternion();

    *(this->_chain->joints[0].p) = T();
    *(this->_chain->joints[0].q) = Quaternion();

    int sumLengths = 0;
    for (int i = 1; i < this->_numJoints; i++) {
        sumLengths = sumLengths + lengths[i-1];
        *(this->_chain->joints[i].p) = T(0, sumLengths, 0);
        *(this->_chain->joints[i].q) = Quaternion();
    }
}

template<typename T = VectorFloat>
void Fabrik2D<T>::_deleteChain() {
    if (this->_chain->joints != nullptr) {
        for (int i = 0; i < this->_numJoints; i++) {
            if (this->_chain->joints[i].p != nullptr) {
                delete this->_chain->joints[i].p;
                this->_chain->joints[i].p = nullptr;
            }
            if (this->_chain->joints[i].q != nullptr) {
                delete this->_chain->joints[i].q;
                this->_chain->joints[i].q = nullptr;
            }
        }
        delete[] this->_chain->joints;
        this->_chain->joints = nullptr;
    }

    if (this->_chain != nullptr) {
        if (this->_chain->p != nullptr) {
            delete this->_chain->p;
            this->_chain->p = nullptr;
        }
        if (this->_chain->q != nullptr) {
            delete this->_chain->q;
            this->_chain->q = nullptr;
        }
        delete this->_chain;
        this->_chain = nullptr;
    }
}

template<typename T = VectorFloat>
uint8_t Fabrik2D<T>::solve(float x, float y, int lengths[]) {
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
            float jx = this->_chain->joints[i].p.x;
            float jy = this->_chain->joints[i].p.y;
            float r_i = _distance(jx, jy, x, y);
            float lambda_i = static_cast<float>(lengths[i])/r_i;

            // Find the new joint positions
            this->_chain->joints[i+1].p.x = static_cast<float>(
                (1-lambda_i)*jx + lambda_i*x);
            this->_chain->joints[i+1].p.y = static_cast<float>(
                (1-lambda_i)*jy + lambda_i*y);
        }

       _resetChain(lengths);
       return 0;
    } else {
        // The target is reachable; this, set as (bx,by) the initial
        // position of the joint i
        float bx = this->_chain->joints[0].p.x;
        float by = this->_chain->joints[0].p.y;

        // Check whether the distance between the end effector
        // joint n (ex,ey) and the target is greater than a tolerance
        float ex = this->_chain->joints[this->_numJoints-1].p.x;
        float ey = this->_chain->joints[this->_numJoints-1].p.y;
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
            this->_chain->joints[this->_numJoints-1].p.x = x;
            this->_chain->joints[this->_numJoints-1].p.y = y;

            for (int i = this->_numJoints-2; i >= 0; i--) {
                // Find the distance r_i between the new joint position
                // i+1 (nx,ny) and the joint i (jx,jy)
                float jx = this->_chain->joints[i].p.x;
                float jy = this->_chain->joints[i].p.y;
                float nx = this->_chain->joints[i+1].p.x;
                float ny = this->_chain->joints[i+1].p.y;
                float r_i = _distance(jx, jy, nx, ny);
                float lambda_i = static_cast<float>(lengths[i])/r_i;

                // Find the new joint positions
                this->_chain->joints[i].p.x = static_cast<float>(
                    (1-lambda_i)*nx + lambda_i*jx);
                this->_chain->joints[i].p.y = static_cast<float>(
                    (1-lambda_i)*ny + lambda_i*jy);
            }

            // STAGE 2: BACKWARD REACHING
            // Set the root at its initial position
            this->_chain->joints[0].p.x = bx;
            this->_chain->joints[0].p.y = by;

            for (int i = 0; i < this->_numJoints-1; i++) {
                // Find the distance r_i between the new joint position
                // i (nx,ny) and the joint i+1 (jx,jy)
                float jx = this->_chain->joints[i+1].p.x;
                float jy = this->_chain->joints[i+1].p.y;
                float nx = this->_chain->joints[i].p.x;
                float ny = this->_chain->joints[i].p.y;
                float r_i = _distance(jx, jy, nx, ny);
                float lambda_i = static_cast<float>(lengths[i])/r_i;

                // Find the new joint positions
                this->_chain->joints[i+1].p.x = static_cast<float>(
                    (1-lambda_i)*nx + lambda_i*jx);
                this->_chain->joints[i+1].p.y = static_cast<float>(
                    (1-lambda_i)*ny + lambda_i*jy);
            }

            // Update distance between end effector and target
            ex = this->_chain->joints[this->_numJoints-1].p.x;
            ey = this->_chain->joints[this->_numJoints-1].p.y;
            dist = _distance(ex, ey, x, y);
        }
    }

    // Calculate quaternions from result positions
    *(this->_chain->joints[0].q) = this->_chain->joints[0].p->getNormalized()
                                        .getRotationFrom(_origin);

    T from = this->_chain->joints[0].p->getNormalized();
    for (int i = 1; i < this->_numJoints; i++) {
        T to = (
            this->_chain->joints[i].p->getNormalized() - from).getNormalized();

        *this->_chain->joints[i].q =
            to.getNormalized().getRotationFrom(from);

        from = to;
    }

    return result_status;
}

template<typename T = VectorFloat>
uint8_t Fabrik2D<T>::solve2(
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
        T oc;
        oc.x = -(lengths[this->_numJoints-2] + grippingOffset);

        Quaternion toolRotation(toolAngle, 0, 0, 1);
        oc.rotate(toolRotation);

        oc.x += r;
        oc.y += y;

        // We solve IK from first joint to wrist center
        int tmp = this->_numJoints;
        this->_numJoints = this->_numJoints-1;

        result_status = solve(oc_r, oc_y, lengths);

        this->_numJoints = tmp;

        if (result_status == 1) {
            T oc_to_end;
            oc_to_end.x = lengths[this->_numJoints-2];
            oc_to_end.rotate(toolRotation);

            // Update the end effector position to preserve tool angle
            *this->_chain->joints[this->_numJoints-1].p =
                *this->_chain->joints[this->_numJoints-2].p + oc_to_end;

            // Calculate quaternions from result positions
            *this->_chain->joints[0].q = this->_chain->joints[0].p->
                                    getNormalized().getRotationFrom(_origin);

            T from = this->_chain->joints[0].p->getNormalized();
            for (int i = 1; i < this->_numJoints; i++) {
                T to = (this->_chain->joints[i].p->getNormalized()
                        - from).getNormalized();

                *this->_chain->joints[i].q =
                    to.getNormalized().getRotationFrom(from);

                from = to;
            }

            // Save base angle
            T desiredPlaneVector(x, 0, z);
            *this->_chain->q = desiredPlaneVector.getRotationFrom(_origin);

            // Update joint positions based on base rotation
            for (int i = 0; i <= this->_numJoints-1; i++) {
                this->_chain->joints[i].p =
                    this->_chain->joints[i].p->
                        getRotated(*this->_chain->q);
            }
        }
    }

    return result_status;
}

template<typename T = VectorFloat>
uint8_t Fabrik2D<T>::solve(float x, float y, float toolAngle, int lengths[]) {
    return solve2(x, y, 0, toolAngle, 0, lengths);
}

template<typename T = VectorFloat>
uint8_t Fabrik2D<T>::solve(
    float x, float y,
    float toolAngle,
    float grippingOffset,
    int lengths[]
) {
    return solve2(x, y, 0, toolAngle, grippingOffset, lengths);
}

template<typename T = VectorFloat>
uint8_t Fabrik2D<T>::solve2(float x, float y, float z, int lengths[]) {
    float r = _distance(0, 0, x, z);

    uint8_t result_status =  solve(r, y, lengths);
    if (result_status == 1) {
        // Save base angle
        T desiredPlaneVector(x, 0, z);
        *this->_chain->q = desiredPlaneVector.getRotationFrom(_origin);

        // Update joint positions based on base rotation
        for (int i = 0; i <= this->_numJoints-1; i++) {
            this->_chain->joints[i].p =
                this->_chain->joints[i].p->
                    getRotated(*this->_chain->q);
        }
    }

    return result_status;
}

template<typename T = VectorFloat>
uint8_t Fabrik2D<T>::solve2(
    float x, float y, float z,
    float toolAngle,
    int lengths[]
) {
    return solve2(x, y, z, toolAngle, 0, lengths);
}

template<typename T = VectorFloat>
float Fabrik2D<T>::getX(int joint) {
  if (joint >= 0 && joint < this->_numJoints) {
      return this->_chain->joints[joint].p.x;
  }
  return this->_chain->joints[this->_numJoints-1].p.x;
}

template<typename T = VectorFloat>
float Fabrik2D<T>::getY(int joint) {
  if (joint >= 0 && joint < this->_numJoints) {
      return this->_chain->joints[joint].p.y;
  }
  return this->_chain->joints[this->_numJoints-1].p.y;
}

template<typename T = VectorFloat>
float Fabrik2D<T>::getZ(int joint) {
    if (joint >= 0 && joint < this->_numJoints) {
        return this->_chain->joints[joint].p.z;
    }
    return this->_chain->joints[this->_numJoints-1].p.z;
}

template<typename T = VectorFloat>
float Fabrik2D<T>::getAngle(int joint) {
  if (joint >= 0 && joint < this->_numJoints) {
      Quaternion q = *(this->_chain->joints[joint].q);
      return atan2(2*q.y*q.z - 2*q.w*q.x, 2*q.w*q -> w.2*q.z*q.z - 1);
  }
  Quaternion q = *(this->_chain->joints[this->_numJoints-1].q);
  return atan2(2*q.y*q.z - 2*q.w*q.x, 2*q.w*q -> w.2*q.z*q.z - 1);
}

template<typename T = VectorFloat>
float Fabrik2D<T>::getBaseAngle() {
    Quaternion q = *(this->_chain->q);
    return -asin(2*q.x*q.z + 2*q.w*q.y);
}

template<typename T = VectorFloat>
void Fabrik2D<T>::setBaseAngle(float baseAngle) {
    // Save base angle
    T desiredPlaneVector(x, 0, z);
    *this->_chain->q = desiredPlaneVector.getRotationFrom(_origin);

    // Rotate back joints from previous base rotation
    for (int i = 0; i <= this->_numJoints-1; i++) {
        this->_chain->joints[i].p =
            this->_chain->joints[i].p->
                getRotated(*this->_chain->q->getConjugate());
    }

    // Update base rotation
    Quaternion q(baseAngle, 0, 1, 0);
    *this->_chain->q = q;

    // Rotate joints to new base rotation
    for (int i = 0; i <= this->_numJoints-1; i++) {
        this->_chain->joints[i].p =
            this->_chain->joints[i].p->
                getRotated(*this->_chain->q);
    }
}

template<typename T = VectorFloat>
float Fabrik2D<T>::getTolerance() {
    return this->_tolerance;
}

template<typename T = VectorFloat>
void Fabrik2D<T>::setTolerance(float tolerance) {
    this->_tolerance = tolerance;
}

template<typename T = VectorFloat>
void Fabrik2D<T>::setJoints(float angles[], int lengths[]) {
    // Calculate quaternions from input angles
    Quaternion q0(angles[0], 0, 0, 1);
    *this->_chain->joints[0].q = q0;

    for (int i = 1; i < this->_numJoints-1; i++) {
        Quaternion q(angles[i], 0, 0, 1);

        *this->_chain->joints[i].q = q;
    }

    T accumVector;
    Quaternion accumQuaternion;

    for (int i = 1; i < this->_numJoints; i++) {
        T v(lengths[i-1], 0, 0);
        accumQuaternion.getProduct(*this->_chain->joints[i-1].q);
        v.rotate(accumQuaternion);
        accumVector += v;
        *this->_chain->joints[i].p = accumVector;
    }
}

template<typename T = VectorFloat>
int Fabrik2D<T>::numJoints() {
    return this->_numJoints;
}

template<typename T = VectorFloat>
Fabrik2D<T>::Chain* Fabrik2D<T>::getChain() {
    return this->_chain;
}

template<typename T = VectorFloat>
float Fabrik2D<T>::_distance(float x1, float y1, float x2, float y2) {
    float xDiff = x2 - x1;
    float yDiff = y2 - y1;
    return sqrt(xDiff*xDiff + yDiff*yDiff);
}
