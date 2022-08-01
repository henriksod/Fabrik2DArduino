/*******************************************************************************
* I2Cdev device library code is placed under the MIT license
* Copyright (c) 2012 Jeff Rowberg

* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:

* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
 ******************************************************************************/

#ifndef SRC_HELPER_3DMATH_H_
#define SRC_HELPER_3DMATH_H_

#include "Arduino.h"

namespace helper_3dmath {
class Quaternion {
 public:
    float w;
    float x;
    float y;
    float z;

    Quaternion() {
        w = 1.0f;
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
    }

    Quaternion(float nw, float nx, float ny, float nz) {
        w = nw;
        x = nx;
        y = ny;
        z = nz;
    }

    Quaternion getProduct(const Quaternion& q) const {
        // Quaternion multiplication is defined by:
        //     (Q1 * Q2).w = (w1w2 - x1x2 - y1y2 - z1z2)
        //     (Q1 * Q2).x = (w1x2 + x1w2 + y1z2 - z1y2)
        //     (Q1 * Q2).y = (w1y2 - x1z2 + y1w2 + z1x2)
        //     (Q1 * Q2).z = (w1z2 + x1y2 - y1x2 + z1w2
        return Quaternion(
            w*q.w - x*q.x - y*q.y - z*q.z,  // new w
            w*q.x + x*q.w + y*q.z - z*q.y,  // new x
            w*q.y - x*q.z + y*q.w + z*q.x,  // new y
            w*q.z + x*q.y - y*q.x + z*q.w);  // new z
    }

    Quaternion getConjugate() const {
        return Quaternion(w, -x, -y, -z);
    }

    float getMagnitude() const {
        return sqrt(w*w + x*x + y*y + z*z);
    }

    void normalize() {
        float m = getMagnitude();
        w /= m;
        x /= m;
        y /= m;
        z /= m;
    }

    Quaternion getNormalized() const {
        Quaternion r(w, x, y, z);
        r.normalize();
        return r;
    }

    Quaternion fromAxis(float angle, float x, float y, float z) {
        return Quaternion(
            cos(angle/2),
            x*sin(angle/2),
            y*sin(angle/2),
            z*sin(angle/2)).getNormalized();
    }
};

template<typename T = float>
class Vector {
 public:
    T x;
    T y;
    T z;

    Vector() {
        x = 0;
        y = 0;
        z = 0;
    }

    Vector(T nx, T ny, T nz) {
        x = nx;
        y = ny;
        z = nz;
    }

    float getMagnitude() const {
        return sqrt(x*x + y*y + z*z);
    }

    void normalize() {
        float m = getMagnitude();
        x /= m;
        y /= m;
        z /= m;
    }

    Vector getNormalized() const {
        Vector r(x, y, z);
        r.normalize();
        return r;
    }

    // Gets the cross product between vector v and this vector
    Vector crossProduct(const Vector& v) const {
        Vector cv;
        cv.x = v.y * this->z - v.z * this->y;
        cv.y = -(v.x * this->z - v.z * this->x);
        cv.z = v.x * this->y - v.y * this->x;

        return cv;
    }

    // Gets the dot product between vector v and this vector
    float dotProduct(const Vector& v) const {
        return v.x*this-> x + v.y*this->y + v.z*this->z;
    }

    void rotate(const Quaternion& q) {
        Quaternion p(0, x, y, z);

        // quaternion multiplication: q * p, stored back in p
        p = q.getProduct(p);

        // quaternion multiplication: p * conj(q), stored back in p
        p = p.getProduct(q.getConjugate());

        // p quaternion is now [0, x', y', z']
        x = p.x;
        y = p.y;
        z = p.z;
    }

    Vector getRotated(const Quaternion& q) const {
        Vector r(x, y, z);
        r.rotate(q);
        return r;
    }

    // Gets the quaternion from vector v to this vector
    Quaternion getRotationFrom(const Vector& v) const {
        float d = this->dotProduct(v);
        Vector w = this->crossProduct(v);

        return Quaternion(d + sqrt(d * d + w * w), w.x, w.y, w.z)
                    .getNormalized();
    }

    // Produces the difference of this vector and v.
    Vector operator-(const Vector& v) const {
        return Vector(this->x-v.x, this->y-v.y, this->z-v.z);
    }

    // Produces the difference of this vector and v.
    Vector& operator-=(const Vector& v) {
        this->x -= v.x;
        this->y -= v.y;
        this->z -= v.z;
        return *this;
    }

    // Produces the sum of this vector and v.
    Vector operator+(const Vector& v) const {
        return Vector(this->x+v.x, this->y+v.y, this->z+v.z);
    }

    // Produces the sum of this vector and v.
    Vector& operator+=(const Vector& v) {
        this->x += v.x;
        this->y += v.y;
        this->z += v.z;
        return *this;
    }

    // Produces the dot product of this vector and v.
    float operator*(const Vector& v) const {
        return this->dotProduct(v);
    }

    // Produces the scaled vector with s.
    Vector operator*(const float& s) const {
        return Vector(this->x*s, this->y*s, this->z*s);
    }

    // Produces the scaled vector with s.
    Vector& operator*=(const float& s) {
        this->x *= s;
        this->y *= s;
        this->z *= s;
        return *this;
    }
};

template<typename T = float>
Vector<T> eulerFromQuaternion(const Quaternion& q) {
    Vector<T> r;

    float sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    float cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    r.x = atan2(sinr_cosp, cosr_cosp);

    float sinp = 2 * (q.w * q.y - q.z * q.x);
    if (abs(sinp) >= 1)
        r.y = (PI/2)*sinp/abs(sinp);
    else
        r.y = asin(sinp);

    float siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    r.z = atan2(siny_cosp, cosy_cosp);

    return r;
}
}  // namespace helper_3dmath

#endif  // SRC_HELPER_3DMATH_H_
