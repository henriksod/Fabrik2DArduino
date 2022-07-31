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
};

class VectorInt16 {
 public:
    int16_t x;
    int16_t y;
    int16_t z;

    VectorInt16() {
        x = 0;
        y = 0;
        z = 0;
    }

    VectorInt16(int16_t nx, int16_t ny, int16_t nz) {
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

    VectorInt16 getNormalized() const {
        VectorInt16 r(x, y, z);
        r.normalize();
        return r;
    }

    // Gets the cross product between vector v and this vector
    VectorInt16 crossProduct(const VectorInt16& v) const {
        VectorInt16 cv;
        cv.x = v.y * this->z - v.z * this->y;
        cv.y = -(v.x * this->z - v.z * this->x);
        cv.z = v.x * this->y - v.y * this->x;

        return cv
    }

    // Gets the dot product between vector v and this vector
    float dotProduct(const VectorInt16& v) const {
        return v.x*this-> x + v.y*this->y + v.z*this->z;
    }

    void rotate(const Quaternion& q) {
        // P_out = q * P_in * conj(q)
        // - P_out is the output vector
        // - q is the orientation quaternion
        // - P_in is the input vector (a*aReal)
        // - conj(q) is the conjugate of the orientation
        //   quaternion (q=[w,x,y,z], q*=[w,-x,-y,-z])
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

    VectorInt16 getRotated(const Quaternion& q) const {
        VectorInt16 r(x, y, z);
        r.rotate(q);
        return r;
    }

    // Gets the quaternion from vector v to this vector
    Quaternion getRotationFrom(VectorInt16 *v) const {
        VectorInt16 a = this->crossProduct(v);

        float fromLength = v->getMagnitude();
        float toLength = this->getMagnitude();

        float w = sqrt(fromLength*fromLength + toLength*toLength)
            + this->dotProduct(v);

        Quaternion q(w, a.x, a.y, a.z);

        return q;
    }

    // Produces the difference of this vector and v.
    VectorInt16 operator-(const VectorInt16& v) const {
        return VectorInt16(this->x-v.x, this->y-v.y, this->z-v.z);
    }

    // Produces the sum of this vector and v.
    VectorInt16 operator+(const VectorInt16& v) const {
        return VectorInt16(this->x+v.x, this->y+v.y, this->z+v.z);
    }
};

class VectorFloat {
 public:
    float x;
    float y;
    float z;

    VectorFloat() {
        x = 0;
        y = 0;
        z = 0;
    }

    VectorFloat(float nx, float ny, float nz) {
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

    VectorFloat getNormalized() const {
        VectorFloat r(x, y, z);
        r.normalize();
        return r;
    }

    // Gets the cross product between vector v and this vector
    VectorFloat crossProduct(const VectorFloat& v) const {
        VectorFloat cv;
        cv.x = v.y * this->z - v.z * this->y;
        cv.y = -(v.x * this->z - v.z * this->x);
        cv.z = v.x * this->y - v.y * this->x;

        return cv
    }

    // Gets the dot product between vector v and this vector
    float dotProduct(const VectorFloat& v) const {
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

    VectorFloat getRotated(const Quaternion& q) const {
        VectorFloat r(x, y, z);
        r.rotate(q);
        return r;
    }

    // Gets the quaternion from vector v to this vector
    Quaternion getRotationFrom(const VectorFloat& v) const {
        VectorFloat a = this->crossProduct(v);

        float fromLength = v.getMagnitude();
        float toLength = this->getMagnitude();

        float w = sqrt(fromLength*fromLength + toLength*toLength)
            + this->dotProduct(v);

        Quaternion q(w, a.x, a.y, a.z);
        q.normalize();

        return q;
    }

    // Produces the difference of this vector and v.
    VectorFloat operator-(const VectorFloat& v) const {
        return VectorFloat(this->x-v.x, this->y-v.y, this->z-v.z);
    }

    // Produces the sum of this vector and v.
    VectorFloat operator+(const VectorFloat& v) const {
        return VectorFloat(this->x+v.x, this->y+v.y, this->z+v.z);
    }

    // Produces the dot product of this vector and v.
    float operator*(const VectorFloat& v) const {
      return this->dotProduct(v);
    }
};

#endif  // SRC_HELPER_3DMATH_H_
