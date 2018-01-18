/**********************************************************************************************
 * FABRIK 2D inverse kinematics solver - Version 0.1
 * by Henrik Söderlund <henrik.a.soderlund@gmail.com>
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/


#include "Arduino.h"
#include "Fabrik2D.h"

/* Fabrik2D(numJoints, lengths)
 * inputs: numJoints, lengths
 *
 * creates the chain to be used for the inverse kinematics solver
 */
Fabrik2D::Fabrik2D(int numJoints, int* lengths)
{
  
  this->numJoints = numJoints;
  createChain(lengths);
  
  this->tolerance = 1; // 1mm tolerance default
}

/* createChain(lengths)
 * inputs: lengths
 *
 * length size should always be one lesser than the number of joints
 */
void Fabrik2D::createChain(int* lengths)
{
    if (sizeof(lengths)/sizeof(int)+1 < this->numJoints-1) {
        return;
    }

    Chain* chain = (Chain*)malloc(sizeof(Chain));
    chain->joints = (Joint*)malloc(sizeof(Joint)*this->numJoints);

    chain->joints[0].x = 0;
    chain->joints[0].y = 0;
    chain->joints[0].angle = 0;
      
    int sumLengths = 0;
    for (int i = 1; i < this->numJoints; i++) 
    {
        sumLengths = sumLengths + lengths[i-1];
        chain->joints[i].x = 0;
        chain->joints[i].y = sumLengths;
        chain->joints[i].angle = 0;
    }

    this->chain = chain;
}

/* solve(x, y, lengths)
 * inputs: x and y positions of target, lengths between each joint
 *
 * solves the inverse kinematics of the stored chain to reach the target
 */
void Fabrik2D::solve(float x, float y, int* lengths)
{
    if (sizeof(lengths)/sizeof(int)+1 < this->numJoints-1) {
        return;
    }

    // Distance between root and target (root is always 0,0)
    int dist = abs(x) + abs(y);
    
    // Total length of chain
    int totalLength = 0;
    for (int i = 0; i < this->numJoints-1; i++)
    {
        totalLength = totalLength + lengths[i];
    }

    
    // Check whether the target is within reach
    if (dist > totalLength) 
    {
       // The target is unreachable
       for (int i = 0; i < this->numJoints-1; i++)
       {
           // Find the distance r_i between the target (x,y) and the joint i position (jx,jy)
           float jx = this->chain->joints[i].x;
           float jy = this->chain->joints[i].y;
           float r_i = abs(x-jx) + abs(y-jy);
           float lambda_i = ((float)lengths[i])/r_i;
           
           // Find the new joint positions
           this->chain->joints[i+1].x = (float)((1-lambda_i)*jx + lambda_i*x);
           this->chain->joints[i+1].y = (float)((1-lambda_i)*jy + lambda_i*y);
       }
    }
    else 
    {
        // The target is reachable; this, set as (bx,by) the initial position of the joint i
        float bx = this->chain->joints[0].x;
        float by = this->chain->joints[0].y;
        
        // Check whether the distance between the end effector joint n (ex,ey) and the target is 
        // greater than a tolerance
        float ex = this->chain->joints[this->numJoints-1].x;
        float ey = this->chain->joints[this->numJoints-1].y;
        float dif = abs(x-ex) + abs(y-ey);
        
        float prevDif = 0;
        float tolerance = this->tolerance;
        while (dif > tolerance)
        {
            if (prevDif == dif)
                tolerance *= 2;
            
            prevDif = dif;
            
            // STAGE 1: FORWARD REACHING
            // Set the end effector as target
            this->chain->joints[this->numJoints-1].x = x;
            this->chain->joints[this->numJoints-1].y = y;
            
            for (int i = this->numJoints-2; i >= 0; i--) 
            {
                // Find the distance r_i between the new joint position i+1 (nx,ny)
                // and the joint i (jx,jy)
                float jx = this->chain->joints[i].x;
                float jy = this->chain->joints[i].y;
                float nx = this->chain->joints[i+1].x;
                float ny = this->chain->joints[i+1].y;
                float r_i = abs(nx-jx) + abs(ny-jy);
                float lambda_i = ((float)lengths[i])/r_i;
                
                // Find the new joint positions
                this->chain->joints[i].x = (float)((1-lambda_i)*nx + lambda_i*jx);
                this->chain->joints[i].y = (float)((1-lambda_i)*ny + lambda_i*jy);
            }
            
            // STAGE 2: BACKWARD REACHING
            // Set the root at its initial position
            this->chain->joints[0].x = bx;
            this->chain->joints[0].y = by;
            
            for (int i = 0; i < this->numJoints-1; i++)
            {
                // Find the distance r_i between the new joint position i (nx,ny)
                // and the joint i+1 (jx,jy)
                float jx = this->chain->joints[i+1].x;
                float jy = this->chain->joints[i+1].y;
                float nx = this->chain->joints[i].x;
                float ny = this->chain->joints[i].y;
                float r_i = abs(nx-jx) + abs(ny-jy);
                float lambda_i = ((float)lengths[i])/r_i;
                
                // Find the new joint positions
                this->chain->joints[i+1].x = (float)((1-lambda_i)*nx + lambda_i*jx);
                this->chain->joints[i+1].y = (float)((1-lambda_i)*ny + lambda_i*jy);
            }
            
            // Update distance between end effector and target
            ex = this->chain->joints[this->numJoints-1].x;
            ey = this->chain->joints[this->numJoints-1].y;
            dif = abs(x-ex) + abs(y-ey);
        }
    }
    
    if (this->numJoints > 1)
        this->chain->joints[0].angle = atan2(this->chain->joints[1].y,this->chain->joints[1].x);
    
    for (int i = 1; i <= this->numJoints-2; i++)
    {
        float ax = this->chain->joints[i+1].x;
        float ay = this->chain->joints[i+1].y;
        float cx = this->chain->joints[i-1].x;
        float cy = this->chain->joints[i-1].y;
        
        float a = lengths[i];
        float b = abs(cx-ax) + abs(cy-ay);
        float c = lengths[i-1];
        
        float angleRad = acos((a*a+c*c-b*b)/(2*a*c));
        this->chain->joints[i].angle = angleRad;
    }
}

/* getX(joint)
 * inputs: joint number
 * outputs: x position of joint
 */
float Fabrik2D::getX(int joint)
{
  if (joint >= 0 && joint < numJoints) {
      
      return this->chain->joints[joint].x;
      
  }
  return 0;
}

/* getY(joint)
 * inputs: joint number
 * outputs: y position of joint
 */
float Fabrik2D::getY(int joint)
{
  if (joint >= 0 && joint < numJoints) {
      
      return this->chain->joints[joint].y;
      
  }
  return 0;
}

/* getAngle(joint)
 * inputs: joint number
 * outputs: angle (radians) of joint
 */
float Fabrik2D::getAngle(int joint)
{
  if (joint >= 0 && joint < numJoints) {
      
      return this->chain->joints[joint].angle;
      
  }
  return 0;
}

/* setTolerance(tolerance)
 * inputs: tolerance value
 * 
 * sets the tolerance of the distance between the end effector and the target
 */
void Fabrik2D::setTolerance(float tolerance) 
{
    this->tolerance = tolerance;
}