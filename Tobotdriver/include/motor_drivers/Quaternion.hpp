#ifndef QUATERNION_HPP
#define QUATERNION_HPP

#include <iostream>
#include <vector>
#include <cstdlib>
#include <math.h>
#define TOLERANCE 0.00001f
#define PIOVER180 3.14159/180
// Convert from Euler Angles
class Quaternion
{
	protected : 
	float x;
	float y;
	float z;
	float w;
	public : 
	Quaternion(float roll, float pitch, float yaw){
		// Basically we create 3 Quaternions, one for pitch, one for yaw, one for roll
		// and multiply those together.
		// the calculation below does the same, just shorter
		float p = pitch * PIOVER180 / 2.0;
		float y = yaw * PIOVER180 / 2.0;
		float r = roll * PIOVER180 / 2.0;
	 
		float sinp = sin(p);
		float siny = sin(y);
		float sinr = sin(r);
		float cosp = cos(p);
		float cosy = cos(y);
		float cosr = cos(r);
		this->x = sinr * cosp * cosy - cosr * sinp * siny;
		this->y = cosr * sinp * cosy + sinr * cosp * siny;
		this->z = cosr * cosp * siny - sinr * sinp * cosy;
		this->w = cosr * cosp * cosy + sinr * sinp * siny;
		normalise();
	}
	
	Quaternion(float x, float y, float z, float w){
		x=x;
		y=y;
		z=z;
		w=w;
	}
	
	float getX(){return x;}
	float getY(){return y;}
	float getZ(){return z;}
	float getW(){return w;}
	
	
	// normalising a quaternion works similar to a vector. This method will not do anything
	// if the quaternion is close enough to being unit-length. define TOLERANCE as something
	// small like 0.00001f to get accurate results
	void normalise()
	{
		// Don't normalize if we don't have to
		float mag2 = w * w + x * x + y * y + z * z;
		if (fabs(mag2) > TOLERANCE && fabs(mag2 - 1.0f) > TOLERANCE) {
			float mag = sqrt(mag2);
			w /= mag;
			x /= mag;
			y /= mag;
			z /= mag;
		}
	}
	
	Quaternion getConjugate()
	{
		return Quaternion(-x, -y, -z, w);
	}


	
};

#endif
