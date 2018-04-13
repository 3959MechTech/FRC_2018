/*
 * Quaternion.hpp
 *
 *  Created on: Oct 28, 2017
 *      Author: Jim
 */

#ifndef SRC_QUATERNION_HPP_
#define SRC_QUATERNION_HPP_

#include <math.h>

class Quaternion
{
public:

	typedef enum
	{
		X,
		Y,
		Z,
		W
	}Element;

	Quaternion(){m_x=0;m_y=0;m_z=0;m_w=0;};

	Quaternion(double w, double x, double y, double z)
			  	  {m_x=x;m_y=y;m_z=z;m_w=w;};

	~Quaternion(){};

	void SetQuaternion(double w, double x, double y, double z)
	{
		m_x = x;
		m_y = y;
		m_z = z;
		m_w = w;
	};

	double Magnitude() const
	{
		return sqrt(m_x*m_x+m_y*m_y+m_z*m_z+m_w*m_w);
	}
	void Normalize()
	{
		double mag = Magnitude();
		m_x /= mag;
		m_y /= mag;
		m_z /= mag;
		m_w /= mag;
	}

	double GetW(){return m_w;};
	double GetX(){return m_x;};
	double GetY(){return m_y;};
	double GetZ(){return m_z;};

	void SetW(double w){m_w = w;};
	void SetX(double x){m_x = x;};
	void SetY(double y){m_y = y;};
	void SetZ(double z){m_z = z;};

	Quaternion conjugate() const
	{
		return Quaternion(m_w, -m_x, -m_y, -m_z);
	}

	//Returns angle between
	double GetAngleBetween(Quaternion Qa,Quaternion Qb) const
	{
		Quaternion a = Qa;
		Quaternion b = Qb;
		a.Normalize();
		b.Normalize();
		Quaternion q = a.conjugate()*b;
		double w = q.GetW();
		return 180.0*(acos(w)*2)/M_PI;
	}

	Quaternion operator*(const Quaternion& q) const
	{
		return Quaternion(
			m_w*q.m_w - m_x*q.m_x - m_y*q.m_y - m_z*q.m_z,
			m_w*q.m_x + m_x*q.m_w + m_y*q.m_z - m_z*q.m_y,
			m_w*q.m_y - m_x*q.m_z + m_y*q.m_w + m_z*q.m_x,
			m_w*q.m_z + m_x*q.m_y - m_y*q.m_x + m_z*q.m_w
		);
	}

	Quaternion operator+(const Quaternion& q) const
	{
		return Quaternion(m_w + q.m_w, m_x + q.m_x, m_y + q.m_y, m_z + q.m_z);
	}

	void operator=(const Quaternion& q)
	{
		m_w = q.m_w;
		m_x = q.m_x;
		m_y = q.m_y;
		m_z = q.m_z;
	}

	Quaternion operator-(const Quaternion& q) const
	{
		return Quaternion(m_w - q.m_w, m_x - q.m_x, m_y - q.m_y, m_z - q.m_z);
	}

	Quaternion operator/(double scalar) const
	{
		return Quaternion(m_w / scalar, m_x / scalar, m_y / scalar, m_z / scalar);
	}

	Quaternion operator*(double scalar) const
	{
		return scale(scalar);
	}

	Quaternion scale(double scalar) const
	{
		return Quaternion(m_w * scalar, m_x * scalar, m_y * scalar, m_z * scalar);
	}


private:
	double m_w, m_x,m_y,m_z;
};



#endif /* SRC_QUATERNION_HPP_ */
