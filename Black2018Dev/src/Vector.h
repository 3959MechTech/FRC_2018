/*
 * Vector.h
 *
 *  Created on: Nov 14, 2017
 *      Author: Jim
 */

#ifndef SRC_VECTOR_H_
#define SRC_VECTOR_H_

class Vector
{
public:
	void SendSDData(std::string name)
	{
		SmartDashboard::PutNumber(name+"_X", x);
		SmartDashboard::PutNumber(name+"_Y", y);
		SmartDashboard::PutNumber(name+"_Z", z);
	};

	Vector(){x = 0.0; y=0.0;z=0.0, time = 0.0;};
//I know public members are a bad idea. it's fine.
	double time;
	double x;
	double y;
	double z;

	void operator=(const Vector& v)
	{
		x = v.x;
		y = v.y;
		z = v.z;
		time = v.time;
	}
};



#endif /* SRC_VECTOR_H_ */
