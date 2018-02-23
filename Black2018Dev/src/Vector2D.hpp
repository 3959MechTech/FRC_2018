/*
 * Vector2D.hpp
 *
 *  Created on: Feb 20, 2018
 *      Author: Jim
 */

#ifndef SRC_VECTOR2D_HPP_
#define SRC_VECTOR2D_HPP_

#include <WPILib.h>

class Vector2D
{
public:
	void SendSDData(std::string name)
	{
		SmartDashboard::PutNumber(name+" X", _x);
		SmartDashboard::PutNumber(name+" Y", _y);
	};

	Vector2D(){_x = 0.0; _y=0.0;};
	Vector2D(double x, double y){_x = x; _y=y;};

	void SetX(double x){_x = x;};
	void SetY(double y){_y = y;};

	double GetX(){return _x;};
	double GetY(){return _y;};


	void operator=(const Vector2D& v)
	{
		_x = v._x;
		_y = v._y;
	}

private:

	double _x;
	double _y;

};





#endif /* SRC_VECTOR2D_HPP_ */
