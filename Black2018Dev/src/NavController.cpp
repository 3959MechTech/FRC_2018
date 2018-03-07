/*
 * NavController.cpp
 *
 *  Created on: Feb 3, 2018
 *      Author: Cloie
 *
 *
 */
#include "NavController.hpp"
#include <cmath>

NavController::NavController(Pose2D* pose)
{
	v0 = 12.0;
	alpha = 1.0;
	robotPose = pose;
	maxOmega = 400.0;
}

Vector2D 	NavController::GoToGoal(double x, double y)
{
	double ex = x-robotPose->GetX();
	double ey = y-robotPose->GetY();
	double emag =sqrt(ex*ex+ey*ey);
	double k=v0*(1.0-exp(-alpha*emag*emag));
	double w = -atan2(ey,ex)/3.14159;
	double dir = 1.0;
	if(ex<0.0)
	{
		dir = -1.0;
	}
	w = maxOmega*w;
	//return Vector2D(k*ex,k*ey);
	//return Vector2D(dir*v0*(1.0-exp(-alpha*e*e)),1.0*w);
	return Vector2D(dir*k,w);

}

double 	NavController::Turn(double phi)
{
	double e=phi - robotPose->GetPhi();

	e=atan2(sin(e),cos(e));

	if(e>1.50)
	{
		e = 3.14159;
	}
	return e;
}
