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

NavController::NavController(Pose2D* pose):
	headingPID(2200.0,0.0,10000.0,pose,&catcher,.005),
	catcher()
{
	v0 = 12.0;
	alpha = 1.0;
	robotPose = pose;
	maxOmega = 2000.0;
	_angle  = 0.0;

	headingPID.SetContinuous();
	headingPID.SetInputRange(-3.14159,3.14159);
	headingPID.SetOutputRange(-maxOmega,maxOmega);

	headingPID.SetPercentTolerance(.02);

	headingPID.Enable();

}
NavController::NavController(Pose2D* pose,double p, double i, double d):
	headingPID(p,i,d,robotPose,&catcher,.005),
	catcher()
{
	v0 = 12.0;
	alpha = 1.0;
	robotPose = pose;
	maxOmega = 400.0;
	_angle  = 0.0;

	headingPID.SetContinuous();
	headingPID.SetInputRange(-3.14159,3.14159);
	headingPID.SetOutputRange(-maxOmega,maxOmega);
	headingPID.SetPercentTolerance(.02);
	headingPID.Enable();
}

void  	NavController::SetPID(double p, double i, double d)
{
	headingPID.SetP(p);
	headingPID.SetI(i);
	headingPID.SetD(d);
}
void  	NavController::SetPID(double f,double p, double i, double d)
{
	headingPID.SetF(f);
	headingPID.SetP(p);
	headingPID.SetI(i);
	headingPID.SetD(d);
}
void 	NavController::SetAngle(double angle)
{
	_angle=atan2(sin(angle),cos(angle));
	headingPID.SetSetpoint(_angle);
}

Vector2D 	NavController::GoToGoal(double x, double y)
{
	double ex = x-robotPose->GetX();
	double ey = y-robotPose->GetY();
	double emag = sqrt(ex*ex+ey*ey);
	double k=v0*(1.0-exp(-alpha*emag*emag));
	double w = atan2(ey,ex)/3.14159;
	double dir = 1.0;
	if(ex<0.0)
	{
		dir = -1.0;
	}
	w = Turn();
	//return Vector2D(k*ex,k*ey);
	//return Vector2D(dir*v0*(1.0-exp(-alpha*e*e)),1.0*w);
	return Vector2D(dir*k,w);

}

double 	NavController::Turn(double phi)
{
	double e=phi - robotPose->GetPhi();

	e=atan2(sin(e),cos(e));

	if(e>2.0)
	{
		e = 3.14159;
	}
	if(e<-2.0)
	{
		e = -3.14159;
	}
	return e;
}
double 	NavController::Turn()
{
	double out = catcher.GetOutput();
	SmartDashboard::PutNumber("out", out);
	SmartDashboard::PutNumber("fabs err", fabs(headingPID.GetError()));
	if(fabs(headingPID.GetError())>.02 && fabs(out)<450.0)
	{
		SmartDashboard::PutString("Debug Statement", "YUP!");

		if(out<0)
		{
			out = -450.0;
		}else
		{
			out = 450.0;
		}

	}else
	{
		SmartDashboard::PutString("Debug Statement", "NOPE!");
	}
	return out;
}
