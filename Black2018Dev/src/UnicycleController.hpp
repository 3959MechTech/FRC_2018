/*
 * UnicycleController.hpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Austin
 */
#ifndef SRC_UNICYCLECONTROLLER_HPP_
#define SRC_UNICYCLECONTROLLER_HPP_

#include"DifferentialMotorCommand.hpp"
#include"VelocityVector.hpp"
#include"Pose2D.hpp"
#include <math.h>

#include "Pose2D.hpp"
//class Pose2D{};


class UnicycleController
{

public:


	UnicycleController(Pose2D* pose);



	DifferentialMotorCommand GetDifferentialMotorCommand(double x,double y);
	DifferentialMotorCommand GetDifferentialMotorCommand(double phi);

	void Set_maxOmega(double w){_maxOmega = w;};
	double Get_maxOmega(){return _maxOmega;};

	void Set_wheelBase(double l){_wheelBase =l;};
	double Get_wheelBase(){return _wheelBase;};

	void Set_wheelRadius(double r){_wheelRadius =r;};
	double Get_wheelRadius(){return _wheelRadius;};

	void Set_transformL(double l){_transformL =l;};
	double Get_transformL(){return _transformL;};

private:

	Pose2D* robotPose;
	double _maxOmega; //this is used to scale turn speed

	double _wheelBase;// R=WheelBase
	double _wheelRadius;//L=WheelRadius

	double _transformL;//l

public:
	VelocityVector Tracker(double x,double y);
	VelocityVector Tracker(double phi);
	VelocityVector Transform (double x,double y );
	VelocityVector Transform(double phi );

	DifferentialMotorCommand DifferentialOutput(VelocityVector input);

};


#endif //SRC_UNICYCLECONTROLLER_HPP_




