/*
 * NavController.hpp
 *
 *  Created on: Feb 1, 2018
 *      Author: Cloie
 */

#ifndef SRC_NAVCONTROLLER_HPP_
#define SRC_NAVCONTROLLER_HPP_

#include "Pose2D.hpp"
#include "Vector2D.hpp"
#include "VelocityVector.hpp"



class NavController
{
	Pose2D* robotPose;
	double v0;
	double alpha;
	double maxOmega;

public:
	NavController(Pose2D* pose);

	void SetV0(double val){v0=val;};
	void SetAlpha(double val){alpha=val;};
	void SetOmega(double val){maxOmega=val;};

	double GetV0(){return v0;};
	double GetAlpha(){return alpha;};
	double GetOmega(){return maxOmega;};

	double Turn(double phi);
	Vector2D GoToGoal(double x, double y);

};





#endif /* SRC_NAVCONTROLLER_HPP_ */
