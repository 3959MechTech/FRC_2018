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

class PIDOutputCatcher: public PIDOutput
{
public:
	PIDOutputCatcher()
	{
		_output=0.0;
	}
	virtual ~PIDOutputCatcher()
	{

	}

	void PIDWrite(double output)
	{
		std::lock_guard<wpi::mutex> sync(m_mutex);
		_output = output;
	}

	double GetOutput()
	{
		std::lock_guard<wpi::mutex> sync(m_mutex);
		return _output;
	}
private:
	double _output;
	mutable wpi::mutex m_mutex;
	//LinearDigitalFilter filt{};
};

class NavController
{
	Pose2D* robotPose;
	double v0;
	double alpha;
	double maxOmega;

	double _angle;
	PIDController headingPID;
	PIDOutputCatcher catcher;


public:
	NavController(Pose2D* pose);
	NavController(Pose2D* pose,double p, double i, double d);

	void SetPID(double p, double i, double d);
	void SetPID(double f,double p, double i, double d);
	void SetV0(double val){v0=val;};
	void SetAlpha(double val){alpha=val;};
	void SetOmega(double val){maxOmega=val; headingPID.SetOutputRange(-val,val);};
	void SetAngle(double angle);

	void 	SetP(double p){headingPID.SetP(p);};
	void 	SetI(double i){headingPID.SetI(i);};
	void 	SetD(double d){headingPID.SetD(d);};
	double 	GetP(){return headingPID.GetP();};
	double 	GetI(){return headingPID.GetI();};
	double 	GetD(){return headingPID.GetD();};

	double GetV0(){return v0;};
	double GetAlpha(){return alpha;};
	double GetOmega(){return maxOmega;};
	double GetAngle(){return _angle;};\
	double GetPIDError(){return headingPID.GetError();};
	double GetAvgPIDError(){return headingPID.GetAvgError();};

	double Turn(double phi);
	double Turn();
	Vector2D GoToGoal(double x, double y);

};





#endif /* SRC_NAVCONTROLLER_HPP_ */
