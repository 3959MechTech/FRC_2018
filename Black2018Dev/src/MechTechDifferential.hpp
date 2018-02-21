/*
 * MechTechDifferential.hpp
 *
 *  Created on: Feb 17, 2018
 *      Author: Jim
 */

#ifndef SRC_MECHTECHDIFFERENTIAL_HPP_
#define SRC_MECHTECHDIFFERENTIAL_HPP_

#include "ctre/Phoenix.h"
#include <WPILib.h>
#include <SmartDashboard/SmartDashboard.h>
#include <DifferentialMotorCommand.hpp>
#include <joystick.h>
#include "MechTechDrive.hpp"

class MechTechDifferential : MechTechDrive
{
private:

	TalonSRX *rM, *lM, *rS, *lS;
	double 	wheelBase,
			wheelDiameter,
			encoderTicks,
			inchPerTick,
			ticksPerInch;

	inline void UpdateTickRatios()
	{
		inchPerTick = (wheelDiameter*3.14159)/encoderTicks;
		ticksPerInch = encoderTicks/(wheelDiameter*3.14159);
	};

public:

	MechTechDifferential(int rightMaster, int leftMaster);
	MechTechDifferential(int rightMaster, int leftMaster, int rightSlave, int leftSlave);

	void ConfigRobot(double diameter, double base, double ticks)
	{
		wheelDiameter = diameter;
		wheelBase = base;
		encoderTicks = ticks;
		UpdateTickRatios();
	}

	void SetWheelDiameter(double d){wheelDiameter = d; UpdateTickRatios();};
	void SetWheelBase(double l){wheelBase = l; UpdateTickRatios();};
	void SetEncoderTicks(double ticks){encoderTicks = ticks; UpdateTickRatios();};

	double GetLeftVelocity();
	double GetRightVelocity();

	void Set(DifferentialMotorCommand cmd);
	void Set(double VR, double VL);

	void SetLeftVelocity(double speed);
	void SetRightVelocity(double speed);

	void SetRamp(double time);
	void SetMaxSpeed(double percent);
	double GetWheelPos(WheelType w);

	double GetLeftEncoderPosition();
	double GetRightEncoderPosition();

	void ResetEncoders();

	double GetLeftP();
	double GetRightP();
	double GetLeftI();
	double GetRightI();
	double GetLeftD();
	double GetRightD();
	double GetLeftF();
	double GetRightF();

	void SetLeftFPID(double F, double P, double I, double D);
	void SetRightFPID(double F, double P, double I, double D);

	void SendData(std::string name ="Drive");

};


#endif /* SRC_MECHTECHDIFFERENTIAL_HPP_ */
