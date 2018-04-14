/*
 * Elevator.hpp
 *
 *  Created on: Feb 8, 2018
 *      Author: Jim
 */

#ifndef SRC_ELEVATOR_HPP_
#define SRC_ELEVATOR_HPP_

#include <ctre/phoenix.h>
#include <WPILib.h>
#include <string>

class Elevator
{
public:
	enum EPos
	{
		Bottom = 0,
		Travel = 1,
		StackedBlock = 2,
		Switch = 3,
		ScaleLow = 4,
		ScaleMedium = 5,
		ScaleHigh = 6,
		Top = 7,
	};

private:
	static const int MaxPos = 8;
	static const int kTimeOut = 10;

	TalonSRX 	eTalon,
				eSTalon;

	double posVals[MaxPos][3];//0 = position, 1 = ramp, 2 = max speed


	//bool eZeroed;

	double maxRamp;

	EPos current;

	FILE*	log;
	std::string logName;

public:


	Elevator(int eTalon, int eSTalon);

	void SendData(std::string name="Elevator");
	void OpenLog(std::string name);
	void CloseLog();
	void Log();

	double GetHeight(EPos);
	double GetRamp(EPos);
	double GetMaxSpeed(EPos);

	bool GetBottomLimitSwitch(){return eTalon.GetSensorCollection().IsRevLimitSwitchClosed();};
	bool GetBottomSlaveLimitSwitch(){return !eSTalon.GetSensorCollection().IsRevLimitSwitchClosed();};
	bool GetTopLimitSwitch(){return eTalon.GetSensorCollection().IsFwdLimitSwitchClosed();};
	bool GetTopSlaveLimitSwitch(){return eSTalon.GetSensorCollection().IsFwdLimitSwitchClosed();};

	void SetMaxRamp(double ramp);

	//void eZeroed();

	EPos GetEPos();
	double GetError();

	double GetEncoderPos();
	double GetSetPoint();

	void SetEPos(EPos);

	void incPos();

	void decPos();

	void SetF(double f, int slot){eTalon.Config_kP(slot,f,kTimeOut);};
	void SetP(double p, int slot){eTalon.Config_kP(slot,p,kTimeOut);};
	void SetI(double i, int slot){eTalon.Config_kI(slot,i,kTimeOut);};
	void SetD(double d, int slot){eTalon.Config_kD(slot,d,kTimeOut);};

	void SetMotorSpeed(double speed);
	void SetPosition(double pos);

};


#endif /* SRC_ELEVATOR_HPP_ */
