/*
 * Claw.cpp
 *
 *  Created on: Feb 10, 2018
 *      Author: Steve
 */

#include "Claw.hpp"

Claw::Claw(int leftMotor, int rightMotor, int sensorport): lm(leftMotor), rm(rightMotor), sensor(sensorport)
{
	lm.SetInverted(false);
	//rm.Follow(lm);
	rm.SetInverted(true);
}

void Claw::Shoot(double speed)
{
	lm.Set(ControlMode::PercentOutput, speed);
	rm.Set(ControlMode::PercentOutput, speed);
}

bool Claw::Feed(double speed)
{
	if(!sensor.Get())
	{
		lm.Set(ControlMode::PercentOutput, -speed*.8);
		rm.Set(ControlMode::PercentOutput, -speed);
		return true;
	}else
	{
		lm.Set(ControlMode::PercentOutput, -0.2);
		rm.Set(ControlMode::PercentOutput, -0.3);
		return false;
	}

}

void Claw::SendData(std::string name)
{
	SmartDashboard::PutBoolean(name + " Sensor",sensor.Get());
	//SmartDashboard::PutNumber(name + " Speed",lm.);
}
