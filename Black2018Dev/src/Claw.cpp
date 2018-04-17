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

	fireTime = 0.5;
	log = NULL;
	logName = "";
}

void Claw::Shoot(double speed)
{
	lm.Set(ControlMode::PercentOutput, speed);
	rm.Set(ControlMode::PercentOutput, speed);
}

bool Claw::Feed(double speed)
{
/*
	if(!sensor.Get())
	{
*/		lm.Set(ControlMode::PercentOutput, -speed*.8);
		rm.Set(ControlMode::PercentOutput, -speed);
		return true;
/*	}else
	{

		lm.Set(ControlMode::PercentOutput, -0.2);
		rm.Set(ControlMode::PercentOutput, -0.3);
		return false;
	}
*/
}

void Claw::ResetFire()
{
	fireTimer.Reset();
}

void Claw::Fire(double speed, double dur)
{
	fireTime = dur;
	fireTimer.Start();
	Shoot(speed);

}

bool Claw::isFiring()
{
	if(fireTimer.Get()>=fireTime)
	{
		Shoot(0.0);
		fireTimer.Stop();
		return false;
	}
	return true;
}

void Claw::SendData(std::string name)
{
	SmartDashboard::PutBoolean(name + " Sensor",sensor.Get());
	SmartDashboard::PutNumber(name+" RM Current",rm.GetOutputCurrent());
	SmartDashboard::PutNumber(name+" RM Voltage",rm.GetMotorOutputVoltage());
	SmartDashboard::PutNumber(name+" RM Output%",rm.GetMotorOutputPercent());
	SmartDashboard::PutNumber(name+" LM Current",lm.GetOutputCurrent());
	SmartDashboard::PutNumber(name+" LM Voltage",lm.GetMotorOutputVoltage());
	SmartDashboard::PutNumber(name+" LM Output%",lm.GetMotorOutputPercent());
	//SmartDashboard::PutNumber(name + " Speed",lm.);
}

void Claw::OpenLog(std::string name)
{
	if(log==NULL)
	{
		logName = name +".txt";

		log = fopen(logName.c_str(),"a");
	}
	if(log!=NULL)
	{
		fprintf(log, "Time, SensorState, RM Current, RM Voltage, RM Output Percent, LM Current, LM Voltage, LM Output Percent\r\n");//header
	}
}

void Claw::CloseLog()
{
	if(log!=NULL)
	{
		fclose(log);
	}
}

void Claw::Log()
{
	if(log!=NULL)
	{
		fprintf(log,"%f, %d, ", RobotController::GetFPGATime(), sensor.Get());
		fprintf(log,"%f, %f, %f, "  , rm.GetOutputCurrent(), rm.GetMotorOutputVoltage(), rm.GetMotorOutputPercent());
		fprintf(log,"%f, %f, %f\r\n", lm.GetOutputCurrent(), lm.GetMotorOutputVoltage(), lm.GetMotorOutputPercent());
	}
}
