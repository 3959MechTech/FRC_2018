/*
 * Elevator.cpp
 *
 *  Created on: Feb 8, 2018
 *      Author: Jim
 */
#include "Elevator.hpp"

#include <ctre/phoenix.h>
#include <WPILib.h>
#include <string>

Elevator::Elevator(int masterMotor, int slaveMotor):eTalon(masterMotor),eSTalon(slaveMotor)
{
	eTalon.ConfigSetParameter(ctre::phoenix::ParamEnum::eClearPositionOnLimitR,1,0,0,kTimeOut);

	eTalon.ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTimeOut);

	eTalon.ConfigForwardLimitSwitchSource(
			RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteTalonSRX,
			LimitSwitchNormal::LimitSwitchNormal_NormallyOpen,
			eSTalon.GetDeviceID(),
			10
		);
	eTalon.ConfigReverseLimitSwitchSource(
			RemoteLimitSwitchSource::RemoteLimitSwitchSource_RemoteTalonSRX,
			LimitSwitchNormal::LimitSwitchNormal_NormallyClosed,
			eSTalon.GetDeviceID(),
			10
		);
	eSTalon.Follow(eTalon);

	//Elevator Inversion
	eTalon.SetInverted(false);
	eSTalon.SetInverted(false);

	eTalon.SetSensorPhase(false);

	maxRamp=1.0;
	current = EPos::Bottom;

	posVals[Bottom][0] = 1000;
	posVals[Bottom][1] = .2;
	posVals[Bottom][2] = 1.0;

	posVals[Travel][0] = 8000;
	posVals[Travel][1] = .2;
	posVals[Travel][2] = 1.0;

	posVals[StackedBlock][0] = 15000;
	posVals[StackedBlock][1] = .2;
	posVals[StackedBlock][2] = 1.0;

	posVals[Switch][0] = 45000;
	posVals[Switch][1] = .3;
	posVals[Switch][2] = .8;

	posVals[ScaleLow][0] = 120000;
	posVals[ScaleLow][1] = .5;
	posVals[ScaleLow][2] = .7;

	posVals[ScaleMedium][0] = 137000;
	posVals[ScaleMedium][1] = .6;
	posVals[ScaleMedium][2] = .6;

	posVals[ScaleHigh][0] = 156000;
	posVals[ScaleHigh][1] = .75;
	posVals[ScaleHigh][2] = .45;

	posVals[Top][0] = 156000;
	posVals[Top][1] = 1.0;
	posVals[Top][2] = .4;

	eTalon.ConfigPeakOutputForward(1.0,0);
	eTalon.ConfigPeakOutputReverse(-.7,0);
	eTalon.ConfigNominalOutputForward(.06,0);
	eTalon.ConfigNominalOutputReverse(0.06,0);//I think this will help with the lowering.

	eTalon.SelectProfileSlot(0,0);

	//Up moves
	eTalon.Config_kF(0,0,kTimeOut);
	eTalon.Config_kP(0,.3,kTimeOut);//original @.15
	eTalon.Config_kI(0,0,kTimeOut);
	eTalon.Config_kD(0,0.2,kTimeOut);//original @ .2

	//Down Moves
	eTalon.Config_kF(1,0,kTimeOut);
	eTalon.Config_kP(1,.1,kTimeOut);//original @.05
	eTalon.Config_kI(1,0,kTimeOut);
	eTalon.Config_kD(1,0.5,kTimeOut);

}

void Elevator::SendData(std::string name)
{

	SmartDashboard::PutNumber(name+" Velocity", eTalon.GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber(name+" Position", eTalon.GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber(name+" Slave Position", eSTalon.GetSelectedSensorPosition(0));

	SmartDashboard::PutNumber(name+" raw position", eTalon.GetSensorCollection().GetQuadraturePosition());
	SmartDashboard::PutBoolean(name+" Top Limit Switch", eTalon.GetSensorCollection().IsFwdLimitSwitchClosed());
	SmartDashboard::PutBoolean(name+" Bottom Limit Switch", eTalon.GetSensorCollection().IsRevLimitSwitchClosed());

	SmartDashboard::PutNumber(name+" Closed Loop Error", eTalon.GetClosedLoopError(0));
	SmartDashboard::PutNumber(name+" Closed Loop Target", eTalon.GetClosedLoopTarget(0));
	SmartDashboard::PutNumber(name+" Bottom Limit Switch", eTalon.ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_P,0,kTimeOut));

	SmartDashboard::PutNumber(name+" F",eTalon.ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_F,0,10));
	SmartDashboard::PutNumber(name+" P",eTalon.ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_P,0,10));
	SmartDashboard::PutNumber(name+" I",eTalon.ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_I,0,10));
	SmartDashboard::PutNumber(name+" D",eTalon.ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_D,0,10));

}

double Elevator::GetHeight(EPos pos)
{
	return posVals[pos][0];
}

double Elevator::GetRamp(EPos pos)
{
	return posVals[pos][1];
}

void Elevator::SetMaxRamp(double ramp)
{
	maxRamp = ramp;
}

//void eZeroed();

Elevator::EPos Elevator::GetEPos()
{
	return current;
}

double Elevator::GetError()
{
	return eTalon.GetClosedLoopError(0);
}

double Elevator::GetSetPoint()
{
	return eTalon.GetClosedLoopTarget(0);
}

double Elevator::GetEncoderPos()
{
	return eTalon.GetSelectedSensorPosition(kTimeOut);
}

void Elevator::SetEPos(EPos pos)
{
	current = pos;
	if(posVals[pos][0]<=eTalon.GetSelectedSensorPosition(0))
	{
		eTalon.SelectProfileSlot(1,0);
	}else
	{
		eTalon.SelectProfileSlot(0,0);
	}
	eTalon.Set(ControlMode::Position, posVals[pos][0]);
}

void Elevator::incPos()
{
	if(current!= EPos::Top)
	{
		current=(EPos)((int)current+1);
		SetEPos(current);
	}

}

void Elevator::decPos()
{
	if(current != EPos::Bottom)
	{
		current=(EPos)((int)current-1);
		SetEPos(current);
	}
}

void Elevator::SetMotorSpeed(double speed)
{
	eTalon.Set(ControlMode::PercentOutput,speed);
}
void Elevator::SetPosition(double pos)
{
	if(pos>posVals[EPos::Top][0])
	{
		pos = posVals[EPos::Top][0];
	}
	//eTalon.GetClosedLoopTarget(0);
	eTalon.Set(ControlMode::Position, pos);
}




