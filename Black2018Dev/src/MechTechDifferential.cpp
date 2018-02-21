/*
 * MechTechDifferential.cpp
 *
 *  Created on: Feb 17, 2018
 *      Author: Jim
 */


#include "MechTechDifferential.hpp"

#include "ctre/Phoenix.h"
#include <WPILib.h>
#include <SmartDashboard/SmartDashboard.h>
#include <DifferentialMotorCommand.hpp>
#include <joystick.h>

#include "MechTechDifferential.hpp"


MechTechDifferential::MechTechDifferential(int rightMaster, int leftMaster)
{
	wheelBase = 24.0;
	wheelDiameter = 4.0;
	encoderTicks = 1440.0;
	inchPerTick = (wheelDiameter*3.14159)/encoderTicks;
	ticksPerInch = encoderTicks/(wheelDiameter*3.14159);

	rM = new TalonSRX(rightMaster);
	lM = new TalonSRX(leftMaster);
	rS = 0;
	lS = 0;

	lM->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);
	rM->ConfigSelectedFeedbackSensor(FeedbackDevice::CTRE_MagEncoder_Relative, 0, 0);

	//rS->Follow(rM);
	//lS->Follow(lM);

	//Drive Inversion
	rM->SetSensorPhase(true);
	lM->SetSensorPhase(false);

	//Individual Drive Motor Inversions
	//rS->SetInverted(true);
	//lS->SetInverted(false);
	rM->SetInverted(true);
	lM->SetInverted(false);

	lM->ConfigOpenloopRamp(.2,1);
	rM->ConfigOpenloopRamp(.2,1);
	lM->ConfigClosedloopRamp(.2,1);
	rM->ConfigClosedloopRamp(.2,1);

	lM->SelectProfileSlot(0,0);
	rM->SelectProfileSlot(0,0);

	lM->ConfigPeakOutputForward(1.0,0);
	lM->ConfigPeakOutputReverse(-1.0,0);
	lM->ConfigNominalOutputForward(.03,0);
	lM->ConfigNominalOutputReverse(-.03,0);

	rM->ConfigPeakOutputForward(1.0,0);
	rM->ConfigPeakOutputReverse(-1.0,0);
	rM->ConfigNominalOutputForward(.03,0);
	rM->ConfigNominalOutputReverse(-.03,0);

}

MechTechDifferential::MechTechDifferential(int rightMaster, int leftMaster, int rightSlave, int leftSlave)
{
	wheelBase = 24.0;
	wheelDiameter = 4.0;
	encoderTicks = 1440.0;
	inchPerTick = (wheelDiameter*3.14159)/encoderTicks;
	ticksPerInch = encoderTicks/(wheelDiameter*3.14159);

	rM = new TalonSRX(rightMaster);
	lM = new TalonSRX(leftMaster);
	rS = new TalonSRX(rightSlave);
	lS = new TalonSRX(leftSlave);

	lM->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);
	rM->ConfigSelectedFeedbackSensor(FeedbackDevice::QuadEncoder, 0, 0);

	rS->Follow(*rM);
	lS->Follow(*lM);

	//Drive Inversion
	rM->SetSensorPhase(false);
	lM->SetSensorPhase(false);

	//Individual Drive Motor Inversions
	rS->SetInverted(true);
	lS->SetInverted(false);
	rM->SetInverted(true);
	lM->SetInverted(false);

	lM->ConfigOpenloopRamp(.2,1);
	rM->ConfigOpenloopRamp(.2,1);
	lM->ConfigClosedloopRamp(.2,1);
	rM->ConfigClosedloopRamp(.2,1);

	lM->SelectProfileSlot(0,0);
	rM->SelectProfileSlot(0,0);

	lM->ConfigPeakOutputForward(1.0,0);
	lM->ConfigPeakOutputReverse(-1.0,0);
	lM->ConfigNominalOutputForward(.03,0);
	lM->ConfigNominalOutputReverse(-.03,0);

	rM->ConfigPeakOutputForward(1.0,0);
	rM->ConfigPeakOutputReverse(-1.0,0);
	rM->ConfigNominalOutputForward(.03,0);
	rM->ConfigNominalOutputReverse(-.03,0);

}

void MechTechDifferential::SetRamp(double time)
{
	lM->ConfigOpenloopRamp(time,10);
	rM->ConfigOpenloopRamp(time,10);
	lM->ConfigClosedloopRamp(time,10);
	rM->ConfigClosedloopRamp(time,10);
}

void MechTechDifferential::SetMaxSpeed(double percent)
{
	lM->ConfigPeakOutputForward(percent,0);
	lM->ConfigPeakOutputReverse(-percent,0);

	rM->ConfigPeakOutputForward(percent,0);
	rM->ConfigPeakOutputReverse(-percent,0);
}

double MechTechDifferential::GetWheelPos(WheelType w)
{
	if(w == WheelType::LeftFront || w == WheelType::LeftRear)
	{
		return GetLeftEncoderPosition();
	}else
	{
		return GetRightEncoderPosition();
	}

}

void	 MechTechDifferential::Set(double VR, double VL)
{

	rM->Set(ControlMode::Velocity,VR*1500.0);
	lM->Set(ControlMode::Velocity,VL*1500.0);

	/*
	rM->Set(ControlMode::PercentOutput,VR);
	lM->Set(ControlMode::PercentOutput,VR);
	*/
}

double MechTechDifferential::GetLeftVelocity()
{
	return lM->GetSelectedSensorVelocity(0);
}

double MechTechDifferential::GetRightVelocity()
{
	return rM->GetSelectedSensorVelocity(0);
}

double MechTechDifferential::GetLeftEncoderPosition()
{
	return lM->GetSelectedSensorPosition(0);
}

double MechTechDifferential::GetRightEncoderPosition()
{
	return rM->GetSelectedSensorPosition(0);
}

void MechTechDifferential::SetLeftVelocity(double speed)
{
	lM->Set(ControlMode::Velocity,speed);
}

void MechTechDifferential::SetRightVelocity(double speed)
{
	rM->Set(ControlMode::Velocity,speed);
}

void MechTechDifferential::Set(DifferentialMotorCommand cmd)
{
	SetLeftVelocity(cmd.VL);
	SetRightVelocity(cmd.VR);
}

void MechTechDifferential::ResetEncoders()
{
	lM->SetSelectedSensorPosition(0,0,0);
	rM->SetSelectedSensorPosition(0,0,0);
}

double MechTechDifferential::GetLeftP()
{
	return lM->ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_P,0,10);
}

double MechTechDifferential::GetRightP()
{
	return rM->ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_P,0,10);
}

double MechTechDifferential::GetLeftI()
{
	return lM->ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_I,0,10);
}

double MechTechDifferential::GetRightI()
{
	return rM->ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_I,0,10);
}

double MechTechDifferential::GetLeftD()
{
	return lM->ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_D,0,10);
}

double MechTechDifferential::GetRightD()
{
	return rM->ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_D,0,10);
}

double MechTechDifferential::GetLeftF()
{
	return lM->ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_F,0,10);
}

double MechTechDifferential::GetRightF()
{
	return rM->ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_F,0,10);
}

void MechTechDifferential::SetLeftFPID(double F, double P, double I, double D)
{
	lM->Config_kF(0,F,10);
	lM->Config_kP(0,P,10);
	lM->Config_kI(0,I,10);
	lM->Config_kD(0,D,10);
}

void MechTechDifferential::SetRightFPID(double F, double P, double I, double D)
{
	rM->Config_kF(0,F,10);
	rM->Config_kP(0,P,10);
	rM->Config_kI(0,I,10);
	rM->Config_kD(0,D,10);
}

void MechTechDifferential::SendData(std::string name )
{
	SmartDashboard::PutNumber(name+" Left Velocity", lM->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber(name+" Left Position", lM->GetSelectedSensorPosition(0));
	SmartDashboard::PutNumber(name+" Right Velocity", rM->GetSelectedSensorVelocity(0));
	SmartDashboard::PutNumber(name+" Right Position", rM->GetSelectedSensorPosition(0));

	SmartDashboard::PutNumber(name+" Left raw position", lM->GetSensorCollection().GetQuadraturePosition());
	SmartDashboard::PutNumber(name+" Right raw position", rM->GetSensorCollection().GetQuadraturePosition());

	SmartDashboard::PutNumber(name+" Left Closed Loop Error", lM->GetClosedLoopError(0));
	SmartDashboard::PutNumber(name+" Left Closed Loop Target", lM->GetClosedLoopTarget(0));
	SmartDashboard::PutNumber(name+" Right Closed Loop Error", rM->GetClosedLoopError(0));
	SmartDashboard::PutNumber(name+" Right Closed Loop Target", rM->GetClosedLoopTarget(0));

	SmartDashboard::PutNumber(name+" Left Ramp Closed",lM->ConfigGetParameter(ctre::phoenix::ParamEnum::eClosedloopRamp,0,10));
	SmartDashboard::PutNumber(name+" Left Ramp Open",lM->ConfigGetParameter(ctre::phoenix::ParamEnum::eOpenloopRamp,0,10));
	SmartDashboard::PutNumber(name+" Right Ramp Closed",lM->ConfigGetParameter(ctre::phoenix::ParamEnum::eClosedloopRamp,0,10));
	SmartDashboard::PutNumber(name+" Right Ramp Open",lM->ConfigGetParameter(ctre::phoenix::ParamEnum::eOpenloopRamp,0,10));

	SmartDashboard::PutNumber(name+" Left F",lM->ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_F,0,10));
	SmartDashboard::PutNumber(name+" Left P",lM->ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_P,0,10));
	SmartDashboard::PutNumber(name+" Left I",lM->ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_I,0,10));
	SmartDashboard::PutNumber(name+" Left D",lM->ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_D,0,10));

	SmartDashboard::PutNumber(name+" Right F",rM->ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_F,0,10));
	SmartDashboard::PutNumber(name+" Right P",rM->ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_P,0,10));
	SmartDashboard::PutNumber(name+" Right I",rM->ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_I,0,10));
	SmartDashboard::PutNumber(name+" Right D",rM->ConfigGetParameter(ctre::phoenix::ParamEnum::eProfileParamSlot_D,0,10));

}




