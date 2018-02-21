/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <memory>
#include <string>

#include <TimedRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "ctre/Phoenix.h"
#include "joystick.h"
#include <WPILib.h>
#include <Encoder.h>
#include "CameraServer.h"

#include "Elevator.hpp"
#include "Claw.hpp"
#include "MechTechDifferential.hpp"
#include "VibController.hpp"
#include "UnicycleController.hpp"
#include "Vector2D.hpp"
#include "BNO055.hpp"

#define vibStrength 10000

cs::UsbCamera camera0;
cs::UsbCamera camera1;




class Robot: public frc::TimedRobot
{
public:

	void RobotInit()
	{
		camera0 = CameraServer::GetInstance()->StartAutomaticCapture(0);
		camera1 = CameraServer::GetInstance()->StartAutomaticCapture(1);

		//chooser.AddDefault(autoNameDefault, autoNameDefault);
		//chooser.AddObject(autoNameCustom, autoNameCustom);
		//frc::SmartDashboard::PutData("Auto  Modes", &chooser);

		drive.ConfigRobot(4.0,24.0,1440.0);
		drive.SetLeftFPID(.72,0.0,0,0);
		drive.SetRightFPID(.72,0.0,0,0);

		drive.SetRamp(.4);

		oldLDrivePos = drive.GetLeftEncoderPosition();
		oldRDrivePos = drive.GetRightEncoderPosition();

		SendData();
	}

	void AutonomousInit() override
	{

		std::cout << "Auto selected: " << autoSelected << std::endl;

		//int sensorPos = 0; // sensor units


		if (autoSelected == autoNameCustom)
		{

		}
		 else
		{
			// Default Auto goes here
		}
		SendData();
	}

	void AutonomousPeriodic()
	{
		if (autoSelected == autoNameCustom)
		{
			// Custom Auto goes here
		} else {
			// Default Auto goes here
		}

		SendData();
	}


	void TeleopPeriodic()
	{

		double rt= stick2.GetTriggerAxis(frc::GenericHID::kRightHand);
		double lt= stick2.GetTriggerAxis(frc::GenericHID::kLeftHand);
		double s = 0.0;

		if(lt>.1)
		{
			claw.Shoot(lt);
			s = lt;
		}else
		{
			if(rt>.1)
			{
				claw.Feed(rt*2.0);
				s = rt;
			}else
			{
				claw.Shoot(0.0);
				s = 0.0;
			}
		}

		SmartDashboard::PutNumber("Claw Speed", s);

		Vator();

		if(stick3.GetAButton())
		{
			ResetPose();
		}

		Drive();
		UpdateDrivePos();
		v.VibrateTimer();

		SendData();
	}
;


	void TeleopInit()
	{
		SendData();
	}

	void TestPeriodic() {

		SendData();
	}

	void DisabledPeriodic()
	{
		SendData();
	}
	void DisabledInit()
	{
		SendData();
	}

	void SendData()
	{
		drivePos.SendData("Drive Pose");
		ele.SendData();
		claw.SendData();
		drive.SendData();

		std::string data;
		data = DriverStation::GetInstance().GetGameSpecificMessage();
		int dl =data.length();
		if(dl>0)
		{


			SmartDashboard::PutNumber("GD Length", data.length());
			SmartDashboard::PutString("GD Data", data);
			SmartDashboard::PutString("our Switch", data.substr(0,1));
			SmartDashboard::PutString("our scale", data.substr(1,1));
			SmartDashboard::PutString("other Switch", data.substr(2,1));
		}

		SmartDashboard::PutNumber("Robot VV V", vv.v);
		SmartDashboard::PutNumber("Robot VV W", vv.w);

		SmartDashboard::PutNumber("Robot DMC VL:", dmc.VL);
		SmartDashboard::PutNumber("Robot DMC VR:", dmc.VR);
	}

	void UpdateDrivePos()
	{

		double l = drive.GetLeftEncoderPosition();
		double r = drive.GetRightEncoderPosition();
		double d = ((l-oldLDrivePos)+(r-oldRDrivePos))/2.0;

		drivePos.SetX( drivePos.GetX() + d*cos(drivePos.GetPhi()));
		drivePos.SetY(drivePos.GetY() + d*sin(drivePos.GetPhi()));
		drivePos.SetPhi( drivePos.GetPhi() + ((r-oldRDrivePos)-(l-oldLDrivePos))/2.0);

		oldLDrivePos = l;
		oldRDrivePos = r;
	}

	void Drive()
	{
		double lY = -stick1.GetY(frc::GenericHID::kLeftHand);
		double rY = -stick1.GetY(frc::GenericHID::kRightHand);

		if(lY>-.2 && lY<.2)
		{
			lY = 0.0;
		}
		if(rY>-.2 && rY<.2)
		{
			rY = 0.0;
		}
		double lX = -stick1.GetX(frc::GenericHID::kLeftHand);
		double rX = -stick1.GetX(frc::GenericHID::kRightHand);

		if(lX>-.2 && lX<.2)
		{
			lX = 0.0;
		}
		if(rX>-.2 && rX<.2)
		{
			rX = 0.0;
		}

		//split Arcade
		vv.v = lY*1500.0;
		vv.w = rX*2500.0;


		//1 stick Arcade
		//vv.v = lY*1500.0;
		//vv.w = lX*2500.0;

		dmc = ucm.DifferentialOutput(vv);

		drive.Set(dmc);//Arcade Drive
		//drive.Set(rY, lY);//Tank Drive
		//drive.Set(rY*1000.0, lY*1000.0);
	}

	void Vator()
	{
/*
		if(stick2.GetBumperPressed(frc::GenericHID::kRightHand))
		{
			ele.incPos();
		}

		if(stick2.GetBumperPressed(frc::GenericHID::kLeftHand))
		{
			ele.decPos();
		}
*/

		if(stick2.GetYButtonPressed())
		{
			ele.SetEPos(Elevator::ScaleHigh);
		}

		if(stick2.GetBButtonPressed())
		{
			ele.SetEPos(Elevator::ScaleMedium);
		}

		if(stick2.GetXButtonPressed())
		{
			ele.SetEPos(Elevator::ScaleLow);
		}

		if(stick2.GetAButtonPressed())
		{
			ele.SetEPos(Elevator::Switch);
		}

		if(stick2.GetStartButtonPressed())
		{
			ele.SetEPos(Elevator::Travel);
		}

		if(stick2.GetBackButtonPressed())
		{
			ele.SetEPos(Elevator::Bottom);
		}

		double s2y = -stick2.GetY(frc::GenericHID::kRightHand);
		if(s2y>.2  || s2y<-.2)
		{

			ele.SetPosition(ele.GetSetPoint()+s2y*4000.0);
		}
/*		else
		{
			s2y = 0;
		}

		ele.SetMotorSpeed(s2y);*/
	}

	void ResetPose()
	{
		drivePos.SetX(0);
		drivePos.SetY(0);
		drivePos.SetPhi(0);
	}

	void GetFMSData()
	{
		if(!gotData)
		{
			std::string data;
			data = DriverStation::GetInstance().GetGameSpecificMessage();

			if(data.length()>=3)
			{
				gotData = true;
				if(data.substr(0,0)=="L")
				{
					ourSwitch = true;
				}
				if(data.substr(1,2)=="L")
				{
					scale = true;
				}
				if(data.substr(3,2)=="L")
				{
					theirSwitch = true;
				}

			}
		}
	}

	//Nav Controller Code!!!
	Vector2D GoToGoal(double x, double y)
	{
		//Pose2D u;

		double ex = x-drivePos.GetX();
		double ey = y-drivePos.GetX();
		double emag =sqrt(ex*ex+ey*ey);
		double k=v0*(1-exp(-alpha*emag*emag))/emag;

		//u.SetX(k*ex);
		//u.SetY(k*ex);
		return Vector2D(k*ex,k*ey);

	}

	double 	Turn(double phi)
	{
		double e=phi - drivePos.GetPhi();

		e=atan2(sin(e),cos(e));

		return e;
	}


public:
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooser;
	const std::string autoNameDefault = "Default";
	const std::string autoNameCustom = "My Auto";
	std::string autoSelected;

	bool gotData ;//have we gotten data from FMS
	//Left = true, false = right
	bool ourSwitch;
	bool scale;
	bool theirSwitch;

	//Nav coefficients
	double v0 = 12.0;
	double alpha = 1.0;

	double wRadius=4.0, wBase=24.0;

	double oldLDrivePos, oldRDrivePos;
	Pose2D drivePos;

	DifferentialMotorCommand dmc;
	VelocityVector vv;

	Vector2D u;

	BNO055 imu{I2C::Port::kOnboard};

	UnicycleController ucm{&drivePos};
	MechTechDifferential drive{0,2,1,3};
	Elevator ele{10,11};
	Claw claw{7,8,0};

	XboxController stick1{0};
	XboxController stick2{1};
	XboxController stick3{2};

	VibController v{&stick1,&stick2};
};

START_ROBOT_CLASS(Robot)
