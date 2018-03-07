/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <iostream>
#include <memory>
#include <string>
#include <math.h>

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
#include "NavController.hpp"


#define vibStrength 10000

cs::UsbCamera camera0;
cs::UsbCamera camera1;




class Robot: public frc::TimedRobot
{
public:

	void RobotInit()
	{
		prefs = Preferences::GetInstance();

		camera0 = CameraServer::GetInstance()->StartAutomaticCapture(0);
		camera1 = CameraServer::GetInstance()->StartAutomaticCapture(1);

		camera0.SetResolution(640,480);
		camera1.SetResolution(640,480);

		autoGoal.AddDefault(autoGoal0, AutoRoutine::Straight);
		autoGoal.AddObject(autoGoal1, AutoRoutine::Switch);
		autoGoal.AddObject(autoGoal2, AutoRoutine::Scale);
		autoGoal.AddObject(autoGoal3, AutoRoutine::BestSwitch);
		autoGoal.AddObject(autoGoal4, AutoRoutine::BestScale);
		frc::SmartDashboard::PutData("Auto Modes", &autoGoal);

		startSpot.AddDefault(spotMid,StartingSpot::Middle);
		startSpot.AddObject(spotLeft,StartingSpot::Left);
		startSpot.AddObject(spotRight,StartingSpot::Right);
		frc::SmartDashboard::PutData("Starting Spot", &startSpot);

		nc.SetV0(v0);
		nc.SetAlpha(alpha);
		nc.SetOmega(1200);

		turning = false;
		headingOffset = 0.0;

		autoStep = 0;
		angle = 0.0;

		drive.ConfigRobot(4.0,24.0,1440.0);
		drive.SetLeftFPID(.72,0.10,0,0);
		drive.SetRightFPID(.72,0.10,0,0);

		drive.SetRamp(0.2);

		oldLDrivePos = drive.GetLeftEncoderPosition();
		oldRDrivePos = drive.GetRightEncoderPosition();

		imu.Begin();
		imu.SetCurrentQuaternionAsInit();
		imu.SetInitEulers(imu.GetVector(BNO055::vector_type_t::VECTOR_EULER));

		GetFMSData();
		SendData();
	}

	void AutonomousInit() override
	{
		ResetPose();
		GetFMSData();
		SendData();
	}

	void AutonomousPeriodic()
	{
		UpdateDrivePos();
		AutoStraight();
		SendData();
	}


	void TeleopPeriodic()
	{
		UpdateDrivePos();


		double rt= stick2.GetTriggerAxis(frc::GenericHID::kRightHand);
		double lt= stick2.GetTriggerAxis(frc::GenericHID::kLeftHand);
		double s = 0.0;
		if(!stick3.GetBButton())
		{
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
		}

		SmartDashboard::PutNumber("Claw Speed", s);

		Vator();

		if(stick3.GetAButton())
		{
			ResetPose();
		}


		if(stick3.GetBButton())
		{
			AutoMidSwitch();
			drive.Set(dmc);
		}else
		{
			if(stick3.GetXButton())
			{
				//dmc = ucm.GetDifferentialMotorCommand(.5);
				drive.Set(ucm.GetDifferentialMotorCommand(500.0));
			}else
			{
				Drive();
			}

		}

		if(stick3.GetBumperPressed(frc::GenericHID::kRightHand))
		{
			autoStep++;
		}
		if(stick3.GetBumperPressed(frc::GenericHID::kLeftHand))
		{
			autoStep=0;
			gotData = false;
			GetFMSData();
		}

		//Drive();
		v.VibrateTimer();

		SendData();
	}



	void TeleopInit()
	{
		SendData();
	}

	void TestPeriodic() {

		SendData();
	}

	void DisabledPeriodic()
	{
		if(stick3.GetAButton())
		{
			ResetPose();
		}

		GetFMSData();
		UpdateDrivePos();
		AutoStraight();
		SendData();
	}
	void DisabledInit()
	{
		GetFMSData();
		SendData();
	}

	void SendData()
	{
		simx = prefs->GetDouble("simx",-1.0);
		simy = prefs->GetDouble("simy",-1.0);
		useSimData = prefs->GetBoolean("useSimData",false);
		SmartDashboard::PutNumber("Put simx",simx);
		SmartDashboard::PutNumber("Put simy",simy);
		SmartDashboard::PutNumber("headingOffset",headingOffset);
		SmartDashboard::PutNumber("Battery Power",DriverStation::GetInstance().GetBatteryVoltage());
		SmartDashboard::PutNumber("autoStep",autoStep);

		double phi = 3.14159*60.0/180.0;
		SmartDashboard::PutNumber("Turn to",phi);
		SmartDashboard::PutNumber("Turn to - fixed",atan2(sin(phi),cos(phi)));
		SmartDashboard::PutNumber("Turn e",phi-drivePos.GetPhi());
		SmartDashboard::PutNumber("Auto Angle Target", angle);

		v0 = prefs->GetDouble("v0",nc.GetV0());
		alpha = prefs->GetDouble("alpha",nc.GetAlpha());
		nc.SetV0(v0);
		nc.SetAlpha(alpha);
		nc.SetOmega(prefs->GetDouble("maxOmega",nc.GetOmega()));
		//ucm.Set_maxOmega(prefs->GetDouble("maxOmega",ucm.Get_maxOmega()));

		drivePos.SendData("Drive Pose");
		//ele.SendData();
		//claw.SendData();
		//drive.SendData();

		SmartDashboard::PutNumber("IMU Chip ID", imu.GetChipID());

		std::string data;
		data = DriverStation::GetInstance().GetGameSpecificMessage();
		int dl =data.length();
		if(dl>0)
		{
			SmartDashboard::PutNumber("GD Length", data.length());
			SmartDashboard::PutString("GD Data", data);
			SmartDashboard::PutBoolean("our Switch", ourSwitch);
			SmartDashboard::PutBoolean("our scale", scale);
			SmartDashboard::PutBoolean("other Switch", theirSwitch);
		}
		double ex = 120.0-drivePos.GetX();
		double ey = 0.0-drivePos.GetY();
		double emag =sqrt(ex*ex+ey*ey);
		double k=(nc.GetV0()*(1.0-exp(-nc.GetAlpha()*emag*emag)))/emag;

		SmartDashboard::PutNumber("Robot ex", ex);
		SmartDashboard::PutNumber("Robot ey", ey);
		//SmartDashboard::PutNumber("Robot v0", nc.GetV0());
		//SmartDashboard::PutNumber("Robot alpha", nc.GetAlpha());
		SmartDashboard::PutNumber("Robot emag", emag);
		SmartDashboard::PutNumber("Robot v0*(1-exp(-alpha*ex*ex))", nc.GetV0()*(1-exp(-nc.GetAlpha()*ex*ex)));
		SmartDashboard::PutNumber("Robot v0*(1-exp(-alpha*ey*ey))", nc.GetV0()*(1-exp(-nc.GetAlpha()*ey*ey)));
		//SmartDashboard::PutNumber("Robot exp(-nc.GetAlpha()*emag*emag)", exp(-nc.GetAlpha()*emag*emag));
		//SmartDashboard::PutNumber("Robot 1-exp(-nc.GetAlpha()*emag*emag)", 1.0-exp(-nc.GetAlpha()*emag*emag));
		SmartDashboard::PutNumber("Robot k", k);
		SmartDashboard::PutNumber("Robot ux", k*ex);
		SmartDashboard::PutNumber("Robot uy", k*ey);

		SmartDashboard::PutNumber("Robot U X", u.GetX());
		SmartDashboard::PutNumber("Robot U Y", u.GetY());

		SmartDashboard::PutNumber("Robot VV V", vv.v);
		SmartDashboard::PutNumber("Robot VV W", vv.w);

		SmartDashboard::PutNumber("Robot DMC VL:", dmc.VL);
		SmartDashboard::PutNumber("Robot DMC VR:", dmc.VR);
	}

	void UpdateDrivePos()
	{
		Vector eulers = imu.GetVector(BNO055::vector_type_t::VECTOR_EULER);
		double phi = -CleanAngle(3.14159*eulers.x/180.0);
		phi = phi-headingOffset;
		double l,r,d;
		if(!turning)
		{
			l = 3.14159*4.0*(drive.GetLeftEncoderPosition()/1440.0);
			r = 3.14159*4.0*(drive.GetRightEncoderPosition()/1440.0);
			d = ((l-oldLDrivePos)+(r-oldRDrivePos))/2.0;

			if(useSimData)
			{
				drivePos.SetX(simx);
				drivePos.SetY(simy);
			}else
			{
				drivePos.SetX( drivePos.GetX() + d*cos(phi));
				drivePos.SetY(drivePos.GetY() + d*sin(phi));
			}

			oldLDrivePos = l;
			oldRDrivePos = r;
		}else
		{
			drive.ResetEncoders();
			oldLDrivePos = drive.GetLeftEncoderPosition();
			oldRDrivePos = drive.GetRightEncoderPosition();
		}


		drivePos.SetPhi(phi);


	}

	void ResetPose()
	{
		drive.ResetEncoders();
		drivePos.SetX(0);
		drivePos.SetY(0);
		headingOffset = imu.GetVector(BNO055::vector_type_t::VECTOR_EULER).x;
		headingOffset = -CleanAngle(3.14159*headingOffset/180.0);
		drivePos.SetPhi(0.0);
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
		/*else
		{
			s2y = 0;
		}*/

		//ele.SetMotorSpeed(s2y);
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
				if(data.substr(0,1)=="L")
				{
					ourSwitch = true;
				}
				if(data.substr(1,1)=="L")
				{
					scale = true;
				}
				if(data.substr(2,1)=="L")
				{
					theirSwitch = true;
				}

			}
		}
	}



	inline double CleanAngle(double angle)
	{
		return atan2(sin(angle),cos(angle));
	}


	void AutoMidSwitch()
	{
		bool done = false;


		double ySign = 1.0;//used to flip the Y value
		if(!ourSwitch)
		{
			ySign = -1.0;
		}

		ucm.Set_maxOmega(2000);



		switch(autoStep)
		{
		case 0: nc.SetAlpha(.005);
				ele.SetEPos(Elevator::Travel);
				//nc.SetV0(1200);
				done = DriveStraight(24.0,0.0,4.0);
				break;
		case 1: angle = atan2(ySign*60.0-drivePos.GetY(),70.0-drivePos.GetX());
				done = true;
				break;
		case 2: done = Turn(angle);
				break;
		case 3: 	//nc.SetAlpha(.003);
				//nc.SetV0(1200);
				done = DriveStraight(70.0,ySign*60.0,4.0);
				break;
		case 4: ele.SetEPos(Elevator::Switch);
				done = Turn(0.0);
				break;
		case 5: 	nc.SetAlpha(.005);
				//nc.SetV0(1200);
				done = DriveStraight(102.0,ySign*60.0,4.0);
				break;
		case 6: autoStep++;
				autoTimer.Start();
				break;
		case 7:	claw.Shoot(.5);
				if(autoTimer.Get()>0.5)
				{
					autoTimer.Stop();
					autoStep++;
				}
				break;
		case 8: claw.Shoot(0.0);
				autoStep++;
				break;
//*/
		default: dmc.VL = 0.0; dmc.VR=0.0;
		}

		if(done)
		{
			done = false;
			autoStep++;
		}
	}

	void AutoStraight()
	{
		nc.SetAlpha(.001);
		nc.SetV0(1200);
		DriveStraight(120.0,0.0,4.0);
	}

	bool DriveStraight(double x, double y, double errOk)
	{
		u   = nc.GoToGoal(x, y);
		//vv  = ucm.Tracker(u.GetX(), u.GetY());
		vv.v = u.GetX();
		vv.w = u.GetY();
		dmc = ucm.DifferentialOutput(vv);
		//dmc = ucm.GetDifferentialMotorCommand(u.GetX(), u.GetY());

		autoVecErr.SetX(x-drivePos.GetX());
		autoVecErr.SetY(y-drivePos.GetY());


		if(autoVecErr.GetMag()>=errOk)//if(e.GetX()>=errOk)
		{
			//drive.Set(dmc);
			return false;
		}else
		{
			dmc.VL = 0.0;
			dmc.VR = 0.0;
			return true;
		}
	}

	bool Turn(double angle)
	{
		turnErr = angle-drivePos.GetPhi();
		vv = ucm.Tracker(nc.Turn(angle));
		dmc = ucm.DifferentialOutput(vv);
		SmartDashboard::PutNumber("Turning Error", sqrt(turnErr*turnErr));
		if(sqrt(turnErr*turnErr)<.04)//less than 2 deg error
		{
			return true;
		}
		return false;
	}
	void DebugStick()
	{
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
/*
		double s2y = -stick2.GetY(frc::GenericHID::kRightHand);
		if(s2y>.2  || s2y<-.2)
		{

			ele.SetPosition(ele.GetSetPoint()+s2y*4000.0);
		}
*/
		double lY = -stick3.GetY(frc::GenericHID::kLeftHand);
		double rY = -stick3.GetY(frc::GenericHID::kRightHand);

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

public:
	frc::LiveWindow* lw = LiveWindow::GetInstance();

	typedef enum
	{
		Straight,
		Switch,
		Scale,
		BestSwitch,
		BestScale,
	}AutoRoutine;
	typedef enum
	{
		Middle,
		Left,
		Right
	}StartingSpot;

	frc::SendableChooser<AutoRoutine> autoGoal;
	const std::string autoGoal0 = "Straight";
	const std::string autoGoal1 = "Switch";
	const std::string autoGoal2 = "Scale";
	const std::string autoGoal3 = "Best (Tie Break Switch)";
	const std::string autoGoal4 = "Best (Tie Break Scale)";
	AutoRoutine autoSelected;

	frc::SendableChooser<StartingSpot> startSpot;
	const std::string spotMid   = "Middle";
	const std::string spotLeft  = "Left";
	const std::string spotRight = "Right";
	StartingSpot spotSelected;

	int autoStep;
	Vector2D autoVecErr;
	double turnErr;
	Timer autoTimer;

	double simx, simy;
	bool useSimData;
	Preferences *prefs;


	bool gotData ;//have we gotten data from FMS
	//Left = true, false = right
	bool ourSwitch;
	bool scale;
	bool theirSwitch;

	//Nav coefficients
	double v0 = 12.0;
	double alpha = 1.0;

	double wRadius=4.0, wBase=24.0;

	double headingOffset;
	double oldLDrivePos, oldRDrivePos;
	Pose2D drivePos;
	bool turning;
	double angle;

	DifferentialMotorCommand dmc;
	VelocityVector vv;
	Vector2D goal;
	Vector2D u;

	BNO055 imu{I2C::Port::kOnboard};

	NavController nc{&drivePos};
	UnicycleController ucm{&drivePos};
	MechTechDifferential drive{0,2,1,3};
	Elevator ele{11,10};
	Claw claw{7,8,0};

	XboxController stick1{0};
	XboxController stick2{1};
	XboxController stick3{2};

	VibController v{&stick1,&stick2};
};

START_ROBOT_CLASS(Robot)
