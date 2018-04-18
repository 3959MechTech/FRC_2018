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
#include "AnalogSonar.hpp"

#include <fstream>

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
		camera0.SetFPS(10);
		camera1.SetFPS(10);

		SetPeriod(.005);

		autoGoal.AddDefault(autoGoal0, AutoRoutine::Switch);
		autoGoal.AddObject(autoGoal1, AutoRoutine::Straight);
		autoGoal.AddObject(autoGoal2, AutoRoutine::Scale);
		autoGoal.AddObject(autoGoal3, AutoRoutine::BestSwitch);
		autoGoal.AddObject(autoGoal4, AutoRoutine::BestScale);
		frc::SmartDashboard::PutData("Auto Modes", &autoGoal);

		startSpot.AddDefault(spotMid,StartingSpot::Middle);
		startSpot.AddObject(spotLeft,StartingSpot::Left);
		startSpot.AddObject(spotRight,StartingSpot::Right);
		frc::SmartDashboard::PutData("Starting Spot", &startSpot);

		autoDetermined = false;

		nc.SetV0(v0);
		nc.SetAlpha(alpha);
		nc.SetOmega(1200);

		turning = false;
		headingOffset = 0.0;

		autoStep = 0;
		angleToTurn = 0.0;

		rearSonar.setVoltsPerInch(.009765625);
		rightSonar.setVoltsPerInch(.009765625);
		leftSonar.setVoltsPerInch(.009765625);

		drive.ConfigRobot(4.0,24.0,1440.0);

		drive.SetLeftFPID( .72,0.0,0.0,0,0);
		drive.SetRightFPID(.72,0.0,0.0,0,0);

		//drive.SetLeftFPID( .4,0.40,0.001,0.3,1);
		//drive.SetRightFPID(.4,0.40,0.001,0.3,1);

		drive.SetRamp(0.2);

		oldLDrivePos = drive.GetLeftEncoderPosition();
		oldRDrivePos = drive.GetRightEncoderPosition();

		imu.Begin();
		imu.SetCurrentQuaternionAsInit();
		imu.SetInitEulers(imu.GetVector(BNO055::vector_type_t::VECTOR_EULER));

		initPositions[StartingSpot::Middle][0] = 0.0;//X value
		initPositions[StartingSpot::Middle][1] = -7.0;//y value
		initPositions[StartingSpot::Left][0] = 0.0;//X value
		initPositions[StartingSpot::Left][1] = 142.70;//y value
		initPositions[StartingSpot::Right][0] = 0.0;//X value
		initPositions[StartingSpot::Right][1] = -142.7;//y value

		GetFMSData();
		SendData();
		lastLoopTime = 0.0;

		//StartLogging();
		logTest = false;
		logStatus = "init";

	}

	void DetermineAuto()
	{
		if(gotData && !autoDetermined)
		{
			if(spotSelected == Middle)
			{
				if(autoSelected != Straight)
				{autoSelected = MidSwitch;}
				autoDetermined = true;
			}
			if(spotSelected == Left)
			{
				switch(autoSelected)
				{
				case Switch: if(!ourSwitch)
							{autoSelected = FarSwitch;}
							break;
				case Scale: if(!scale)
							{autoSelected = FarScale;}
							break;
				case BestSwitch:	if(ourSwitch)
								{autoSelected = Switch;}
								if(!ourSwitch&&scale)
								{autoSelected = Scale;}
								if(!ourSwitch&&!scale)
								{autoSelected = FarSwitch;}
								break;
				case BestScale:	if(scale)
								{autoSelected = Scale;}
								if(ourSwitch&&!scale)
								{autoSelected = Switch;}
								if(!ourSwitch&&!scale)
								{autoSelected = FarScale;}
								break;
				default: autoSelected = Straight;
				}
				autoDetermined = true;
			}
			if(spotSelected == Right)
			{
				switch(autoSelected)
				{
				case Switch: if(ourSwitch)
							{autoSelected = FarSwitch;}
							break;
				case Scale: if(scale)
							{autoSelected = FarScale;}
							break;
				case BestSwitch:	if(!ourSwitch)
								{autoSelected = Switch;}
								if(ourSwitch&&!scale)
								{autoSelected = Scale;}
								if(ourSwitch&&scale)
								{autoSelected = FarSwitch;}
								break;
				case BestScale:	if(!scale)
								{autoSelected = Scale;}
								if(!ourSwitch&&scale)
								{autoSelected = Switch;}
								if(ourSwitch&&scale)
								{autoSelected = FarScale;}
								break;
				default: autoSelected = Straight;
				}
				autoDetermined = true;
			}
		}
	}
	void AutonomousInit() override
	{
		ResetPose();
/*
		bool logbool = StartLogging();
		if(logbool)
		{
			logStatus = "Passed";
		}else
		{
			logStatus = "Failing";
		}
*/
		GetFMSData();

		autoDetermined=false;

		autoSelected = autoGoal.GetSelected();
		spotSelected = startSpot.GetSelected();

		DetermineAuto();

		//Log("auto");
		SendData();
	}

	void AutonomousPeriodic()
	{
		loopTimer.Reset();
		loopTimer.Start();
		UpdateDrivePos();
		UpdateSonars();

		if(gotData && autoDetermined)
		{
			switch(autoSelected)
			{
			case Straight: 	AutoStraight();
							break;
			case MidSwitch: AutoMidSwitch();
							break;
			case Switch:	AutoNearSideSwitch();
							break;
			case Scale:		AutoNearSideScale2();
							break;
			case FarScale:	AutoFarSideScale();
							break;
			case FarSwitch:	AutoFarSideSwitch();
							break;
			default:		break;
			}


		}else
		{
			GetFMSData();
			DetermineAuto();
		}

		drive.Set(dmc);

		SendData();
		//Log("auto");
		loopTimer.Stop();
		lastLoopTime = loopTimer.Get();
		SmartDashboard::PutNumber("Loop Timer",lastLoopTime);
	}


	void TeleopPeriodic()
	{
		loopTimer.Reset();
		loopTimer.Start();
		UpdateDrivePos();
		UpdateSonars();

		double rt= stick2.GetTriggerAxis(frc::GenericHID::kRightHand);
		double lt= stick2.GetTriggerAxis(frc::GenericHID::kLeftHand);
		double s = 0.0;
		if(!stick3.GetBButton())
		{
			if(lt>.1)
			{
				//throttle shooter if at scale levels
				//and shooting +/- 36deg up or down range
				if(ele.GetEPos()>=Elevator::ScaleLow &&
						(
							fabs(drivePos.GetPhi())<(3.14159/5.0) ||
							fabs(drivePos.GetPhi())<(3.14159-3.14159/5.0)
						)
				)
				{
					s = lt*.7;//decrease power by 70%
					claw.Shoot(s);
				}else
				{
					claw.Shoot(lt);
					s = lt;
				}

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
			if(stick2.GetBumper(frc::GenericHID::kLeftHand))
			{
				claw.Shoot(1.0);
				s=1.0;
			}
		}

		SmartDashboard::PutNumber("Claw Speed", s);

		Vator();

		if(stick3.GetAButtonPressed())
		{
			autoStep=0;
			gotData = false;
			ResetPose();
			GetFMSData();
		}



		//

		if(stick3.GetBButton())
		{
			//AutoMidSwitch();
			drive.Set(dmc);
		}else
		{
			if(stick3.GetXButton())
			{
				//dmc = ucm.GetDifferentialMotorCommand(.5);
				drive.Set(ucm.GetDifferentialMotorCommand(500.0));
			}else
			{
				if(stick3.GetYButton())
				{
					nc.SetAngle(0.0);
					vv.w=nc.Turn();
					vv.v = 0.0;
					dmc = ucm.DifferentialOutput(vv);
					drive.Set(dmc);
				}else
				{
					double x = stick3.GetX(frc::GenericHID::kRightHand);
					double y = -stick3.GetY(frc::GenericHID::kRightHand);
					if(sqrt(x*x+y*y)>.2)
					{
						vv.v = y;
						vv.w = x;
						drive.Set(ucm.DifferentialOutput(vv));
					}
					else
					{
						Drive();
					}

				}
			}

		}

		if(stick3.GetBumperPressed(frc::GenericHID::kRightHand))
		{
			autoStep++;
		}
		if(stick3.GetBumperPressed(frc::GenericHID::kLeftHand))
		{
			autoStep=30;

		}
/*
		double foo = -stick2.GetY(frc::GenericHID::kLeftHand);
		if(fabs(foo)<.2)
		{
			foo = 0;
		}

		ele.SetMotorSpeed(foo);
//*/
		//Drive();
		v.VibrateTimer();

		SendData();
		//Log("tele");
		loopTimer.Stop();
		lastLoopTime = loopTimer.Get();
		SmartDashboard::PutNumber("Loop Timer",lastLoopTime);
	}



	void TeleopInit()
	{
		//ResetPose();
/*
		bool logbool = StartLogging();
		if(logbool)
		{
			logStatus = "Passed";
		}else
		{
			logStatus = "Failing";
		}
*/
		ele.SetEPos(ele.GetEPos());
		SendData();
	}

	void TestPeriodic() {
		UpdateDrivePos();
		UpdateSonars();

/*
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

		if(stick3.GetAButtonPressed())
		{
			autoStep=0;
			gotData = false;
			ResetPose();
			GetFMSData();
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
				if(stick3.GetYButton())
				{
					nc.SetAngle(0.0);
					vv.w=nc.Turn();
					vv.v = 0.0;
					dmc = ucm.DifferentialOutput(vv);
					drive.Set(dmc);
				}else
				{
					double x = stick3.GetX(frc::GenericHID::kRightHand);
					double y = -stick3.GetY(frc::GenericHID::kRightHand);
					if(sqrt(x*x+y*y)>.2)
					{
						vv.v = y;
						vv.w = x;
						drive.Set(ucm.DifferentialOutput(vv));
					}
					else
					{
						Drive();
					}

				}
			}

		}

		if(stick3.GetBumperPressed(frc::GenericHID::kRightHand))
		{
			autoStep++;
		}
		if(stick3.GetBumperPressed(frc::GenericHID::kLeftHand))
		{
			autoStep=30;

		}
*/
/*		double foo = -stick3.GetY(frc::GenericHID::kLeftHand);
		if(fabs(foo)<.2)
		{
			foo = 0;
		}

		ele.SetMotorSpeed(foo);
*/
		//Drive();
	//	v.VibrateTimer();
		Log("test");
	}

	void TestInit()
	{
		bool logbool = StartLogging();
		if(logbool)
		{
			logStatus = "Passed";
		}else
		{
			logStatus = "Failing";
		}
	}

	void DisabledPeriodic()
	{

		SmartDashboard::PutString("Log File Status",logStatus);


		if(stick3.GetAButton())
		{
			ResetPose();
		}
		autoStep = 0;

		GetFMSData();
		UpdateDrivePos();
		UpdateSonars();
		SendData();
	}
	void DisabledInit()
	{
		//ResetPose();
		StopLogging();
		GetFMSData();
		SendData();
	}

	void SendData()
	{
		simx = prefs->GetDouble("simx",-1.0);
		simy = prefs->GetDouble("simy",-1.0);
		useSimData = prefs->GetBoolean("useSimData",false);
		//SmartDashboard::PutNumber("Put simx",simx);
		//SmartDashboard::PutNumber("Put simy",simy);
		//SmartDashboard::PutNumber("headingOffset",headingOffset);
		SmartDashboard::PutNumber("Battery Power",DriverStation::GetInstance().GetBatteryVoltage());
		SmartDashboard::PutNumber("autoStep",autoStep);

		//SmartDashboard::PutNumber("NC PID Angle", nc.GetAngle());
		//SmartDashboard::PutNumber("NC PID Output", nc.Turn());
		//SmartDashboard::PutNumber("NC PID Error", nc.GetPIDError());
		SmartDashboard::PutNumber("Auto Angle Target", angleToTurn);
		SmartDashboard::PutNumber("Auto Angle Error", CleanAngle(angleToTurn-drivePos.GetPhi()));

		SmartDashboard::PutNumber("Front Sonar Distance", frontSonarDistance);
		SmartDashboard::PutNumber("Rear  Sonar Distance", rearSonarDistance);
		SmartDashboard::PutNumber("Left  Sonar Distance", leftSonarDistance);
		SmartDashboard::PutNumber("Right  Sonar Distance", rightSonarDistance);

		v0 = prefs->GetDouble("v0",nc.GetV0());
		alpha = prefs->GetDouble("alpha",nc.GetAlpha());
		nc.SetV0(v0);
		nc.SetAlpha(alpha);
		nc.SetOmega(prefs->GetDouble("maxOmega",nc.GetOmega()));
		//ucm.Set_maxOmega(prefs->GetDouble("maxOmega",ucm.Get_maxOmega()));
/*
		nc.SetP(prefs->GetDouble("NC P",nc.GetP()));
		nc.SetI(prefs->GetDouble("NC I",nc.GetI()));
		nc.SetD(prefs->GetDouble("NC D",nc.GetD()));

		double lf,lp,li,ld,rf,rp,ri,rd;
		lf = drive.GetLeftF();
		lp = drive.GetLeftP();
		li = drive.GetLeftI();
		ld = drive.GetLeftD();
		rf = drive.GetRightF();
		rp = drive.GetRightP();
		ri = drive.GetRightI();
		rd = drive.GetRightD();

		lf = prefs->GetDouble("Drive Left F",lf);
		lp = prefs->GetDouble("Drive Left P",lp);
		li = prefs->GetDouble("Drive Left I",li);
		ld = prefs->GetDouble("Drive Left D",ld);
		rf = prefs->GetDouble("Drive Right F",rf);
		rp = prefs->GetDouble("Drive Right P",rp);
		ri = prefs->GetDouble("Drive Right I",ri);
		rd = prefs->GetDouble("Drive Right D",rd);
*/
		drivePos.SendData("Drive Pose");
		ele.SendData();
		claw.SendData();
		drive.SendData();

		SmartDashboard::PutNumber("IMU Chip ID", imuChipId);
		SmartDashboard::PutNumber("IMU Euler X", eulers.x);
		SmartDashboard::PutNumber("IMU Euler Y", eulers.y);
		SmartDashboard::PutNumber("IMU Euler Z", eulers.z);

		SmartDashboard::PutNumber("autoSelected", autoSelected);

		std::string data;
		data = DriverStation::GetInstance().GetGameSpecificMessage();
		int dl =data.length();
		if(dl>0)
		{
//			SmartDashboard::PutNumber("GD Length", data.length());
//			SmartDashboard::PutString("GD Data", data);
			SmartDashboard::PutBoolean("our Switch", ourSwitch);
			SmartDashboard::PutBoolean("our scale", scale);
			SmartDashboard::PutBoolean("other Switch", theirSwitch);
		}

		SmartDashboard::PutNumber("Robot U X", u.GetX());
		SmartDashboard::PutNumber("Robot U Y", u.GetY());

		SmartDashboard::PutNumber("Robot VV V", vv.v);
		SmartDashboard::PutNumber("Robot VV W", vv.w);

		SmartDashboard::PutNumber("Robot DMC VL:", dmc.VL);
		SmartDashboard::PutNumber("Robot DMC VR:", dmc.VR);


	}

	void UpdateDrivePos()
	{

		//grab euler angles from imu and store in member vector.
		imuChipId = imu.GetChipID();
		eulers = imu.GetVector(BNO055::vector_type_t::VECTOR_EULER);

		//convert yaw angle to radians
		double phi = -CleanAngle(3.14159*eulers.x/180.0);
		phi = CleanAngle(phi-headingOffset); //apply offset

		double l,r,d;
		//update x and y position as long as we aren't doing a 0 point turn
		if(!turning)
		{
			//get the distance traveled by each wheel
			l = 3.14159*4.0*(drive.GetLeftEncoderPosition()/1440.0);
			r = 3.14159*4.0*(drive.GetRightEncoderPosition()/1440.0);

			//remove old position, so we are only adding the change in distance
			//find the distance the CENTER of the robot moved
			d = ((l-oldLDrivePos)+(r-oldRDrivePos))/2.0;

			if(useSimData) //used to simulate robot moving
			{
				drivePos.SetX(simx);
				drivePos.SetY(simy);
			}else
			{
				//update position based on heading and distance moved.
				drivePos.SetX( drivePos.GetX() + d*cos(phi));
				drivePos.SetY(drivePos.GetY() + d*sin(phi));
			}
			//store current l and r into old values to be removed next time
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
		oldLDrivePos = 0.0;
		oldRDrivePos = 0.0;
		drivePos.SetX(initPositions[spotSelected][0]);
		drivePos.SetY(initPositions[spotSelected][1]);
		headingOffset = eulers.x;
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
		vv.v = lY*1500.0;//stick value multiplied by max translational speed
		vv.w = rX*1750.0;//stick value multiplied by max rotational speed


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

		if(ele.GetEncoderPos()<ele.GetHeight(Elevator::ScaleLow))
		{
			drive.SetMaxSpeed(1.0);
		}else
		{
			if(ele.GetEncoderPos()>ele.GetHeight(Elevator::ScaleMedium))
			{
				drive.SetMaxSpeed( ele.GetMaxSpeed(Elevator::ScaleHigh));
			}else
			{
				if(ele.GetEncoderPos()>ele.GetHeight(Elevator::ScaleLow))
				{
					drive.SetMaxSpeed( ele.GetMaxSpeed(Elevator::ScaleMedium));
				}else
				{
					drive.SetMaxSpeed( ele.GetMaxSpeed(Elevator::ScaleLow));
				}
			}
			drive.SetMaxSpeed( ele.GetMaxSpeed(ele.GetEPos()));
		}
		//drive.SetMaxSpeed( ele.GetMaxSpeed(ele.GetEPos()));

		//ele.SetMotorSpeed(s2y);
	}



	void GetFMSData()
	{
		autoSelected = autoGoal.GetSelected();//Get selected Auto Routine
		spotSelected = startSpot.GetSelected();//Get selected Auto Routine

		std::string data;
		data = DriverStation::GetInstance().GetGameSpecificMessage();

		//if(!gotData)
		{

			if(data.length()>=3)
			{
				gotData = true;
				if(data.substr(0,1)=="L")
				{
					ourSwitch = true;
				}else
				{
					ourSwitch = false;
				}
				if(data.substr(1,1)=="L")
				{
					scale = true;
				}else
				{
					scale = false;
				}
				if(data.substr(2,1)=="L")
				{
					theirSwitch = true;
				}else
				{
					theirSwitch = false;
				}

			}
		}
	}



	inline double CleanAngle(double angle)
	{
		return atan2(sin(angle),cos(angle));
	}

	void AutoNearSideScale()
	{

		//AutoStraight();
		bool done = false;

		double ySign = 1.0;//used to flip the Y value
		if(!ourSwitch)
		{
			ySign = -1.0;
		}
		ucm.Set_maxOmega(2000);
		nc.SetP(1800.0);
		nc.SetI(0.0);
		nc.SetD(8000.0);

		switch(autoStep)
		{
				//deploy cube/shooter
		case 0: turning = false;
				ele.SetEPos(Elevator::Bottom);
				autoStep++;
				break;

				//drive straight 200"
		case 1: turning = false;
				angleToTurn = 0.0;
				nc.SetAngle(angleToTurn);
				nc.SetAlpha(.005);
				nc.SetOmega(1000.0);
				ele.SetEPos(Elevator::Switch);
				nc.SetV0(1100);
				done = DriveStraighter(300.0,initPositions[spotSelected][1],4.0, true);
				break;

				//setup 90deg turn
		case 2: angleToTurn = -ySign*3.14159/2.0;//-90 deg in rad for left side, 90 deg on right
				nc.SetAngle(angleToTurn);
				done = true;
				break;

				//turn
		case 3: turning = true;
				nc.SetOmega(1200);
				done = Turn();
				break;

				//align with wall
		case 4:	done = ApproachRearWall(8.0,800.0);
				//raise to lower scale here
				ele.SetEPos(Elevator::ScaleLow);
				break;
				//Depart wall
		case 5: done = DepartRearWall(26.0,550.0);
				ele.SetEPos(Elevator::ScaleHigh);
				break;
				//raise to high elevator
		case 6:	ele.SetEPos(Elevator::ScaleHigh);
				if(ele.GetError()<4000.0){done = true;} //wait till we are close.
				claw.ResetFire();
				break;

				//shoot!
		case 7:	claw.Fire(.6,.5);
				done = true;
				break;

				//are we done shooting
		case 8: done = !claw.isFiring();
				break;
				//Back up to wall
		case 9: done = ApproachRearWall(20.0, 700.0);
				break;
				//drop ele
		case 10: ele.SetEPos(Elevator::Bottom);
				done=true;
				break;

		case 11:if(ele.GetError()<4000){done = true;} //wait till we are close.
				break;
				//Turn towards second cube
		case 12:angleToTurn = atan2(ySign*83.0-drivePos.GetY(),200.0-drivePos.GetX());
				angleToTurn = CleanAngle(angleToTurn);
				nc.SetAngle(angleToTurn);
				done = true;
				break;
				//Straighten up turn
		case 13: turning = true;
				nc.SetOmega(1200);
				done = Turn();
				break;
				//Drive towards second cube
		case 14: turning = false;
				nc.SetOmega(1200.0);
				nc.SetV0(800.0);
				claw.Feed(1.0);
				done = DriveStraight(200.0,ySign*83.0,18.0, true);
				break;

		case 16: autoTimer.Reset();
				autoTimer.Start();
				done=true;
				break;

		case 17: if(autoTimer.Get()>.25){done=true;}
				break;

		case 18: nc.SetAlpha(.001);
				nc.SetV0(800);
				turning = false;
				nc.SetOmega(1200);
				ele.SetEPos(Elevator::Switch);
				done = DriveStraight(240.0,ySign*100.0,10.0, false);
				break;
//*/
		default: dmc.VL = 0.0; dmc.VR=0.0;
				claw.Feed(0);

		}

		if(done)
		{
			done = false;
			autoStep++;
		}

	}

	void AutoNearSideScale2()
	{

		//AutoStraight();
		bool done = false;


		double ySign = 1.0;//used to flip the Y value
		if(!ourSwitch)
		{
			ySign = -1.0;
		}
		ucm.Set_maxOmega(2000);
		nc.SetP(1800.0);
		nc.SetI(0.0);
		nc.SetD(8000.0);

		switch(autoStep)
		{
				//Enable Claw Intake to straighten up cube
				//deploy cube/shooter
		case 0: turning = false;
				ele.SetEPos(Elevator::Bottom);
				nc.SetV0(1400);
				autoStep++;
				claw.Feed(0.5);
				nc.SetOmega(1000.0);
				break;

				//Disables cube intake
				//drive straight 200"
		case 1: turning = false;
				angleToTurn = 0.0;
				nc.SetAngle(angleToTurn);
				nc.SetAlpha(.002);
				done = DriveStraight(266.0,initPositions[spotSelected][1],4.0, true);
				if(drivePos.GetX()>12.0 && ele.GetEPos()==Elevator::Bottom)
				{
					claw.Feed(0.3);
					ele.SetEPos(Elevator::Switch);
				}
				if(drivePos.GetX()>120.0 && ele.GetEPos()==Elevator::Switch)
				{
					ele.SetEPos(Elevator::ScaleHigh);
					nc.SetV0(1200);
					nc.SetOmega(900);
				}
				if(done && ele.GetError()>4000.0)
				{
					done = false;
				}
				claw.Feed(0.3);
				if(drivePos.GetX()>260.0){done= true;}
				break;

				//setup 90deg turn
		case 2: angleToTurn = atan2(ySign*60.0-drivePos.GetY(),320.0-drivePos.GetX());// was x=200 tr
				angleToTurn = CleanAngle(angleToTurn);
				//angleToTurn = -ySign*3.14159/2.5;//-60 deg in rad for left side, 90 deg on right
				nc.SetAngle(angleToTurn);
				done = true;
				nc.SetOmega(1200);
				break;

				//turn
		case 3: turning = true;
				nc.SetOmega(2000);//1000
				done = Turn();
				break;

		case 4: turning = false;
				nc.SetAlpha(.005);
				nc.SetOmega(1000.0);
				nc.SetV0(600);
				done = DriveStraight(280.0, ySign*130.0,8.0, false);
				break;

				//shoot!
		case 5:	claw.Fire(1.0,.5);
				done = true;
				break;

				//are we done shooting
		case 6: done = !claw.isFiring();
				break;

				//turn back
				//Turn towards second cube
		case 7: angleToTurn = atan2(ySign*92.0-drivePos.GetY(),210.0-drivePos.GetX());// was x=200 tr
		        angleToTurn = CleanAngle(angleToTurn);
				nc.SetAngle(angleToTurn);
				nc.SetOmega(900.0);
				done = true;
				break;
				//Straighten up turn
		case 8: turning = true;
				if(fabs(drivePos.GetPhi())>(3.14159/1.50))
				{
					nc.SetOmega(800.0);
					ele.SetEPos(Elevator::Bottom);
				}
				if(fabs(ele.GetEncoderPos())<(ele.GetHeight(Elevator::Switch)+4000.0))
				{
					nc.SetOmega(1500.0);
				}
				done = Turn();
				break;
		case 9: if(fabs(ele.GetError())<4000.0){done = true;}
				break;
		case 10: done = true;
//				drivePos.SetX(280.0);
//				drivePos.SetY(80.0);
				break;
				//Drive towards second cube
		case 11: turning = false;
				nc.SetOmega(1200.0);
				nc.SetV0(800.0);
				ele.SetEPos(Elevator::Bottom);
				claw.Feed(1.0);
				done = DriveStraight(210.0,ySign*92.0,6.0, true);//was x=200 tr
				break;

		case 12: autoTimer.Reset();
				 autoTimer.Start();
				done=true;//was true
				break;

		case 13: if(autoTimer.Get()>.5){done=true;}
//				drivePos.SetX(240.0);
//				drivePos.SetY(80.0);
				break;

		//set elevator position to switch and wait for it to get their to go to next step
		case 14: turning = false;
				 if(drivePos.GetX()<230.0)
				 {
					dmc.VL=-800.0;
					dmc.VR=-800.0;
				 }else
				 {
					dmc.VL=-800.0+ySign*100.0;
					dmc.VR=-800.0-ySign*100.0;
				 }
				 ele.SetEPos(Elevator::ScaleHigh);
				 if(drivePos.GetX()>240.0)
				 {
					 claw.Feed(.3);
					 dmc.VL= 0.0;
					 dmc.VR= 0.0;
					 done = true;
				 }
				 break;
		//setup turn to scale
		case 15: angleToTurn = -ySign*3.14159/3.0;
				 nc.SetAngle(angleToTurn);
				 nc.SetOmega(1200);
				 done = true;
				 break;
		//turn
		case 16:  turning = true;
				//nc.SetOmega(1000);//1000
				done = Turn();
				break;
		//approach scale
		case 17:turning = false;
				nc.SetAlpha(.005);
				nc.SetOmega(1000.0);
				nc.SetV0(900);
				ele.SetEPos(Elevator::ScaleHigh);
				done = true;
				break;
		//wait for elevator to raise
		case 18: if((ele.GetEncoderPos()>=(ele.GetHeight(Elevator::ScaleHigh)-4000.0)))
				 {
					nc.SetV0(700);
					//done = DriveStraight(270.0, ySign*130.0,12.0, false);
					done = true;
				 }
				//claw.ResetFire();
				break;
		//fire
		case 19: claw.ResetFire();
				 claw.Fire(.8,.5);
				 done = true;
				 break;
		//wait for fire to finish
		case 20: done = !claw.isFiring();
				 break;

		case 21: claw.Feed(0.0);
				 //ele.SetEPos(Elevator::Bottom);
				 angleToTurn = atan2(ySign*70.0-drivePos.GetY(),210.0-drivePos.GetX());// was x=200 tr
				 angleToTurn = CleanAngle(angleToTurn);
				 nc.SetAngle(angleToTurn);
				 nc.SetOmega(1000.0);
				 done = true;
				 break;
		case 22: turning = true;
				 done = Turn();
				 if(fabs(drivePos.GetPhi())<(3.14159/2.0))
				 {
				 	//nc.SetOmega(600.0);
					ele.SetEPos(Elevator::Bottom);
				 }
				 break;
//*/
		default: turning = false;
				 dmc.VL = 0.0; dmc.VR=0.0;
				 claw.Feed(0);

		}

		if(done)
		{
			done = false;
			autoStep++;
		}

	}

	void AutoFarSideScale()
	{
		//AutoStraight();

		bool done = false;

		double ySign = 1.0;//used to flip the Y value
		if(!ourSwitch)
		{
			ySign = -1.0;
		}
		ucm.Set_maxOmega(2000);

		nc.SetP(1800.0);
		nc.SetI(0.0);
		nc.SetD(8000.0);

		switch(autoStep)
		{
		case 0: turning = false;
				ele.SetEPos(Elevator::Bottom);
				autoStep++;
				break;
		case 1: turning = false;
				angleToTurn = 0.0;
				nc.SetAngle(angleToTurn);
				nc.SetAlpha(.005);
				ele.SetEPos(Elevator::Travel);
				nc.SetV0(1000);
				done = DriveStraight(222.0,initPositions[spotSelected][1],4.0, true);
				break;
		case 2: angleToTurn = ySign*3.14159/2.0;
				nc.SetAngle(angleToTurn);
				//ele.SetEPos(Elevator::Switch);
				done = true;
				break;
		case 3: turning = true;
				nc.SetOmega(900);
				done = Turn();
				break;

		case 4: nc.SetOmega(1000);
				nc.SetAlpha(.005);
				nc.SetV0(700);
				done = DriveStraight(222.0, 0.0, 12.0, false);
				break;

		default: dmc.VL = 0.0; dmc.VR=0.0;
		}

		if(done)
		{
			done = false;
			autoStep++;
		}
	}

	void AutoFarSideSwitch()
	{
		//AutoStraight();
		AutoFarSideScale();
	}

	void AutoNearSideSwitch()
	{
		bool done = false;

		double ySign = 1.0;//used to flip the Y value
		if(!ourSwitch)
		{
			ySign = -1.0;
		}
		ucm.Set_maxOmega(2000);

		nc.SetP(1800.0);
		nc.SetI(0.0);
		nc.SetD(8000.0);

		switch(autoStep)
		{
		case 0: turning = false;
				ele.SetEPos(Elevator::Bottom);
				autoStep++;
				break;
		case 1: turning = false;
				angleToTurn = 0.0;
				nc.SetAngle(angleToTurn);
				nc.SetAlpha(.005);
				ele.SetEPos(Elevator::Switch);
				nc.SetV0(1200);
				done = DriveStraighter(140.0,initPositions[spotSelected][1],4.0, true);
				break;
		case 2: angleToTurn = -ySign*3.14159/2.0;
				nc.SetAngle(angleToTurn);
				ele.SetEPos(Elevator::Switch);
				done = true;
				break;
		case 3: turning = true;
				nc.SetOmega(1200);
				done = Turn();
				break;
		case 4: nc.SetAlpha(.001);
				nc.SetV0(800);
				turning = false;
				nc.SetOmega(1200);
				done = ApproachWall(13.0,800.0);
				break;
		case 5: turning = false;
				autoStep++;
				autoTimer.Start();
				break;
		case 6:	turning = false;
				claw.Shoot(.5);
				if(autoTimer.Get()>0.5)
				{
					autoTimer.Stop();
					autoStep++;
				}
				break;
		case 7: turning = false;
				claw.Shoot(0.0);
				autoStep++;
				break;
		case 8: nc.SetAlpha(.001);
				nc.SetV0(800);
				turning = false;
				nc.SetOmega(1200);
				done = ApproachRearWall(24.0,700);
				break;
/*		case 9: ele.SetEPos(Elevator::Bottom);
				angleToTurn = -ySign*3.14159/4.0;
				nc.SetAngle(angleToTurn);
				done = true;
				break;
		case 10: turning = true;
				nc.SetOmega(1200);
				done = Turn();
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

	void AutoMidSwitch()
	{
		bool done = false;

		double ySign = 1.0;//used to flip the Y value
		//If switch is on the right side, all Y points and angles are inverted
		if(!ourSwitch)
		{
			ySign = -1.0;
		}

		//set max turning speed for Nav Controller
		ucm.Set_maxOmega(2000);

		//Auto routine State Machine
		switch(autoStep)
		{
				//Set elevator to bottom position to deploy cube
		case 0: turning = false;//make sure that we are updating robot x&y position
				ele.SetEPos(Elevator::Bottom);//set elevator to bottom value (above mechanical limit)
				autoStep++;//go to next step on the next pass
				break;

				//drive straight enough to turn
		case 1: turning = false;
				angleToTurn = 0.0;//we need to drive straight at this heading
				nc.SetAngle(angleToTurn);//set the angle in the PID
				nc.SetAlpha(.005);//set how soon we start dropping speed to stop
				ele.SetEPos(Elevator::Travel);//raise up to travel height
				nc.SetV0(1200);
				done = DriveStraight(24.0,initPositions[Middle][1],4.0, true);//drive straight 24"
				if(drivePos.GetX()>16.0){done = true;}
				break;

				//do math for turning to the next way point
				//next way point is 70,70, or 70,-70
		case 2: angleToTurn = atan2(ySign*50.0-drivePos.GetY(),70.0-drivePos.GetX());
				nc.SetAngle(angleToTurn);//set angle in turn controller
				done = true;//progress to next step
				break;
				//turn to way point
		case 3: turning = true;//start turn
				nc.SetOmega(1200);//set max turn speed
				done = Turn();//turn
				break;
				//drive to waypoint
		case 4: nc.SetAlpha(.001);//need longer ramp down of speed to kill momentum
				nc.SetV0(1200);//set max speed
				turning = false;//turn off turning flag to update x,y again
				ele.SetEPos(Elevator::Switch);//raise block for deployment
				nc.SetOmega(1200);//set max turn speed
				done = DriveStraight(70.0,ySign*50.0,6.0, false);//drive
				if((drivePos.GetY()>42.0 && ySign==1.0)||(drivePos.GetY()<-42.0 && ySign==-1.0)){done = true;}
				break;

				//setup turn to switch
		case 5: angleToTurn = 0.0;//set angle to face switch
				nc.SetAngle(angleToTurn);//set angle
				turning = true;//set turn flag
				done = true;//go to next step
				break;

				//turn to switch
		case 6:	nc.SetOmega(1100);//set max speed
				done = Turn();//turn
				if(!(fabs(drivePos.GetPhi())<.02))//check error, error was passing through check
				{
					done = false;
				}
				break;

				//Approach switch wall
		case 7: turning = false;//turn off turn flag
				nc.SetAlpha(.005);//set sensitivity of curve
				//nc.SetV0(1200);
				ucm.Set_maxOmega(1000);//set max drive speed
				done = ApproachWall(16.0,900);//approach wall within 13"
				if(frontSonar.GetDistance()<16.0)
				{
					done = true;
				}else
				{
					done = false;
				}
				vv.w=0;//no turning allowed
				break;

				//FIRE!!!!
		case 8: turning = false;
				autoStep++;//go to next step
				autoTimer.Reset();
				autoTimer.Start();//Start timer
				break;
				//FIRE!!!
		case 9:	turning = false;
				claw.Shoot(.5);//set shooter speed
				if(autoTimer.Get()>0.5)//shoot for at least .75sec
				{
					autoTimer.Stop();//stop timer
					autoStep++;//next step
				}
				break;
				//HOLD FIRE!
		case 10: turning = false;
				claw.Shoot(0.0);//stop shooting
				autoStep++;//Next step
				break;

				//reset drive position,
				//assuming we lost position by hitting wall
		case 11: drivePos.SetX(90.0);//update with sonar
				drivePos.SetY(ySign*50.0);//pray this is right
				done = true;
				break;

				//backup to
		case 12: nc.SetAlpha(.005);
				nc.SetV0(800);
				turning = false;
				nc.SetOmega(1200);
				done = DriveStraight(30.0,drivePos.GetY(),4.0, true);
				break;

				//setup turn to cubes
		case 13: angleToTurn = atan2((ySign*1.0-drivePos.GetY()),(80.0-drivePos.GetX()));
				nc.SetAngle(angleToTurn);
				ele.SetEPos(Elevator::Bottom);
				done = true;
				break;

				//turn to cubes
		case 14: turning = true;
				nc.SetOmega(2000);
				done = Turn();
				break;

				//approach cubes
		case 15: nc.SetAlpha(.001);
				nc.SetV0(800);
				turning = false;
				nc.SetOmega(1000);
				claw.Feed(1.0);
				done = DriveStraight(75.0,ySign*1.0,16.0, false);
				break;

		case 16: autoTimer.Reset();
				autoTimer.Start();
				done=true;
				break;
		case 17: if(autoTimer.Get()>.25){done=true;}
				break;
		case 18: nc.SetAlpha(.001);
				nc.SetV0(800);
				turning = false;
				nc.SetOmega(1200);
				claw.Feed(.8);
				ele.SetEPos(Elevator::Travel);
				done = DriveStraight(30.0,ySign*50.0,10.0, false);
				break;
				//setup turn to switch

		case 19: angleToTurn = 0.0;//set angle to face switch
				nc.SetAngle(angleToTurn);//set angle
				turning = true;//set turn flag
				claw.Feed(0.3);
				done = true;//go to next step
				break;

				//turn to switch
		case 20:nc.SetOmega(1500);//set max speed
				done = Turn();//turn
				ele.SetEPos(Elevator::Switch);
				if(!(fabs(drivePos.GetPhi())<.02))//check error, error was passing through check
				{
					done = false;
				}
				break;

				//Approach switch wall
		case 21: turning = false;//turn off turn flag
				nc.SetAlpha(.005);//set sensitivity of curve
				//nc.SetV0(1200);
				ucm.Set_maxOmega(1000);//set max drive speed
				done = ApproachWall(16.0,700);//approach wall within 13"
				if(frontSonar.GetDistance()<16.0)
				{
					done = true;
				}else
				{
					done = false;
				}
				vv.w=0;//no turning allowed
				break;

				//FIRE!!!!
		case 22: turning = false;
				autoStep++;//go to next step
				autoTimer.Reset();
				autoTimer.Start();//Start timer
				break;
				//FIRE!!!
		case 23:	turning = false;
				claw.Shoot(.5);//set shooter speed
				if(autoTimer.Get()>0.5)//shoot for at least .75sec
				{
					autoTimer.Stop();//stop timer
					autoStep++;//next step
				}
				break;
				//HOLD FIRE!
		case 24: turning = false;
				claw.Shoot(0.0);//stop shooting
				autoStep++;//Next step
				break;

				//reset drive position,
				//assuming we lost position by hitting wall
		case 25: drivePos.SetX(90.0);//update with sonar
				drivePos.SetY(ySign*50.0);//pray this is right
				done = true;
				break;

				//backup to
		case 26: nc.SetAlpha(.005);
				nc.SetV0(800);
				turning = false;
				nc.SetOmega(1200);
				done = DriveStraight(30.0,drivePos.GetY(),4.0, true);
				break;

				//setup turn to cubes
		case 27: angleToTurn = atan2((ySign*1.0-drivePos.GetY()),(80.0-drivePos.GetX()));
				nc.SetAngle(angleToTurn);
				ele.SetEPos(Elevator::Bottom);
				done = true;
				break;

				//turn to cubes
		case 28: turning = true;
				nc.SetOmega(2000);
				done = Turn();
				break;

				//approach cubes
		case 29: nc.SetAlpha(.001);
				nc.SetV0(800);
				turning = false;
				nc.SetOmega(1200);
				claw.Feed(1.0);
				done = DriveStraight(87.0,ySign*7.0,16.0, false);
				break;



//*/
		default: dmc.VL = 0.0; dmc.VR=0.0; claw.Feed(0.0);
		}

		if(done)
		{
			done = false;
			autoStep++;
		}
	}

	void AutoStraight()
	{
		nc.SetAlpha(.001);//max sensitivity
		nc.SetV0(1000);//max speed of 1000, Coach said 1200 looked crazy
		ele.SetEPos(Elevator::Travel);
		DriveStraight(120.0,0.0,4.0, true);
		if((120.0 - drivePos.GetX())<2.0)
		{
			dmc.VL = 0.0;
			dmc.VR = 0.0;
			ele.SetEPos(Elevator::Switch);
		}
	}

	bool DriveCurved(double x, double y, double errOk, bool xMatters)
	{
		u   = nc.GoToGoal(x, y);

		double headingError = atan((y-drivePos.GetY())/(x - drivePos.GetX()));//calc target heading
		headingError = CleanAngle(headingError);//clean target heading
		headingError = CleanAngle(headingError-drivePos.GetPhi());//find heading error

		vv.v = u.GetX();
		vv.w = (headingError/3.14159)*1000;

		dmc = ucm.DifferentialOutput(vv);

		autoVecErr.SetX(x-drivePos.GetX());
		autoVecErr.SetY(y-drivePos.GetY());

		double matters;
		if (xMatters)
		{
			matters = autoVecErr.GetX();
		}else
		{
			matters = autoVecErr.GetY();
		}

		if(fabs(matters)>=errOk)//if(e.GetX()>=errOk)
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

	bool DriveStraight(double x, double y, double errOk, bool xMatters)
	{
		u   = nc.GoToGoal(x, y);
		//vv  = ucm.Tracker(u.GetX(), u.GetY());
		vv.v = u.GetX();
		vv.w = u.GetY();
		dmc = ucm.DifferentialOutput(vv);
		//dmc = ucm.GetDifferentialMotorCommand(u.GetX(), u.GetY());

		autoVecErr.SetX(x-drivePos.GetX());
		autoVecErr.SetY(y-drivePos.GetY());

		double matters;
		if (xMatters)
		{
			matters = autoVecErr.GetX();
		}else
		{
			matters = autoVecErr.GetY();
		}

		if(fabs(matters)>=errOk)//if(e.GetX()>=errOk)
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
	bool DriveStraighter(double x, double y, double errOk, bool xMatters)
	{
		u   = nc.GoToGoal(x, y);
		//vv  = ucm.Tracker(u.GetX(), u.GetY());
		vv.v = u.GetX();
		vv.w = 0.0;
		dmc = ucm.DifferentialOutput(vv);
		//dmc = ucm.GetDifferentialMotorCommand(u.GetX(), u.GetY());

		autoVecErr.SetX(x-drivePos.GetX());
		autoVecErr.SetY(y-drivePos.GetY());

		double matters;
		if (xMatters)
		{
			matters = autoVecErr.GetX();
		}else
		{
			matters = autoVecErr.GetY();
		}

		if(fabs(matters)>=errOk)//if(e.GetX()>=errOk)
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

	bool Turn()
	{
		turnErr = angleToTurn - drivePos.GetPhi();
		turnErr = CleanAngle(turnErr);
		vv.w = nc.Turn();
		vv.v = 0;
		dmc = ucm.DifferentialOutput(vv);
		if(fabs(turnErr)<.02)//less than 1 deg error
		{
			return true;
		}
		return false;
	}


	bool Turn(double angle)
	{

		turnErr = angle-drivePos.GetPhi();
		turnErr = CleanAngle(turnErr);

		vv = ucm.Tracker(nc.Turn(angle));
		dmc = ucm.DifferentialOutput(vv);
		SmartDashboard::PutNumber("Turning Error", sqrt(turnErr*turnErr));
		if(sqrt(turnErr*turnErr)<.04 && vv.w<100.0)//less than 2 deg error
		{
			return true;
		}
		return false;
	}

	bool ApproachWall(double goal, double maxSpeed)
	{
		double e = frontSonar.GetDistance()-goal;

		if(fabs(e)>1.0)
		{
			//if(e>0)
			{
				vv.v = maxSpeed*(1-exp(-.005*e*e));
			//}else
			//{
				//vv.v = -maxSpeed*(1-exp(-.1*e*e));
			}
			vv.w = 0.0;
			dmc = ucm.DifferentialOutput(vv);
			return false;
		}else
		{
			vv.v = 0.0;
			vv.w = 0.0;
			dmc = ucm.DifferentialOutput(vv);
			return true;
		}

	}
	bool ApproachRearWall(double goal, double maxSpeed)
	{
		double e = rearSonar.GetDistance()-goal;

		if(fabs(e)>1.0)
		{
			//if(e>0)
			{
				vv.v = -maxSpeed*(1-exp(-.1*e*e));
			//}else
			//{
				//vv.v = -maxSpeed*(1-exp(-.1*e*e));
			}
			vv.w = 0.0;
			dmc = ucm.DifferentialOutput(vv);
			return false;
		}else
		{
			vv.v = 0.0;
			vv.w = 0.0;
			dmc = ucm.DifferentialOutput(vv);
			return true;
		}

	}

	bool DepartRearWall(double goal, double maxSpeed)
	{
		double e = rearSonar.GetDistance()-goal;

		if(fabs(e)>1.0)
		{
			//if(e>0)
			{
				vv.v = maxSpeed*(1-exp(-.01*e*e));
			//}else
			//{
				//vv.v = -maxSpeed*(1-exp(-.1*e*e));
			}
			vv.w = 0.0;
			dmc = ucm.DifferentialOutput(vv);
			return false;
		}else
		{
			vv.v = 0.0;
			vv.w = 0.0;
			dmc = ucm.DifferentialOutput(vv);
			return true;
		}

	}
	void RollSafety()
	{
		if(fabs(eulers.z)>8.0)//roll angle of +/-8 deg lower tower.
		{
			//ele.SetEPos(Elevator::Bottom);
		}
	}

	void UpdateSonars()
	{
		frontSonarDistance = frontSonar.GetDistance();
		rearSonarDistance = rearSonar.GetDistance();
		leftSonarDistance = leftSonar.GetDistance();
		rightSonarDistance = rightSonar.GetDistance();
	}

	bool StartLogging()
	{
		StopLogging();

		int id;

		logIds.open("/home/lvuser/LogId.txt", std::ios::in);
		if(logIds.is_open())
		{
			logIds.seekg(-1,std::ios_base::end);
			while(1)
			{
				char c;
				logIds.get(c);
				//If the data was at or before the 0th byte quit
				if((int)logIds.tellg()<=1)
				{
					logIds.seekg(0);
					break;
				}
				if(c=='\n')
				{
					break;
				}else
				{
					logIds.seekg(-2,std::ios_base::cur);
				}
			}
			//get last line
			std::string lastLine;
			std::getline(logIds,lastLine);
			//convert id to int
			if(lastLine.length()>0)
			{
				id = std::atoi(lastLine.c_str());
			}else
			{
				id = 0;
			}
			id++;//increment for new id

			//update log id file
			logIds.close();



		}else
		{
			id = 1;
		}

		logIds.open("/home/lvuser/LogId.txt", std::ios::out|std::ios::app);

		if(logIds.is_open())
		{
			if(id==1)
			{
				logIds << std::to_string(id);
			}else
			{
				logIds << "\n"+ std::to_string(id);
			}
			logIds.close();
		}
		std::string fn = "/home/lvuser/log/log_"+std::to_string(id) + ".csv";
		log.open(fn, std::ios::out);
		if(log.is_open())
		{
			std::string cols;
			cols = LogHeader();
			log << cols<<std::endl;
		}else
		{
			return false;
		}
		return true;
	}
	std::string LogHeader()
	{
		std::string cols;
		cols =  "state, time, battVolt, battCurrent, lastLoopTime, imuId, x, y, phi, yaw, roll, pitch, ";
		cols += "userAuto, userPos, ourSwitch, scale, autoSelected, autoStep, ";
		cols += "autoAngleTarget, autoAngleError, Ux, Uy, VV.v, VV.w, VL, VR, ";
		cols += "frontSonar, rearSonar, leftSonar, rightSonar, ";
		cols += drive.GetLogCols() + ", ";
		cols += ele.GetLogCols() + ", ";
		cols += claw.GetLogCols();
		return cols;
	}

	void Log(std::string state)
	{
		std::string data;
		data =  state + ", ";
		data += std::to_string(RobotController::GetFPGATime()) + ", ";
		data += std::to_string(pdp.GetVoltage()) + ", ";
		data += std::to_string(pdp.GetTotalCurrent()) + ", ";
		data += std::to_string(lastLoopTime) + ", ";
		data += std::to_string(imuChipId) + ", ";
		data += std::to_string(drivePos.GetX()) + ", ";
		data += std::to_string(drivePos.GetY()) + ", ";
		data += std::to_string(drivePos.GetPhi()) + ", ";
		data += std::to_string(eulers.x) + ", ";
		data += std::to_string(eulers.y) + ", ";
		data += std::to_string(eulers.z) + ", ";
		data += std::to_string(autoGoal.GetSelected()) + ", ";
		data += std::to_string(startSpot.GetSelected()) + ", ";
		data += std::to_string(ourSwitch) + ", ";
		data += std::to_string(scale) + ", ";
		data += std::to_string(autoSelected) + ", ";
		data += std::to_string(autoStep) + ", ";
		data += std::to_string(angleToTurn) + ", ";
		data += std::to_string(CleanAngle(angleToTurn-drivePos.GetPhi())) + ", ";
		data += std::to_string(u.GetX()) + ", ";
		data += std::to_string(u.GetY()) + ", ";
		data += std::to_string(vv.v) + ", ";
		data += std::to_string(vv.w) + ", ";
		data += std::to_string(dmc.VL) + ", ";
		data += std::to_string(dmc.VR) + ", ";
		data += std::to_string(frontSonarDistance) + ", ";
		data += std::to_string(rearSonarDistance) + ", ";
		data += std::to_string(leftSonarDistance) + ", ";
		data += std::to_string(rightSonarDistance) + ", ";
		data += drive.GetLogData() + ", ";
		data += ele.GetLogData() + ", ";
		data += claw.GetLogData();

		if(log.is_open())
		{
			log << std::endl << data;
		}


	}

	void StopLogging()
	{
		if(logIds.is_open())
		{
			logIds.close();
			logIds.clear();
		}

		if(log.is_open())
		{
			log.close();
		}
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
		FarSwitch,
		FarScale,
		MidSwitch
	}AutoRoutine;
	typedef enum
	{
		Middle = 0,
		Left = 1,
		Right = 2
	}StartingSpot;

	frc::SendableChooser<AutoRoutine> autoGoal;
	const std::string autoGoal1 = "Straight";
	const std::string autoGoal0 = "Switch";
	const std::string autoGoal2 = "Scale";
	const std::string autoGoal3 = "Best (Tie Break Switch)";
	const std::string autoGoal4 = "Best (Tie Break Scale)";
	AutoRoutine autoSelected;
	bool autoDetermined;

	frc::SendableChooser<StartingSpot> startSpot;
	const std::string spotMid   = "Middle";
	const std::string spotLeft  = "Left";
	const std::string spotRight = "Right";
	StartingSpot spotSelected;

	int autoStep;
	Vector2D autoVecErr;
	double turnErr;
	Timer autoTimer;

	Timer loopTimer;
	double lastLoopTime;


	double simx, simy;
	bool useSimData;
	Preferences *prefs;


	bool gotData ;//have we gotten data from FMS
	//Left = true, false = right
	bool ourSwitch;
	bool scale;
	bool theirSwitch;

	double initPositions[3][2];

	//Nav coefficients
	double v0 = 12.0;
	double alpha = 1.0;

	double wRadius=4.0, wBase=24.0;

	double headingOffset;
	double oldLDrivePos, oldRDrivePos;
	Pose2D drivePos;
	bool turning;
	double angleToTurn;
	double wpX, wpY;

	DifferentialMotorCommand dmc;
	VelocityVector vv;
	Vector2D goal;
	Vector2D u;

	std::fstream logIds;
	std::ofstream log;
	bool logTest;
	std::string logStatus;

	BNO055 imu{I2C::Port::kMXP};
	int imuChipId;
	Vector eulers;

	AnalogSonar frontSonar{0};
	AnalogSonar rearSonar{1};
	AnalogSonar leftSonar{2};
	AnalogSonar rightSonar{3};
	double frontSonarDistance;
	double rearSonarDistance;
	double leftSonarDistance;
	double rightSonarDistance;


	NavController nc{&drivePos};
	UnicycleController ucm{&drivePos};
	MechTechDifferential drive{0,2,1,3};
	Elevator ele{11,10};
	Claw claw{7,8,0};

	XboxController stick1{0};
	XboxController stick2{1};
	XboxController stick3{2};

	VibController v{&stick1,&stick2};
	PowerDistributionPanel pdp{};
};

START_ROBOT_CLASS(Robot)
