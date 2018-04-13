/*
 * VibController.hpp
 *
 *  Created on: Feb 10, 2018
 *      Author: Jim
 */

#ifndef SRC_VIBCONTROLLER_HPP_
#define SRC_VIBCONTROLLER_HPP_

#include <WPILib.h>

class VibController
{
public:
	VibController(XboxController *s1,XboxController *s2)
	{
		stick1 = s1;
		stick2 = s2;

		times[0][0] = 40.0; //start
		times[0][1] = 39.85;//stop
		times[0][2] = 10000.0;//power

		times[1][0] = 39.71; //start
		times[1][1] = 39.57;//stop
		times[1][2] = 10000.0;//power

		times[2][0] = 39.42; //start
		times[2][1] = 39.28;//stop
		times[2][2] = 10000.0;//power

		times[3][0] = 39.14; //start
		times[3][1] = 39.0;//stop
		times[3][2] = 10000.0;//power

		times[4][0] = 30.0; //start
		times[4][1] = 29.8;//stop
		times[4][2] = 10000.0;//power

		times[5][0] = 29.6; //start
		times[5][1] = 29.4;//stop
		times[5][2] = 10000.0;//power

		times[6][0] = 29.2; //start
		times[6][1] = 29.0;//stop
		times[6][2] = 10000.0;//power

		times[7][0] = 20.0; //start
		times[7][1] = 19.45;//stop
		times[7][2] = 10000.0;//power

		times[8][0] = 18.7; //start
		times[8][1] = 18.0;//stop
		times[8][2] = 10000.0;//power

		times[9][0] = 15.0; //start
		times[9][1] = 14.8;//stop
		times[9][2] = 10000.0;//power

		times[10][0] = 14.6; //start
		times[10][1] = 14.4;//stop
		times[10][2] = 10000.0;//power

		times[11][0] = 14.2; //start
		times[11][1] = 14.0;//stop
		times[11][2] = 10000.0;//power

		times[12][0] = 13.8; //start
		times[12][1] = 13.6;//stop
		times[12][2] = 10000.0;//power

		times[13][0] = 13.4; //start
		times[13][1] = 13.2;//stop
		times[13][2] = 10000.0;//power

		times[14][0] = 13.0; //start
		times[14][1] = 12.8;//stop
		times[14][2] = 10000.0;//power

		times[15][0] = 12.6; //start
		times[15][1] = 12.4;//stop
		times[15][2] = 10000.0;//power

		times[16][0] = 12.2; //start
		times[16][1] = 12.0;//stop
		times[16][2] = 10000.0;//power

		times[17][0] = 11.8; //start
		times[17][1] = 11.6;//stop
		times[17][2] = 10000.0;//power

		times[18][0] = 11.4; //start
		times[18][1] = 11.2;//stop
		times[18][2] = 10000.0;//power

		times[19][0] = 11.0; //start
		times[19][1] = 10.8;//stop
		times[19][2] = 10000.0;//power

		times[20][0] = 10.6; //start
		times[20][1] = 10.4;//stop
		times[20][2] = 10000.0;//power

		times[21][0] = 10.2; //start
		times[21][1] = 10.0;//stop
		times[21][2] = 10000.0;//power
	}

	void Vibrate(double left, double right)
	{
		if(stick1 !=0)
		{
			stick1->SetRumble(frc::GenericHID::RumbleType::kLeftRumble,left);
			stick1->SetRumble(frc::GenericHID::RumbleType::kRightRumble,right);
		}
		if(stick2 !=0)
		{
			stick2->SetRumble(frc::GenericHID::RumbleType::kLeftRumble,left);
			stick2->SetRumble(frc::GenericHID::RumbleType::kRightRumble,right);
		}

	}

	void VibrateTimer()
	{
		double t = DriverStation::GetInstance().GetMatchTime();

		for(int i = 0; i<MaxTimes;i++)
		{
			if(t<times[i][0] && t>times[i][1])
			{
				Vibrate(times[i][2],times[i][2]);
				return;
			}
		}

		Vibrate(0.0,0.0);

	}

private:
	static const int MaxTimes = 22;

	XboxController *stick1, *stick2;
	double times[MaxTimes][3];



};


#endif /* SRC_VIBCONTROLLER_HPP_ */
