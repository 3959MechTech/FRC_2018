/*
 * Claw.hpp
 *
 *  Created on: Feb 10, 2018
 *      Author: Reuben
 */

#ifndef CLAW_HPP_
#define CLAW_HPP_

#include <WPILib.h>
#include <ctre/phoenix.h>
#include <cstdio>

class Claw
{

private:
	TalonSRX lm, rm;
	DigitalInput sensor;

	Timer 	fireTimer;
	double 	fireTime;
	FILE*	log;
	std::string logName;


public:
	Claw(int leftMotor, int rightMotor, int sensor);
    void Shoot(double speed);
    bool Feed(double speed);

    void ResetFire();
    bool isFiring();
    void Fire(double speed, double dur);

    void OpenLog(std::string name);
    void CloseLog();
    void Log();

    void SendData(std::string name = "Claw");
};
#endif /* CLAW_HPP_ */
