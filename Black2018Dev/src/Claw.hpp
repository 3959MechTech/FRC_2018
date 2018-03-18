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


class Claw
{

private:
	TalonSRX lm, rm;
	DigitalInput sensor;

	Timer 	fireTimer;
	double 	fireTime;

public:
	Claw(int leftMotor, int rightMotor, int sensor);
    void Shoot(double speed);
    bool Feed(double speed);

    void ResetFire();
    bool isFiring();
    void Fire(double speed, double dur);


    void SendData(std::string name = "Claw");
};
#endif /* CLAW_HPP_ */
