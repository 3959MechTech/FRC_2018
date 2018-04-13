/*
 * DifferentialMotorControl.hpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Austin
 */
#ifndef SRC_DIFFERENTIALMOTORCOMMAND_HPP_
#define SRC_DIFFERENTIALMOTORCOMMAND_HPP_

#include "Pose2D.hpp"


class DifferentialMotorCommand
{

public:
	double VR,VL;
	DifferentialMotorCommand(){VR=0.0;VL=0.0;}

};







#endif /* SRC_DIFFERENTIALMOTORCOMMAND_HPP_ */
