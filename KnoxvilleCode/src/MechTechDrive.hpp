/*
 * MechTechDrive.hpp
 *
 *  Created on: Feb 18, 2018
 *      Author: Jim
 */

#ifndef SRC_MECHTECHDRIVE_HPP_
#define SRC_MECHTECHDRIVE_HPP_

class MechTechDrive
{
public:
	typedef enum
	{
		LeftFront,
		LeftRear,
		RightFront,
		RightRear
	} WheelType;

	MechTechDrive(){};
	virtual ~MechTechDrive(){};

	virtual void SetRamp(double ramp){};
	virtual void SetMaxSpeed(double speed){};

	virtual double GetWheelPos(WheelType w){return 0.0;};
};



#endif /* SRC_MECHTECHDRIVE_HPP_ */
