	/*
 * Pose2D.hpp
 *
 *  Created on: Jan 27, 2018
 *      Author: Cloie and Cole
 */

#ifndef SRC_POSE2D_HPP_
#define SRC_POSE2D_HPP_

//#include <support/deprecated.h>
//#include <support/mutex.h>
#include <WPILib.h>

class Pose2D: public PIDSource
{

private:
	 double _x,_y,_phi;
	 mutable wpi::mutex m_mutex;


public:

	Pose2D(){_x=0.0;_y=0.0;_phi=0.0;};
	Pose2D(double x, double y, double phi){_x=x;_y=y;_phi=phi;};

	virtual ~Pose2D()
	{

	}

	void SetX(double val)
	{
		std::lock_guard<wpi::mutex> sync(m_mutex);
		_x=val;
	};

	void SetY(double val)
	{
		std::lock_guard<wpi::mutex> sync(m_mutex);
	    _y=val;

	};
	void SetPhi(double val)
	{
		std::lock_guard<wpi::mutex> sync(m_mutex);
	    _phi=val;
	};

    double GetX()
    {
		std::lock_guard<wpi::mutex> sync(m_mutex);
		return _x;
    };

    double GetY()
    {
		std::lock_guard<wpi::mutex> sync(m_mutex);
		return _y;
    };
    double GetPhi()
    {
    		std::lock_guard<wpi::mutex> sync(m_mutex);
    		return _phi;
    }

    virtual double PIDGet() override
    {
    		std::lock_guard<wpi::mutex> sync(m_mutex);
    		return _phi;
    }

    void SendData(std::string name = "Pose2D")
    {
    		SmartDashboard::PutNumber(name+" X:",GetX());
    		SmartDashboard::PutNumber(name+" Y:",GetY());
    		SmartDashboard::PutNumber(name+" Phi:",GetPhi());
    }
 };



#endif /* SRC_POSE2D_HPP_ */
