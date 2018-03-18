/*
 * AnalogSonar.cpp
 *
 *  Created on: Jan 15, 2018
 *      Author: Cole
 */


#include "AnalogSonar.hpp"

/*
AnalogSonar::AnalogSonar(uint32_t channel)
{
	m_channel=channel;
	m_sensor = new AnalogInput(m_channel);
	m_allocatedChannel = true;
	init();

}

AnalogSonar::AnalogSonar(AnalogInput *channel)
{
	if(channel == NULL)
	{
		wpi_setWPIError(NullParameter);
	}else
	{
		m_sensor = channel;
		m_channel = m_sensor->GetChannel();
		init();
	}
	m_allocatedChannel = false;

}
*/

AnalogSonar::AnalogSonar(int channel)
{
	m_sensor = new AnalogInput(channel);
	m_channel = channel;
	init();

	m_allocatedChannel = true;

}
AnalogSonar::~AnalogSonar()
{
	if(m_allocatedChannel)
	{
		delete m_sensor;
	}
}


void AnalogSonar::init()
{
	m_voltsPerInch = 0.0248046875;
//	LiveWindow::GetInstance()->AddSensor("AnalogSonar", m_sensor->GetChannel(), this);

	m_sensor->SetSampleRate(50000);
	m_sensor->SetAverageBits(5);
	m_sensor->SetOversampleBits(5);
}

float AnalogSonar::GetDistance()
{
	return (m_sensor->GetAverageVoltage())/m_voltsPerInch;
}

/*
double AnalogSonar::PIDGet()
{
	return GetDistance();
}
*/









