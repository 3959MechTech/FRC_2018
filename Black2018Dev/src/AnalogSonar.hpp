#ifndef H_ANALOG_SONAR
#define H_ANALOG_SONAR

#include <WPILib.h>
#include <LiveWindow/LiveWindowSendable.h>
#include <LiveWindow/LiveWindow.h>
#include <HAL/HAL.h>

#include <string.h>

class AnalogSonar //: public SensorBase, public PIDSource, public LiveWindowSendable
{
private:
		AnalogInput *m_sensor;
		int		 	m_channel;
        float		m_voltsPerInch;
		float		m_DistanceOffset;
		bool		m_allocatedChannel;

public:
	//explicit DistanceSensor(uint32_t channel);
	// tom AnalogSonar(AnalogInput *channel);
	AnalogSonar(int  channel);
	virtual ~AnalogSonar();

	void 	init();

	double 	PIDGet();
	float 	GetDistance();
	float	GetDistanceOffset(){return m_DistanceOffset;};
	int		GetSampleRate(){return m_sensor->GetSampleRate();};

	void	setDistanceOffset(float offset){m_DistanceOffset = offset;}
	void 	setVoltsPerInch(float vdp){m_voltsPerInch = vdp;}

	void UpdateTable();
	void StartLiveWindowMode(){};
	void StopLiveWindowMode(){};
	std::string GetSmartDashboardType(){return "AnalogInput";};


		//static const float sensVolts = 0.009765625; // volts per Inch sensitivity


};



#endif /* H_ANALOG_SONAR */

