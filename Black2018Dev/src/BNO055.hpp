
#ifndef SRC_BNO055_HPP_
#define SRC_BNO055_HPP_

#include "Quaternion.hpp"
#include <WPILib.h>
#include "Vector.h"
class BNO055: public Controller
{

public:
	typedef enum
	{
		BNO055_ADR_A = 0x28,
		BNO055_ADR_B = 0x29
	} bno055_i2c_adr;
	typedef enum
	{
	  /* Page id register definition */
	  BNO055_PAGE_ID_ADDR                                     = 0X07,

	  /* PAGE0 REGISTER DEFINITION START*/
	  BNO055_CHIP_ID_ADDR                                     = 0x00,
	  BNO055_ACCEL_REV_ID_ADDR                                = 0x01,
	  BNO055_MAG_REV_ID_ADDR                                  = 0x02,
	  BNO055_GYRO_REV_ID_ADDR                                 = 0x03,
	  BNO055_SW_REV_ID_LSB_ADDR                               = 0x04,
	  BNO055_SW_REV_ID_MSB_ADDR                               = 0x05,
	  BNO055_BL_REV_ID_ADDR                                   = 0X06,

	  /* Accel data register */
	  BNO055_ACCEL_DATA_X_LSB_ADDR                            = 0X08,
	  BNO055_ACCEL_DATA_X_MSB_ADDR                            = 0X09,
	  BNO055_ACCEL_DATA_Y_LSB_ADDR                            = 0X0A,
	  BNO055_ACCEL_DATA_Y_MSB_ADDR                            = 0X0B,
	  BNO055_ACCEL_DATA_Z_LSB_ADDR                            = 0X0C,
	  BNO055_ACCEL_DATA_Z_MSB_ADDR                            = 0X0D,

	  /* Mag data register */
	  BNO055_MAG_DATA_X_LSB_ADDR                              = 0X0E,
	  BNO055_MAG_DATA_X_MSB_ADDR                              = 0X0F,
	  BNO055_MAG_DATA_Y_LSB_ADDR                              = 0X10,
	  BNO055_MAG_DATA_Y_MSB_ADDR                              = 0X11,
	  BNO055_MAG_DATA_Z_LSB_ADDR                              = 0X12,
	  BNO055_MAG_DATA_Z_MSB_ADDR                              = 0X13,

	  /* Gyro data registers */
	  BNO055_GYRO_DATA_X_LSB_ADDR                             = 0X14,
	  BNO055_GYRO_DATA_X_MSB_ADDR                             = 0X15,
	  BNO055_GYRO_DATA_Y_LSB_ADDR                             = 0X16,
	  BNO055_GYRO_DATA_Y_MSB_ADDR                             = 0X17,
	  BNO055_GYRO_DATA_Z_LSB_ADDR                             = 0X18,
	  BNO055_GYRO_DATA_Z_MSB_ADDR                             = 0X19,

	  /* Euler data registers */
	  BNO055_EULER_H_LSB_ADDR                                 = 0X1A,
	  BNO055_EULER_H_MSB_ADDR                                 = 0X1B,
	  BNO055_EULER_R_LSB_ADDR                                 = 0X1C,
	  BNO055_EULER_R_MSB_ADDR                                 = 0X1D,
	  BNO055_EULER_P_LSB_ADDR                                 = 0X1E,
	  BNO055_EULER_P_MSB_ADDR                                 = 0X1F,

	  /* Quaternion data registers */
	  BNO055_QUATERNION_DATA_W_LSB_ADDR                       = 0X20,
	  BNO055_QUATERNION_DATA_W_MSB_ADDR                       = 0X21,
	  BNO055_QUATERNION_DATA_X_LSB_ADDR                       = 0X22,
	  BNO055_QUATERNION_DATA_X_MSB_ADDR                       = 0X23,
	  BNO055_QUATERNION_DATA_Y_LSB_ADDR                       = 0X24,
	  BNO055_QUATERNION_DATA_Y_MSB_ADDR                       = 0X25,
	  BNO055_QUATERNION_DATA_Z_LSB_ADDR                       = 0X26,
	  BNO055_QUATERNION_DATA_Z_MSB_ADDR                       = 0X27,

	  /* Linear acceleration data registers */
	  BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR                     = 0X28,
	  BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR                     = 0X29,
	  BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR                     = 0X2A,
	  BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR                     = 0X2B,
	  BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR                     = 0X2C,
	  BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR                     = 0X2D,

	  /* Gravity data registers */
	  BNO055_GRAVITY_DATA_X_LSB_ADDR                          = 0X2E,
	  BNO055_GRAVITY_DATA_X_MSB_ADDR                          = 0X2F,
	  BNO055_GRAVITY_DATA_Y_LSB_ADDR                          = 0X30,
	  BNO055_GRAVITY_DATA_Y_MSB_ADDR                          = 0X31,
	  BNO055_GRAVITY_DATA_Z_LSB_ADDR                          = 0X32,
	  BNO055_GRAVITY_DATA_Z_MSB_ADDR                          = 0X33,

	  /* Temperature data register */
	  BNO055_TEMP_ADDR                                        = 0X34,

	  /* Status registers */
	  BNO055_CALIB_STAT_ADDR                                  = 0X35,
	  BNO055_SELFTEST_RESULT_ADDR                             = 0X36,
	  BNO055_INTR_STAT_ADDR                                   = 0X37,

	  BNO055_SYS_CLK_STAT_ADDR                                = 0X38,
	  BNO055_SYS_STAT_ADDR                                    = 0X39,
	  BNO055_SYS_ERR_ADDR                                     = 0X3A,

	  /* Unit selection register */
	  BNO055_UNIT_SEL_ADDR                                    = 0X3B,
	  BNO055_DATA_SELECT_ADDR                                 = 0X3C,

	  /* Mode registers */
	  BNO055_OPR_MODE_ADDR                                    = 0X3D,
	  BNO055_PWR_MODE_ADDR                                    = 0X3E,

	  BNO055_SYS_TRIGGER_ADDR                                 = 0X3F,
	  BNO055_TEMP_SOURCE_ADDR                                 = 0X40,

	  /* Axis remap registers */
	  BNO055_AXIS_MAP_CONFIG_ADDR                             = 0X41,
	  BNO055_AXIS_MAP_SIGN_ADDR                               = 0X42,

	  /* SIC registers */
	  BNO055_SIC_MATRIX_0_LSB_ADDR                            = 0X43,
	  BNO055_SIC_MATRIX_0_MSB_ADDR                            = 0X44,
	  BNO055_SIC_MATRIX_1_LSB_ADDR                            = 0X45,
	  BNO055_SIC_MATRIX_1_MSB_ADDR                            = 0X46,
	  BNO055_SIC_MATRIX_2_LSB_ADDR                            = 0X47,
	  BNO055_SIC_MATRIX_2_MSB_ADDR                            = 0X48,
	  BNO055_SIC_MATRIX_3_LSB_ADDR                            = 0X49,
	  BNO055_SIC_MATRIX_3_MSB_ADDR                            = 0X4A,
	  BNO055_SIC_MATRIX_4_LSB_ADDR                            = 0X4B,
	  BNO055_SIC_MATRIX_4_MSB_ADDR                            = 0X4C,
	  BNO055_SIC_MATRIX_5_LSB_ADDR                            = 0X4D,
	  BNO055_SIC_MATRIX_5_MSB_ADDR                            = 0X4E,
	  BNO055_SIC_MATRIX_6_LSB_ADDR                            = 0X4F,
	  BNO055_SIC_MATRIX_6_MSB_ADDR                            = 0X50,
	  BNO055_SIC_MATRIX_7_LSB_ADDR                            = 0X51,
	  BNO055_SIC_MATRIX_7_MSB_ADDR                            = 0X52,
	  BNO055_SIC_MATRIX_8_LSB_ADDR                            = 0X53,
	  BNO055_SIC_MATRIX_8_MSB_ADDR                            = 0X54,

	  /* Accelerometer Offset registers */
	  ACCEL_OFFSET_X_LSB_ADDR                                 = 0X55,
	  ACCEL_OFFSET_X_MSB_ADDR                                 = 0X56,
	  ACCEL_OFFSET_Y_LSB_ADDR                                 = 0X57,
	  ACCEL_OFFSET_Y_MSB_ADDR                                 = 0X58,
	  ACCEL_OFFSET_Z_LSB_ADDR                                 = 0X59,
	  ACCEL_OFFSET_Z_MSB_ADDR                                 = 0X5A,

	  /* Magnetometer Offset registers */
	  MAG_OFFSET_X_LSB_ADDR                                   = 0X5B,
	  MAG_OFFSET_X_MSB_ADDR                                   = 0X5C,
	  MAG_OFFSET_Y_LSB_ADDR                                   = 0X5D,
	  MAG_OFFSET_Y_MSB_ADDR                                   = 0X5E,
	  MAG_OFFSET_Z_LSB_ADDR                                   = 0X5F,
	  MAG_OFFSET_Z_MSB_ADDR                                   = 0X60,

	  /* Gyroscope Offset register s*/
	  GYRO_OFFSET_X_LSB_ADDR                                  = 0X61,
	  GYRO_OFFSET_X_MSB_ADDR                                  = 0X62,
	  GYRO_OFFSET_Y_LSB_ADDR                                  = 0X63,
	  GYRO_OFFSET_Y_MSB_ADDR                                  = 0X64,
	  GYRO_OFFSET_Z_LSB_ADDR                                  = 0X65,
	  GYRO_OFFSET_Z_MSB_ADDR                                  = 0X66,

	  /* Radius registers */
	  ACCEL_RADIUS_LSB_ADDR                                   = 0X67,
	  ACCEL_RADIUS_MSB_ADDR                                   = 0X68,
	  MAG_RADIUS_LSB_ADDR                                     = 0X69,
	  MAG_RADIUS_MSB_ADDR                                     = 0X6A
	} bno055_reg_t;

	typedef enum
	{
	  POWER_MODE_NORMAL                                       = 0X00,
	  POWER_MODE_LOWPOWER                                     = 0X01,
	  POWER_MODE_SUSPEND                                      = 0X02
	} bno055_powermode_t;

	typedef enum
	{
	  /* Operation mode settings*/
	  OPERATION_MODE_CONFIG                                   = 0X00,
	  OPERATION_MODE_ACCONLY                                  = 0X01,
	  OPERATION_MODE_MAGONLY                                  = 0X02,
	  OPERATION_MODE_GYRONLY                                  = 0X03,
	  OPERATION_MODE_ACCMAG                                   = 0X04,
	  OPERATION_MODE_ACCGYRO                                  = 0X05,
	  OPERATION_MODE_MAGGYRO                                  = 0X06,
	  OPERATION_MODE_AMG                                      = 0X07,
	  OPERATION_MODE_IMUPLUS                                  = 0X08,
	  OPERATION_MODE_COMPASS                                  = 0X09,
	  OPERATION_MODE_M4G                                      = 0X0A,
	  OPERATION_MODE_NDOF_FMC_OFF                             = 0X0B,
	  OPERATION_MODE_NDOF                                     = 0X0C
	} bno055_opmode_t;
	std::string BNO055_Mode_String(bno055_opmode_t mode)
	{
		switch(mode)
		{
		case OPERATION_MODE_CONFIG: return "Config";
		case OPERATION_MODE_ACCONLY: return "Acc Only";
		case OPERATION_MODE_MAGONLY: return "Mag Only";
		case OPERATION_MODE_GYRONLY: return "Gyro Only";
		case OPERATION_MODE_ACCMAG: return "Acc+Mag";
		case OPERATION_MODE_ACCGYRO: return "Acc+Gyro";
		case OPERATION_MODE_MAGGYRO: return "Mag+Gyro";
		case OPERATION_MODE_AMG: return "AMG";
		case OPERATION_MODE_IMUPLUS: return "IMU";
		case OPERATION_MODE_COMPASS: return "Compass";
		case OPERATION_MODE_M4G: return "M4G";
		case OPERATION_MODE_NDOF_FMC_OFF: return "NDOF_FMC_OFF";
		case OPERATION_MODE_NDOF: return "NDOF";
		default: return "invalid mode";
		}
	}
	typedef enum
	{
	  REMAP_CONFIG_P0                                         = 0x21,
	  REMAP_CONFIG_P1                                         = 0x24, // default
	  REMAP_CONFIG_P2                                         = 0x24,
	  REMAP_CONFIG_P3                                         = 0x21,
	  REMAP_CONFIG_P4                                         = 0x24,
	  REMAP_CONFIG_P5                                         = 0x21,
	  REMAP_CONFIG_P6                                         = 0x21,
	  REMAP_CONFIG_P7                                         = 0x24
	} bno055_axis_remap_config_t;

	typedef enum
	{
	  REMAP_SIGN_P0                                           = 0x04,
	  REMAP_SIGN_P1                                           = 0x00, // default
	  REMAP_SIGN_P2                                           = 0x06,
	  REMAP_SIGN_P3                                           = 0x02,
	  REMAP_SIGN_P4                                           = 0x03,
	  REMAP_SIGN_P5                                           = 0x01,
	  REMAP_SIGN_P6                                           = 0x07,
	  REMAP_SIGN_P7                                           = 0x05
	} bno055_axis_remap_sign_t;

	typedef struct
	{
	  uint8_t  accel_rev;
	  uint8_t  mag_rev;
	  uint8_t  gyro_rev;
	  uint16_t sw_rev;
	  uint8_t  bl_rev;
	} bno055_rev_info_t;

	typedef enum
	{
	  SYSTEM_IDLE = 0x00,
	  SYSTEM_ERROR = 0X01,
	  INITIALIZING_PERIPHERALS = 0x02,
	  SYSTEM_INITIALIZING = 0x03,
	  EXECUTING_SELFTEST = 0x04,
	  SENSOR_FUSION_ALGORITHM_RUNNING = 0x05,
	  SYSTEM_RUNNING_WITHOUT_FUSION_ALGORITHM = 0x06
	} bno055_status_t;

	std::string BNO055_Status_String(bno055_status_t status)
	{
		switch(status)
		{
		case SYSTEM_IDLE : return "SYSTEM_IDLE";
		case SYSTEM_ERROR : return "SYSTEM_ERROR";
		case INITIALIZING_PERIPHERALS : return "INITIALIZING_PERIPHERALS";
		case SYSTEM_INITIALIZING : return "SYSTEM_INITIALIZING";
		case EXECUTING_SELFTEST : return "EXECUTING_SELFTEST";
		case SENSOR_FUSION_ALGORITHM_RUNNING : return "SENSOR_FUSION_ALGORITHM_RUNNING";
		case SYSTEM_RUNNING_WITHOUT_FUSION_ALGORITHM : return "SYSTEM_RUNNING_WITHOUT_FUSION_ALGORITHM";
		default : return "Unknown Status";
		}
	};
	typedef enum
	{
	  VECTOR_ACCELEROMETER = BNO055_ACCEL_DATA_X_LSB_ADDR,
	  VECTOR_MAGNETOMETER  = BNO055_MAG_DATA_X_LSB_ADDR,
	  VECTOR_GYROSCOPE     = BNO055_GYRO_DATA_X_LSB_ADDR,
	  VECTOR_EULER         = BNO055_EULER_H_LSB_ADDR,
	  VECTOR_LINEARACCEL   = BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR,
	  VECTOR_GRAVITY       = BNO055_GRAVITY_DATA_X_LSB_ADDR
	} vector_type_t;

	BNO055(I2C::Port port, int deviceAddress = bno055_i2c_adr::BNO055_ADR_A);
	~BNO055();

	typedef enum
	{
		X,
		Y,
		Z,
		W
	}QuaternionElement;

	typedef enum
	{
		INIT,
		UPDATE,
		CONFIG,
		REBOOT
	} IMUState;



	void Begin();
	void SetOperationMode(BNO055::bno055_opmode_t mode);

	bno055_status_t GetStatus();
	uint8_t GetError();
	double GetChipID();
	bno055_opmode_t GetMode();

	int Read(uint8_t reg, int Len, uint8_t *data);
	int Write(uint8_t reg, uint8_t data);

	void UpdateQuaternion();
	void SetCurrentQuaternionAsInit();
	double GetCurrentQuaternion(QuaternionElement element);
	double GetInitQuaternion(QuaternionElement element);

	void GetCalData();
	uint8_t GetCalStatus();
	uint8_t GetCalSys(){return (cal&0xC0)>>6;};
	uint8_t GetCalGryo(){return (cal&0x30)>>4;};
	uint8_t GetCalAcc(){return (cal&0x0C)>>2;};
	uint8_t GetCalMag(){return (cal&0x03);};

	double GetTemp();

	void SetInitEulers(Vector v){initEulerAngles=v;};
	Vector GetInitEulers(){return initEulerAngles;};

	Quaternion GetQuat(void);
	Vector GetVector(vector_type_t type);

	void Update();
	void DirectUpdate();

	//Multi tasking functions
	void Enable();
	void Disable();
	bool IsEnabled(){return m_enabled;};
	void SetPeriod(float p){m_period = p;};
	float GetPeriod(){return m_period;};

	void SendSMData(std::string name);

protected:
	void Run(); //main function for thread

private:
	I2C chip;
	uint8_t adr;
	Quaternion initVector;
	Quaternion currentVector;

	Vector initEulerAngles;

	Vector eulerAngles;
	Vector mag;
	Vector gyro;
	Vector acc;
	Vector gravity;

	std::unique_ptr<Notifier> 	m_task;
	float 		m_period;
	//MUTEX_ID 	m_sem;
	mutable wpi::mutex m_mutex;
	bool		m_enabled;

	static void CallUpdate(void *imu);

	IMUState m_state;

	uint8_t cal;
	static const int calDataSize=22;
	double calData[calDataSize];
	static const uint8_t initCalData[calDataSize];

};



#endif  //SRC_BNO055_HPP_


