

#include "stm32f4xx.h"

#include "CK_SYSTEM.h"
#include "CK_TIME_HAL.h"
#include "CK_SPI.h"
#include "CK_GPIO.h"
#include "math.h"
#include "stdbool.h"

#include "USBD_CDC/CK_USBD_INTERFACE.h"

#define M_PIf        				 3.14159265358979323846f
#define HIGH_GAIN_TIME_DURATION     	1000000 // 1 sec
#define USB_DEBUG

//#define 	GYRO_DEBUG
//#define 	ACC_DEBUG
//#define 	MAG_DEBUG
#define 	IMU_DEBUG

#define CK_ICM20602_SMPLRT_DIV_REG          0x19
#define CK_ICM20602_CONFIG_REG              0x1A
#define CK_ICM20602_GYRO_CONFIG_REG         0x1B
#define CK_ICM20602_ACCEL_CONFIG_REG        0x1C
#define CK_ICM20602_ACCEL_CONFIG2_REG       0x1D
#define CK_ICM20602_SIGNAL_PATH_RESET_REG   0x68
#define CK_ICM20602_USER_CONTROL_REG        0x6A
#define CK_ICM20602_PWR_MNG1_REG            0x6B
#define CK_ICM20602_I2C_IF_REG              0x70

#define ICM20602_WHO_AM_I_ID                0x12

#define ICM20602_RESET_BIT                  1u<<7
#define ICM20602_I2C_IF_DIS_BIT             1u<<6

#define CK_ICM20602_WHO_AM_I_READ_REG       0x75|0x80 // If we set MSB, indicates a read operation
#define CK_ICM20602_GYRO_XOUT_H             0x43|0x80
#define CK_ICM20602_GYRO_XOUT_L             0x44|0x80
#define CK_ICM20602_GYRO_YOUT_H             0x45|0x80
#define CK_ICM20602_GYRO_YOUT_L             0x46|0x80
#define CK_ICM20602_GYRO_ZOUT_H             0x47|0x80
#define CK_ICM20602_GYRO_ZOUT_L             0x48|0x80

#define CK_ICM20602_ACCEL_XOUT_H            0x3B|0x80
#define CK_ICM20602_ACCEL_XOUT_L            0x3C|0x80
#define CK_ICM20602_ACCEL_YOUT_H            0x3D|0x80
#define CK_ICM20602_ACCEL_YOUT_L            0x3E|0x80
#define CK_ICM20602_ACCEL_ZOUT_H            0x3F|0x80
#define CK_ICM20602_ACCEL_ZOUT_L            0x40|0x80

SPI_TypeDef * SPI_ICM20602 = SPI1;
GPIO_TypeDef* GPIO_CS_ICM20602 = GPIOB;
uint8_t CS_PIN_ICM20602 = 12;

int16_t gyroRaw[3];
int32_t gyroCalib[3];
float gyroFinal[3];
float gyroFiltered[3];
float gyroSensitivity;

int16_t accRaw[3];
int32_t accCalib[3];
float accFinal[3];
float accFiltered[3];
float accSensitivity;

int16_t magRaw[3];
float magGauss[3];
float magMicroTesla[3];
float magFinal[3];
float magFiltered[3];
float mag_heading;

uint8_t i2cdata[20];

void initSensor();

void readGyro(void);
void updateGyro(void);

void readAcc(void);
void updateAcc(void);

void CK_IMU_MahonyUpdate_2Axis(void);

float CK_MATH_invSqrt(float x);
void CK_IMU_ComputeEulerAngles();
void printResults(float roll, float pitch, float heading);
void calibrateAccGyro(void);

typedef struct{

    float dcm_kp;
    float dcm_ki;
    float dcm_beta;

    uint32_t gain_timer;
    bool imu_converged;

}imuConfig_t;

typedef struct{

    float       eulerAngles[3];
    float       eulerAnglesCentiDegrees[3]; // 100 * degrees
    float       eulerAnglesDeciDegrees[3]; // 10 * degrees
    float       eulerAnglesFiltered[3];

}imuAngles_t;

typedef struct vectorXYZ_s{

	float x;
	float y;
	float z;

}vectorXYZ_t;


typedef struct{

    float q0;
    float q1;
    float q2;
    float q3;

    float integralX;
    float integralY;
    float integralZ;

    float imu_dT;

    bool is_imu_init;

}imuParameters_t;

imuConfig_t gain_parameters = {
	.dcm_kp = 250.0f,
	.dcm_ki = 0.0f,
	.dcm_beta = 1.0f,

    .gain_timer = 0,
    .imu_converged = false
};

imuParameters_t imu_parameters = {
    .q0 = 1.0f,
    .q1 = 0.0f,
    .q2 = 0.0f,
    .q3 = 0.0f,

    .integralX = 0.0f,
    .integralY = 0.0f,
    .integralZ = 0.0f,

    .imu_dT = 2000 * 0.000001f,

	.is_imu_init = false
};

imuAngles_t imu;

uint32_t loopTimer;
uint32_t loopTime = 250; // 4KHz
uint32_t imuTime = 2000; // 500 Hz
int imuSyncCounter = 0;

uint32_t imu_idle_counter = 0;

int main(void){

	CK_SYSTEM_SetSystemClock(SYSTEM_CLK_168MHz);

	HAL_Init();

	CK_USBD_Init();

	CK_SPI_Init(SPI_ICM20602, CK_SPIx_CR1_Fclk_Div8);

	initSensor();

	loopTimer = CK_TIME_GetMicroSec();

	while(1){

		updateGyro();
		updateAcc();

		imuSyncCounter++;

		if(!gain_parameters.imu_converged){
			imu_idle_counter += loopTime;
		}

		if((imu_idle_counter >= HIGH_GAIN_TIME_DURATION) && !gain_parameters.imu_converged){
			gain_parameters.dcm_kp = 10.0f;
			gain_parameters.dcm_ki = 0.0001f;
			gain_parameters.imu_converged = true;

		}

		if(imuSyncCounter == 8){

			imuSyncCounter = 0;

			CK_IMU_MahonyUpdate_2Axis();

			printResults(imu.eulerAngles[0], imu.eulerAngles[1], (360.0f - imu.eulerAngles[2]));
		}

		while((CK_TIME_GetMicroSec() - loopTimer) < loopTime);
		loopTimer = CK_TIME_GetMicroSec();

	}
}

void initSensor(){

	//ICM20602 CS Pin
	CK_GPIO_ClockEnable(GPIO_CS_ICM20602);

	CK_GPIO_Init(GPIO_CS_ICM20602, CS_PIN_ICM20602, CK_GPIO_OUTPUT_PP, CK_GPIO_NOAF, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);

	CK_GPIO_SetPin(GPIO_CS_ICM20602, CS_PIN_ICM20602); //Set CS High for Idle

	int init_counter = 0;
	bool is_init_done = false;

	while(init_counter < 10 && !is_init_done){

		init_counter++;

		if(CK_SPI_WriteRegister(0x75|0x80, 0xFF, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602) == 0x12){

			// ICM20602 Setup
			CK_TIME_DelayMilliSec(100);

			// Device Reset, Takes everyting to default
			CK_SPI_WriteRegister(CK_ICM20602_PWR_MNG1_REG, ICM20602_RESET_BIT, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
			CK_TIME_DelayMilliSec(100);

			// I2C Disable, SPI Only Mode
			CK_SPI_WriteRegister(CK_ICM20602_I2C_IF_REG, ICM20602_I2C_IF_DIS_BIT, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
			CK_TIME_DelayMilliSec(10);

			// Reset Signal Path and Sensor Register
			CK_SPI_WriteRegister(CK_ICM20602_USER_CONTROL_REG, 0x05, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
			CK_TIME_DelayMilliSec(100);

			// Clear PWR MNG1 Register
			//CK_SPI_WriteRegister(CK_ICM20602_PWR_MNG1_REG, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
			//CK_TIME_DelayMilliSec(10);

			// CLK Selection for Best Gyro Performance
			CK_SPI_WriteRegister(CK_ICM20602_PWR_MNG1_REG, 0x01, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
			CK_TIME_DelayMilliSec(10);

			// SMPL RATE DIV = 0 8Khz/1+SMPL RATE DIV = 8Khz
			CK_SPI_WriteRegister(CK_ICM20602_SMPLRT_DIV_REG, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
			CK_TIME_DelayMilliSec(10);

			// DLPF_CFG[00] 8KHz, Bit 7 set to 0
			CK_SPI_WriteRegister(CK_ICM20602_CONFIG_REG, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
			CK_TIME_DelayMilliSec(10);

			// FCHOICE_B [00], FSEL[11] CK_ICM20602_DPS2000
			CK_SPI_WriteRegister(CK_ICM20602_GYRO_CONFIG_REG, 0x18, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
			CK_TIME_DelayMilliSec(10);



			// ACCEL,TEMP SIGNAL PATH RESET
			CK_SPI_WriteRegister(CK_ICM20602_SIGNAL_PATH_RESET_REG, 0x03, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
			CK_TIME_DelayMilliSec(10);

			// ACCEL_FS_SEL[11] +-16g 0x18, 2g 0x00
			CK_SPI_WriteRegister(CK_ICM20602_ACCEL_CONFIG_REG, 0x18, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);
			CK_TIME_DelayMilliSec(10);

			// ACCEL_FCHOICE_B=0,A_DLPF_CFG=0 1KHz ACCEL
			CK_SPI_WriteRegister(CK_ICM20602_ACCEL_CONFIG2_REG, 0x05, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);

			// ACCEL_FCHOICE_B=1,A_DLPF_CFG=0 4KHz ACCEL
			//CK_SPI_WriteRegister(CK_ICM20602_ACCEL_CONFIG2_REG, 0x08, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);

			CK_TIME_DelayMilliSec(10);

			is_init_done = true;

			gyroSensitivity = 1.0f/16.4f;

			CK_USBD_StringPrintln("Init Correct");
			CK_USBD_Transmit();

			calibrateAccGyro();
		}
		else{
			CK_USBD_StringPrintln("Init Error");
			CK_USBD_Transmit();

			CK_TIME_DelayMilliSec(1000);
		}
	}
}

void readGyro(void){

	gyroRaw[0] = (int16_t)(CK_SPI_WriteRegister(0x43|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602) <<8 | CK_SPI_WriteRegister(0x44|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602));

	gyroRaw[1] = (int16_t)(CK_SPI_WriteRegister(0x45|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602) <<8 | CK_SPI_WriteRegister(0x46|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602));

	gyroRaw[2] = (int16_t)(CK_SPI_WriteRegister(0x47|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602) <<8 | CK_SPI_WriteRegister(0x48|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602));

}

void updateGyro(void){

	readGyro();

	gyroRaw[0] -= gyroCalib[0];
	gyroRaw[1] -= gyroCalib[1];
	gyroRaw[2] -= gyroCalib[2];

	gyroFinal[0] = (float)gyroRaw[0] * gyroSensitivity;
	gyroFinal[1] = (float)gyroRaw[1] * gyroSensitivity;
	gyroFinal[2] = (float)gyroRaw[2] * gyroSensitivity;

	gyroFiltered[0] = gyroFiltered[0] * 0.8 + gyroFinal[0] * 0.2;
	gyroFiltered[1] = gyroFiltered[1] * 0.8 + gyroFinal[1] * 0.2;
	gyroFiltered[2] = gyroFiltered[2] * 0.8 + gyroFinal[2] * 0.2;

}

void readAcc(void){

	accRaw[0] = (int16_t)(CK_SPI_WriteRegister(0x3B|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602) <<8 | CK_SPI_WriteRegister(0x3C|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602));

	accRaw[1] = (int16_t)(CK_SPI_WriteRegister(0x3D|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602) <<8 | CK_SPI_WriteRegister(0x3E|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602));

	accRaw[2] = (int16_t)(CK_SPI_WriteRegister(0x3F|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602) <<8 | CK_SPI_WriteRegister(0x40|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602));

}

void updateAcc(void){

	readAcc();

	accRaw[0] -= accCalib[0];
	accRaw[1] -= accCalib[1];
	//accRaw[2] -= accCalib[2];

	// g value
	accFinal[0] = (float)accRaw[0];
	accFinal[1] = (float)accRaw[1];
	accFinal[2] = (float)accRaw[2];

	accFiltered[0] = accFiltered[0] * 0.8 + accFinal[0] * 0.2;
	accFiltered[1] = accFiltered[1] * 0.8 + accFinal[1] * 0.2;
	accFiltered[2] = accFiltered[2] * 0.0 + accFinal[2] * 1.0;

}

void CK_IMU_MahonyUpdate_2Axis(void){

	#if defined(DEBUG_TIMING)
	imu_debug.start_time = CK_TIME_GetMicroSec();
	#endif

	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	float gx = -gyroFiltered[0];
	float gy = -gyroFiltered[1];
	float gz = -gyroFiltered[2];

	float ax = accFiltered[0];
	float ay = accFiltered[1];
	float az = accFiltered[2];

	// convert to g
	ax /= 2048.0f;
	ay /= 2048.0f;
	az /= 2048.0f;

	// Convert gyroscope degrees/sec to radians/sec
	gx *= 0.0174533f;
	gy *= 0.0174533f;
	gz *= 0.0174533f;

	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = CK_MATH_invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Estimated direction of gravity
		halfvx = imu_parameters.q1 * imu_parameters.q3 - imu_parameters.q0 * imu_parameters.q2;
		halfvy = imu_parameters.q0 * imu_parameters.q1 + imu_parameters.q2 * imu_parameters.q3;
		halfvz = imu_parameters.q0 * imu_parameters.q0 - 0.5f + imu_parameters.q3 * imu_parameters.q3;

		// Error is sum of cross product between estimated
		// and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(gain_parameters.dcm_ki > 0.0f) {
			// integral error scaled by Ki
			imu_parameters.integralX += gain_parameters.dcm_ki * halfex * imu_parameters.imu_dT;
			imu_parameters.integralY += gain_parameters.dcm_ki * halfey * imu_parameters.imu_dT;
			imu_parameters.integralZ += gain_parameters.dcm_ki * halfez * imu_parameters.imu_dT;
			gx += imu_parameters.integralX;	// apply integral feedback
			gy += imu_parameters.integralY;
			gz += imu_parameters.integralZ;
		}
		else {
			imu_parameters.integralX = 0.0f;	// prevent integral windup
			imu_parameters.integralY = 0.0f;
			imu_parameters.integralZ = 0.0f;
		}

		// Apply proportional feedback
		gx += gain_parameters.dcm_kp * halfex;
		gy += gain_parameters.dcm_kp * halfey;
		gz += gain_parameters.dcm_kp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * imu_parameters.imu_dT);		// pre-multiply common factors
	gy *= (0.5f * imu_parameters.imu_dT);
	gz *= (0.5f * imu_parameters.imu_dT);
	qa = imu_parameters.q0;
	qb = imu_parameters.q1;
	qc = imu_parameters.q2;
	imu_parameters.q0 += (-qb * gx - qc * gy - imu_parameters.q3 * gz);
	imu_parameters.q1 += (qa * gx + qc * gz - imu_parameters.q3 * gy);
	imu_parameters.q2 += (qa * gy - qb * gz + imu_parameters.q3 * gx);
	imu_parameters.q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = CK_MATH_invSqrt( imu_parameters.q0 * imu_parameters.q0
							   + imu_parameters.q1 * imu_parameters.q1
							   + imu_parameters.q2 * imu_parameters.q2
							   + imu_parameters.q3 * imu_parameters.q3);
	imu_parameters.q0 *= recipNorm;
	imu_parameters.q1 *= recipNorm;
	imu_parameters.q2 *= recipNorm;
	imu_parameters.q3 *= recipNorm;

	CK_IMU_ComputeEulerAngles();


}

void CK_IMU_ComputeEulerAngles(void){

	// Compute Roll, Pitch Yaw Angles
	imu.eulerAngles[0]  = atan2f(imu_parameters.q0 * imu_parameters.q1 + imu_parameters.q2 * imu_parameters.q3, 0.5f - imu_parameters.q1 * imu_parameters.q1 - imu_parameters.q2 * imu_parameters.q2);
	imu.eulerAngles[1] = asinf(-2.0f * (imu_parameters.q1 * imu_parameters.q3 - imu_parameters.q0 * imu_parameters.q2));
	imu.eulerAngles[2]   = atan2f(imu_parameters.q1 * imu_parameters.q2 + imu_parameters.q0 * imu_parameters.q3, 0.5f - imu_parameters.q2 * imu_parameters.q2 - imu_parameters.q3 * imu_parameters.q3);

	imu.eulerAngles[0]  *= 57.29578f;
	imu.eulerAngles[1] *= 57.29578f;
	imu.eulerAngles[2]   *= 57.29578f;
	imu.eulerAngles[2]   +=  180.0f;

	imu.eulerAngles[2] = 360.0f - imu.eulerAngles[2];

	// Due to the orientation in gyro's way take negatives for roll and pitch
	// In this way i get nose down at +pitch and right down at +roll
	imu.eulerAngles[0]  *= -1.0f;
	imu.eulerAngles[1] *= -1.0f;

	imu.eulerAnglesDeciDegrees[0] 	= imu.eulerAngles[0] * 10.0f;
	imu.eulerAnglesDeciDegrees[1] 	= imu.eulerAngles[1] * 10.0f;
	imu.eulerAnglesDeciDegrees[2] 	= imu.eulerAngles[2] * 10.0f;

	imu.eulerAnglesCentiDegrees[0] 	= imu.eulerAngles[0] * 100.0f;
	imu.eulerAnglesCentiDegrees[1]  = imu.eulerAngles[1] * 100.0f;
	imu.eulerAnglesCentiDegrees[2] 	= imu.eulerAngles[2] * 100.0f;

}

float CK_MATH_invSqrt(float x){

	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void printResults(float roll, float pitch, float heading){

#ifdef USB_DEBUG

#ifdef GYRO_DEBUG
	CK_USBD_IntPrint(gyroRaw[0]);CK_USBD_StringPrint("\t");
	CK_USBD_IntPrint(gyroRaw[1]);CK_USBD_StringPrint("\t");
	CK_USBD_IntPrint(gyroRaw[2]);CK_USBD_StringPrint("\t");

	CK_USBD_FloatPrint(gyroFinal[0]);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrint(gyroFinal[1]);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrint(gyroFinal[2]);CK_USBD_StringPrint("\t");

	CK_USBD_FloatPrint(gyroFiltered[0]);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrint(gyroFiltered[1]);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrintln(gyroFiltered[2]);

	CK_USBD_Transmit();

#endif

#ifdef ACC_DEBUG
	CK_USBD_IntPrint(accRaw[0]);CK_USBD_StringPrint("\t");
	CK_USBD_IntPrint(accRaw[1]);CK_USBD_StringPrint("\t");
	CK_USBD_IntPrint(accRaw[2]);CK_USBD_StringPrint("\t");

	CK_USBD_FloatPrint(accFinal[0]);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrint(accFinal[1]);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrint(accFinal[2]);CK_USBD_StringPrint("\t");

	CK_USBD_FloatPrint(accFiltered[0]);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrint(accFiltered[1]);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrintln(accFiltered[2]);

	CK_USBD_Transmit();
#endif

#ifdef IMU_DEBUG
	CK_USBD_FloatPrint(roll);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrint(pitch);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrintln(heading);

	CK_USBD_Transmit();
#endif

#endif

}

void calibrateAccGyro(void){

	int numOfSamples = 10000;
	gyroCalib[0] = 0;
	gyroCalib[1] = 0;
	gyroCalib[2] = 0;

	CK_USBD_StringPrint("Gyro Calibration");
	CK_USBD_Transmit();

	for(int i = 0; i < numOfSamples; i++){
		readGyro();
		gyroCalib[0] += gyroRaw[0];
		gyroCalib[1] += gyroRaw[1];
		gyroCalib[2] += gyroRaw[2];

		CK_TIME_DelayMicroSec(loopTime);
		if(i%10 == 1){
			CK_USBD_StringPrint(".");
			CK_USBD_Transmit();
		}
	}

	gyroCalib[0] /= numOfSamples;
	gyroCalib[1] /= numOfSamples;
	gyroCalib[2] /= numOfSamples;

	CK_USBD_StringPrintln("");
	CK_USBD_Transmit();

	accCalib[0] = 0;
	accCalib[1] = 0;
	accCalib[2] = 0;

	CK_USBD_StringPrint("Acc Calibration");
	CK_USBD_Transmit();

	for(int i = 0; i < numOfSamples; i++){
		readAcc();
		accCalib[0] += accRaw[0];
		accCalib[1] += accRaw[1];
		accCalib[2] += accRaw[2];

		CK_TIME_DelayMicroSec(loopTime);
		if(i%10 == 1){
			CK_USBD_StringPrint(".");
			CK_USBD_Transmit();
		}
	}

	accCalib[0] /= numOfSamples;
	accCalib[1] /= numOfSamples;
	accCalib[2] /= numOfSamples;

	CK_USBD_StringPrintln("");
	CK_USBD_Transmit();

	CK_USBD_IntPrint(gyroCalib[0]);CK_USBD_StringPrint(",");
	CK_USBD_IntPrint(gyroCalib[1]);CK_USBD_StringPrint(",");
	CK_USBD_IntPrint(gyroCalib[2]);
	CK_USBD_StringPrint("    ");
	CK_USBD_IntPrint(accCalib[0]);CK_USBD_StringPrint(",");
	CK_USBD_IntPrint(accCalib[1]);CK_USBD_StringPrint(",");
	CK_USBD_IntPrintln(accCalib[2]);
	CK_USBD_Transmit();

}


