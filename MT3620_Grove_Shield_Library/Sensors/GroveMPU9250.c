#include "stdlib.h"
#include "GroveMPU9250.h"
#include "../HAL/GroveI2C.h"

#define CMD_SOFT_RESET		(0x30a2)
#define CMD_SINGLE_HIGH		(0x2400)

typedef struct
{
	int I2cFd;
	uint8_t deviceAddress;
} GroveMPU9250Instance;

void* GroveMPU9250_Open(int i2cFd)
{
	GroveMPU9250Instance* this = (GroveMPU9250Instance*)malloc(sizeof(GroveMPU9250Instance));

	this->I2cFd = i2cFd;
	this->deviceAddress = MPU9250_DEFAULT_ADDRESS;

	// Initialisation of the sensor as per the Arduino library
	GroveMPU9250_setClockSource(this, MPU9250_CLOCK_PLL_XGYRO);
	GroveMPU9250_setFullScaleGyroRange(this, MPU9250_GYRO_FS_250);
	GroveMPU9250_setFullScaleAccelRange(this, MPU9250_ACCEL_FS_2);
	GroveMPU9250_setSleepEnabled(this, false);

	return this;
}

void GroveMPU9250_getMotion6(void* inst, int16_t * ax, int16_t * ay, int16_t * az, int16_t * gx, int16_t * gy, int16_t * gz)
{
	// The original Arduino implementation seems to pull 14 bytes out of the registers at once, skipping the ones for temperature
	// measurements. This is probably quicker than reading the registers one-by-one, but requires an addition to the GroveI2C lib.

	// Get accel data
	GroveMPU9250_getAcceleration(inst, ax, ay, az);

	// Get gyro data
	GroveMPU9250_getRotation(inst, gx, gy, gz);
}

void GroveMPU9250_getAcceleration(void* inst, int16_t * x, int16_t * y, int16_t * z)
{
	GroveMPU9250_getSingleMeasurement(inst, MPU9250_RA_ACCEL_XOUT_H, x);
	GroveMPU9250_getSingleMeasurement(inst, MPU9250_RA_ACCEL_YOUT_H, y);
	GroveMPU9250_getSingleMeasurement(inst, MPU9250_RA_ACCEL_ZOUT_H, z);
}

void GroveMPU9250_getSingleMeasurement(void* inst, uint8_t reg, int16_t *measurement)
{
	GroveMPU9250Instance* this = (GroveMPU9250Instance*)inst;
	// Using uint8_t buffer of double length instead of single uint16_t buffer
	// so we can get two registry values at once (*_H and *_L) and combine them
	// to a signed measurement
	uint8_t buffer[2];

	GroveI2C_ReadReg16(this->I2cFd, this->deviceAddress, reg, &buffer);
	*measurement = (((int16_t)buffer[0]) << 8) | buffer[1];
}

void GroveMPU9250_setFullScaleAccelRange(void* inst, uint8_t range)
{
	GroveMPU9250Instance* this = (GroveMPU9250Instance*)inst;

	GroveI2C_WriteBits(this->I2cFd, this->deviceAddress, MPU9250_RA_ACCEL_CONFIG, MPU9250_ACONFIG_AFS_SEL_BIT, &range, MPU9250_ACONFIG_AFS_SEL_LENGTH);
}

void GroveMPU9250_getRotation(void* inst, int16_t * x, int16_t * y, int16_t * z)
{
	GroveMPU9250_getSingleMeasurement(inst, MPU9250_RA_GYRO_XOUT_H, x);
	GroveMPU9250_getSingleMeasurement(inst, MPU9250_RA_GYRO_YOUT_H, y);
	GroveMPU9250_getSingleMeasurement(inst, MPU9250_RA_GYRO_ZOUT_H, z);
}

void GroveMPU9250_setFullScaleGyroRange(void* inst, uint8_t range)
{
	GroveMPU9250Instance* this = (GroveMPU9250Instance*)inst;

	GroveI2C_WriteBits(this->I2cFd, this->deviceAddress, MPU9250_RA_GYRO_CONFIG, MPU9250_GCONFIG_FS_SEL_BIT, &range, MPU9250_GCONFIG_FS_SEL_LENGTH);
}

void GroveMPU9250_setClockSource(void* inst, uint8_t source)
{
	GroveMPU9250Instance* this = (GroveMPU9250Instance*)inst;

	GroveI2C_WriteBits(this->I2cFd, this->deviceAddress, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_CLKSEL_BIT, &source, MPU9250_PWR1_CLKSEL_LENGTH);
}

uint8_t GroveMPU9250_getClockSource(void* inst)
{
	GroveMPU9250Instance* this = (GroveMPU9250Instance*)inst;

	uint8_t buffer = 0;
	GroveI2C_ReadReg8(this->I2cFd, this->deviceAddress, MPU9250_RA_PWR_MGMT_1, &buffer); // Get entire byte

	uint8_t mask = ((1 << (uint8_t)MPU9250_PWR1_CLKSEL_LENGTH) - 1) << ((uint8_t)MPU9250_PWR1_CLKSEL_BIT - (uint8_t)MPU9250_PWR1_CLKSEL_LENGTH + 1);
	buffer &= mask; // Only leave the relevant bits

	return buffer;
}

void GroveMPU9250_setSleepEnabled(void* inst, bool enabled)
{
	GroveMPU9250Instance* this = (GroveMPU9250Instance*)inst;

	uint8_t enabledBit = (uint8_t)enabled;

	GroveI2C_WriteBits(this->I2cFd, this->deviceAddress, MPU9250_RA_PWR_MGMT_1, MPU9250_PWR1_SLEEP_BIT, &enabledBit, 1);
}

bool GroveMPU9250_testConnection(void * inst)
{
	return GroveMPU9250_getDeviceID(inst) == 0x71;
}

uint8_t GroveMPU9250_getDeviceID(void * inst)
{
	GroveMPU9250Instance* this = (GroveMPU9250Instance*)inst;
	uint8_t result;

	GroveI2C_ReadReg8(this->I2cFd, this->deviceAddress, MPU9250_RA_WHO_AM_I, &result);

	return result;
}

double GroveMPU9250_convertAccelDataToG(int16_t datum)
{
	return (double) datum / 16384; //16384  LSB/g
}

double GroveMPU9250_convertGyroDataToDpS(int16_t datum)
{
	return (double) datum * 250 / 32768; //131 LSB(deg/s)
}
