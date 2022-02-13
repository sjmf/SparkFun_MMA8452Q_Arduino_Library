/******************************************************************************
SparkFun_MMA8452Q.h
SparkFun_MMA8452Q Library Header File
Jim Lindblom and Andrea DeVore @ SparkFun Electronics
Original Creation Date: June 3, 2014
https://github.com/sparkfun/MMA8452_Accelerometer

This file prototypes the MMA8452Q class, implemented in SFE_MMA8452Q.cpp. In
addition, it defines every register in the MMA8452Q.

Development environment specifics:
	IDE: Arduino 1.0.5
	Hardware Platform: Arduino Uno

	**Updated for Arduino 1.8.5 2/2019**

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef SparkFun_MMA8452Q_h
#define SparkFun_MMA8452Q_h

#include <Arduino.h>
#include <Wire.h>

///////////////////////////////////
// MMA8452Q Register Definitions //
///////////////////////////////////
enum MMA8452Q_Register
{
	STATUS_MMA8452Q = 0x00,
	OUT_X_MSB = 0x01,
	OUT_X_LSB = 0x02,
	OUT_Y_MSB = 0x03,
	OUT_Y_LSB = 0x04,
	OUT_Z_MSB = 0x05,
	OUT_Z_LSB = 0x06,
	SYSMOD = 0x0B,
	INT_SOURCE = 0x0C,
	WHO_AM_I = 0x0D,
	XYZ_DATA_CFG = 0x0E,
	HP_FILTER_CUTOFF = 0x0F,
	PL_STATUS = 0x10,
	PL_CFG = 0x11,
	PL_COUNT = 0x12,
	PL_BF_ZCOMP = 0x13,
	P_L_THS_REG = 0x14,
	FF_MT_CFG = 0x15,
	FF_MT_SRC = 0x16,
	FF_MT_THS = 0x17,
	FF_MT_COUNT = 0x18,
	TRANSIENT_CFG = 0x1D,
	TRANSIENT_SRC = 0x1E,
	TRANSIENT_THS = 0x1F,
	TRANSIENT_COUNT = 0x20,
	PULSE_CFG = 0x21,
	PULSE_SRC = 0x22,
	PULSE_THSX = 0x23,
	PULSE_THSY = 0x24,
	PULSE_THSZ = 0x25,
	PULSE_TMLT = 0x26,
	PULSE_LTCY = 0x27,
	PULSE_WIND = 0x28,
	ASLP_COUNT = 0x29,
	CTRL_REG1 = 0x2A,
	CTRL_REG2 = 0x2B,
	CTRL_REG3 = 0x2C,
	CTRL_REG4 = 0x2D,
	CTRL_REG5 = 0x2E,
	OFF_X = 0x2F,
	OFF_Y = 0x30,
	OFF_Z = 0x31
};

////////////////////////////////
// MMA8452Q Misc Declarations //
////////////////////////////////
enum MMA8452Q_Scale
{
	SCALE_2G = 2,
	SCALE_4G = 4,
	SCALE_8G = 8
}; // Possible full-scale settings

enum MMA8452Q_ODR
{
	ODR_800,
	ODR_400,
	ODR_200,
	ODR_100,
	ODR_50,
	ODR_12,
	ODR_6,
	ODR_1
}; // possible data rates

// Port from https://github.com/akupila/Arduino-MMA8452
enum MMA8452Q_InterruptTypes {
	INT_AUTOSLEEP = 0x80,
	INT_TRANSIENT = 0x20,
	INT_LANDSCAPEPORTRAIT = 0x10,
	INT_PULSE_TAP = 0x08,
	INT_FREEFALL_MOTION = 0x04,
	INT_DATA_READY = 0x01
}; // Possible Interrupt Types for CTRL_REG4 and CTRL_REG5

enum MMA8452Q_FreefallMotionReg {
	FF_MT_CFG_ELE = 0x80,
	FF_MT_CFG_OAE = 0x40,
	FF_MT_CFG_ZEFE = 0x20,
	FF_MT_CFG_YEFE = 0x10,
	FF_MT_CFG_XEFE = 0x08,
	FF_MT_CFG_ALL_AXIS = 0x38,
}; // Freefall/Motion configuration register FF_MT_CFG values

enum MMA_FreefallSourceRegister {
	FF_MT_SRC_EA = 0x80,
	FF_MT_SRC_ZHE = 0x20,
	FF_MT_SRC_ZHP = 0x10,
	FF_MT_SRC_YHE = 0x08,
	FF_MT_SRC_YHP = 0x04,
	FF_MT_SRC_XHE = 0x02,
	FF_MT_SRC_XHP = 0x01
}; // Frefall source register bits (page 27 datasheet)

// sleep sampling mode
enum MMA_SleepFrequency {
	MMA_SLEEP_50hz = 0,
	MMA_SLEEP_12_5hz,
	MMA_SLEEP_6_25hz,
	MMA_SLEEP_1_56hz
};

// power mode
enum MMA_PowerMode {
	MMA_NORMAL = 0,
	MMA_LOW_NOISE_LOW_POWER,
	MMA_HIGH_RESOLUTION,
	MMA_LOW_POWER
};

// Possible portrait/landscape settings
#define PORTRAIT_U 0
#define PORTRAIT_D 1
#define LANDSCAPE_R 2
#define LANDSCAPE_L 3
#define LOCKOUT 0x40
#define MMA8452Q_DEFAULT_ADDRESS 0x1D

// Posible SYSMOD (system mode) States
#define SYSMOD_STANDBY 0b00
#define SYSMOD_WAKE 0b01
#define SYSMOD_SLEEP 0b10

////////////////////////////////
// MMA8452Q Class Declaration //
////////////////////////////////
class MMA8452Q
{
  public:
	MMA8452Q(byte addr = MMA8452Q_DEFAULT_ADDRESS); // Constructor
	MMA8452Q_Scale scale;
	MMA8452Q_ODR odr;

	bool begin(TwoWire &wirePort = Wire, uint8_t deviceAddress = MMA8452Q_DEFAULT_ADDRESS);
	byte init(MMA8452Q_Scale fsr = SCALE_2G, MMA8452Q_ODR odr = ODR_800);
	void read();
	byte available();
	byte readTap();
	byte readPL();
	byte readID();

	short x, y, z;
	float cx, cy, cz;

	short getX();
	short getY();
	short getZ();

	float getCalculatedX();
	float getCalculatedY();
	float getCalculatedZ();

	bool isRight();
	bool isLeft();
	bool isUp();
	bool isDown();
	bool isFlat();

	void setScale(MMA8452Q_Scale fsr);
	void setDataRate(MMA8452Q_ODR odr);

	// SF port from https://github.com/akupila/Arduino-MMA8452/ the following functions:
	void setInterruptsEnabled(uint8_t interruptMask);
	void configureInterrupts(bool activeHigh, bool openDrain);
	// true: pin1, false: pin2
	void setInterruptPins(bool autoSleepWake, bool transient, bool landscapePortraitChange, bool tap, bool freefall_motion, bool dataReady);
	void setupMotionDetection(uint8_t motionThreshold, uint8_t debounceCount);
	uint8_t readMotionSourceRegister();
	void setAutoSleep(bool enabled, uint8_t time, MMA_SleepFrequency sleepFrequencySampling, MMA_PowerMode sleepPowerMode);
	void setWakeOnInterrupt(bool transient, bool landscapePortraitChange, bool tap, bool freefall_motion);

  private:
	TwoWire *_i2cPort = NULL; //The generic connection to user's chosen I2C hardware
	uint8_t _deviceAddress;   //Keeps track of I2C address. setI2CAddress changes this.

	void standby();
	void active();
	bool isActive();
	void setupPL();
	void setupTap(byte xThs, byte yThs, byte zThs);
	void writeRegister(MMA8452Q_Register reg, byte data);
	void writeRegisters(MMA8452Q_Register reg, byte *buffer, byte len);
	byte readRegister(MMA8452Q_Register reg);
	void readRegisters(MMA8452Q_Register reg, byte *buffer, byte len);
};

#endif
