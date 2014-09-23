/*
@file	PCA9685.h
Header file for PCA9685
16-channel, 12-bit PWM Fm+ I2C-bus LED controller
See PCA9685.pdf datasheet for details

@author	Thomas Oppenhoff

https://github.com/TOppenhoff/PCA9685

Language: C++
License: GNU Public License

*/

#ifndef __PCA9685_H
#define __PCA9685_H

//#define PCA9685_SERIAL_DEBUG

#include <stdio.h>
#include <Arduino.h>

#define PCA9685_BASEADR			0x40 // I2C base address

#define PCA9685_MODE1			0x00 // Mode register 1
#define PCA9685_MODE2			0x01 // Mode register 2
#define PCA9685_PRE_SCALE		0xFE // prescaler for output frequency
#define PCA9685_LED0_ON_L		0x06 // LED0 ON Low Byte

#define PCA9685_MODE_MOTOR_DIRECT	0x0
#define PCA9685_MODE_LED_DIRECT		0x1
#define PCA9685_MODE_N_DRIVER		0x2
#define PCA9685_MODE_P_DRIVER		0x3

#define PCA9685_NUM_PINS			16

#define PCA9685_MAX_VALUE				0xFFF
#define PCA9685_FULL_ON					0xFFF
#define PCA9685_FULL_OFF				0x0
#define PCA9685_DEFAULT_PRE_SCALE		0x1E
#define PCA9685_ADR_PINS				0x3F
#define PCA9685_CLOCK					25000000
#define PCA9685_MIN_FREQUENCY			23.0
#define PCA9685_MAX_FREQUENCY			1500.0

#define PCA9685_MODE1_SLEEP_BIT			0x10
#define PCA9685_LED_ON_FULL_ON_BIT		0x1000
#define PCA9685_LED_OFF_FULL_OFF_BIT	0x1000
#define PCA9685_MODE2_INVRT_BIT			0x10
#define PCA9685_MODE2_OUTDRV_BIT		0x04

#define PCA9685_LED0	0
#define PCA9685_LED1	1
#define PCA9685_LED2	2
#define PCA9685_LED3	3
#define PCA9685_LED4	4
#define PCA9685_LED5	5
#define PCA9685_LED6	6
#define PCA9685_LED7	7
#define PCA9685_LED8	8
#define PCA9685_LED9	9
#define PCA9685_LED10	10
#define PCA9685_LED11	11
#define PCA9685_LED12	12
#define PCA9685_LED13	13
#define PCA9685_LED14	14
#define PCA9685_LED15	15

class PCA9685
{
public:
	class Led
	{
		friend class PCA9685;
	private:
		PCA9685* _parent;
		uint16_t _value;
		uint8_t _idx;
	public:
		Led();

		void write() const;

		uint16_t getValue();
		void setValue(uint16_t value);
		void setValueAndWrite(uint16_t value);

		void fullOn();
		void fullOnAndWrite();
		void fullOff();
		void fullOffAndWrite();

	private:
		void attachParent(PCA9685* parent);
		void setIdx(uint8_t idx);
	};

private:
	uint8_t _i2cArd;
	uint8_t _preScale;
	uint8_t _driveMode;

	Led _leds[PCA9685_NUM_PINS];

public:
	PCA9685();
	PCA9685(uint8_t i2cSlaveAdr);
	PCA9685(uint8_t i2cSlaveAdr, uint8_t driveMode);
	PCA9685(uint8_t i2cSlaveAdr, uint8_t driveMode, float frequency);

	Led& getLed(uint8_t pin);

	float getFrequency() const;
	void setFrequency(float frequency);
	uint8_t getPreScale() const;
	void setPreScale(uint8_t preScale);
	uint8_t getDriveMode() const;
	void setDriveMode(uint8_t driveMode);

	void setup() const;
	void writeAllLeds() const;

protected:

	void init(uint8_t i2cSlaveAdr, uint8_t driveMode, uint8_t prescale);

	void sleep() const;
	void wakeup() const;

	uint8_t calculatePrescale(float frequency) const;

	void writeLed(const Led& led) const;

	uint8_t read8(uint8_t reg) const;
	void write8(uint8_t reg, uint8_t value) const;
	void read32(uint8_t reg, uint16_t& on, uint16_t& off) const;
	void write32(uint8_t reg, uint16_t on, uint16_t off) const;

};



#endif // #ifndef __PCA9685_H