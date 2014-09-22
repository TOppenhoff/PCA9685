#include <PCA9685.h>
#include <Wire.h>

PCA9685::Led::Led()
{
	_parent = NULL;
	_idx = -1;
	_value = PCA9685_FULL_OFF;
}

void PCA9685::Led::read()
{
	_parent->readLed(*this);
}

void PCA9685::Led::write() const
{
	_parent->writeLed(*this);
}

uint16_t PCA9685::Led::getValue()
{
	return _value;
}

void PCA9685::Led::setValue(uint16_t value)
{
	if (value > PCA9685_MAX_VALUE)
		value = PCA9685_MAX_VALUE;

	_value = value;
}

void PCA9685::Led::fullOn()
{
	_value = PCA9685_FULL_ON;
}

void PCA9685::Led::fullOff()
{
	_value = PCA9685_FULL_OFF;
}

void PCA9685::Led::attachParent(PCA9685* parent)
{
	_parent = parent;
}

void PCA9685::Led::setIdx(uint8_t idx)
{
	_idx = idx;
}


PCA9685::PCA9685()
{
	init(0, PCA9685_DEFAULT_PRE_SCALE, PCA9685_MODE_MOTOR_DIRECT);
}

PCA9685::PCA9685(uint8_t i2cSlaveAdr)
{
	init(i2cSlaveAdr, PCA9685_DEFAULT_PRE_SCALE, PCA9685_MODE_MOTOR_DIRECT);
}

PCA9685::PCA9685(uint8_t i2cSlaveAdr, float frequency)
{
	init(i2cSlaveAdr, calculatePrescale(frequency), PCA9685_MODE_MOTOR_DIRECT);
}

PCA9685::PCA9685(uint8_t i2cSlaveAdr, float frequency, uint8_t driveMode)
{
	init(i2cSlaveAdr, calculatePrescale(frequency), driveMode);
}

PCA9685::PCA9685(uint8_t i2cSlaveAdr, uint8_t driveMode)
{
	init(i2cSlaveAdr, PCA9685_DEFAULT_PRE_SCALE, driveMode);
}

PCA9685::Led& PCA9685::getLed(uint8_t pin)
{
	if (pin >= PCA9685_NUM_PINS)
		pin = PCA9685_NUM_PINS - 1;

	return _leds[pin];
}

void PCA9685::setup() const
{
	sleep();

	// set auto-increment enabled
	uint8_t mode1 = read8(PCA9685_MODE1);
	mode1 |= 0x20; // set auto increment bit high
	write8(PCA9685_MODE1, mode1);

	// set the prescale
	write8(PCA9685_PRE_SCALE, _preScale);

	// set the drive mode
	uint8_t mode2 = read8(PCA9685_MODE2);
	if (_driveMode == PCA9685_MODE_LED_DIRECT)
	{
		mode2 |= PCA9685_MODE2_INVRT_BIT; // invert
		mode2 &= ~PCA9685_MODE2_OUTDRV_BIT; // open drain
	}
	else if (_driveMode == PCA9685_MODE_N_DRIVER)
	{
		mode2 &= ~PCA9685_MODE2_INVRT_BIT; // no invert
		mode2 |= PCA9685_MODE2_OUTDRV_BIT; // totem pole
	}
	else if (_driveMode == PCA9685_MODE_P_DRIVER)
	{
		// invert high, totem pole
		mode2 |= PCA9685_MODE2_INVRT_BIT; // invert
		mode2 |= PCA9685_MODE2_OUTDRV_BIT; // totem pole
	}
	else
	{
		// default
		mode2 &= ~PCA9685_MODE2_INVRT_BIT; // no invert
		mode2 &= ~PCA9685_MODE2_OUTDRV_BIT; // open drain
	}
	write8(PCA9685_MODE2, mode2);

	wakeup();
}

void PCA9685::readAllLeds()
{
	int i;
	for (i = 0; i < PCA9685_NUM_PINS; ++i)
	{
		_leds[i].read();
	}
}

void PCA9685::writeAllLeds() const
{
	int i;
	for (i = 0; i < PCA9685_NUM_PINS; ++i)
	{
		_leds[i].write();
	}
}


void PCA9685::init(uint8_t i2cSlaveAdr, uint8_t preScale, uint8_t driveMode)
{
	_i2cArd = PCA9685_BASEADR + (i2cSlaveAdr & PCA9685_ADR_PINS);
	_preScale = preScale;
	_driveMode = driveMode;

	int i;
	for (i = 0; i < PCA9685_NUM_PINS; ++i)
	{
		_leds[i].attachParent(this);
		_leds[i].setIdx(i);
	}
}

void PCA9685::sleep() const
{
	uint8_t sleep = read8(PCA9685_MODE1);
	sleep |= PCA9685_MODE1_SLEEP_BIT; // set sleep bit high
	write8(PCA9685_MODE1, sleep);
}

void PCA9685::wakeup() const
{
	uint8_t sleep = read8(PCA9685_MODE1);
	sleep &= ~PCA9685_MODE1_SLEEP_BIT; // set sleep bit low
	write8(PCA9685_MODE1, sleep);
}

uint8_t PCA9685::calculatePrescale(float frequency) const
{
	// frequency has a limit!
	// lower than prescale 3 is not possible (upper bound of ~1500Hz)
	// higher than prescale 0xFF is not possible (lower bound of ~23Hz)
	if (frequency < PCA9685_MIN_FREQUENCY)
		frequency = PCA9685_MIN_FREQUENCY;
	if (frequency > PCA9685_MAX_FREQUENCY)
		frequency = PCA9685_MAX_FREQUENCY;

	// note:
	// yes, the frequency is a bit off and some people get good results with a multiplier of 0.9
	// but: don't overengieer this stuff. you usually shouldn't see the difference. only when using an osci :-)

	return (uint8_t)(floor((PCA9685_CLOCK / 4096 / frequency) + 0.5) - 1);
}

void PCA9685::writeLed(const Led& led) const
{
	// calculate the ON LOW register for this led
	uint8_t reg = PCA9685_LED0_ON_L + (led._idx * 4);

	uint16_t on;
	uint16_t off;

	if (led._value == 0) // full off
	{
		on = 0x0;
		off = PCA9685_LED_OFF_FULL_OFF_BIT;
	}
	else if (led._value >= 4096) // full on
	{
		on = PCA9685_LED_ON_FULL_ON_BIT;
		off = 0x0;
	}
	else
	{
		// distribute load of all 16 pin so that not all pins open at the same time in the cycle
		// offset between pins of 256 (16 * 256 = 4096)
		on = led._idx * 256;
		off = (on + led._value) % 4096;
	}

	write32(reg, on, off);
}

void PCA9685::readLed(Led& led) const
{
	// calculate the ON LOW register for this led
	uint8_t reg = PCA9685_LED0_ON_L + (led._idx * 4);

	uint16_t on;
	uint16_t off;
	read32(reg, on, off);
	if (on == 0x1000)
	{
		led._value = 4096; // full on
	}
	else if (off == 0x1000)
	{
		led._value = 0; // full off
	}
	else
	{
		if (off < on)
		{
			off += 4096;
		}
		led._value = off - on;
	}
}

uint8_t PCA9685::read8(uint8_t reg) const
{
	Wire.beginTransmission(_i2cArd);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(_i2cArd, (uint8_t)1); // request one byte of data from PCA9685
	return Wire.read();
}

void PCA9685::write8(uint8_t reg, uint8_t value) const
{
	Wire.beginTransmission(_i2cArd);
	Wire.write(reg);
	Wire.write(value);
	Wire.endTransmission();
}

void PCA9685::read32(uint8_t reg, uint16_t& on, uint16_t& off) const
{
	Wire.beginTransmission(_i2cArd);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(_i2cArd, (uint8_t)4); // request 4 byte of data from PCA9685

	uint8_t value;
	value = Wire.read();
	on = value & 0xF;
	value = Wire.read();
	on = (value << 8) & 0xF0;
	value = Wire.read();
	off = value & 0xF;
	value = Wire.read();
	off = (value << 8) & 0xF0;
}

void PCA9685::write32(uint8_t reg, uint16_t on, uint16_t off) const
{
	Wire.beginTransmission(_i2cArd);
	Wire.write(reg);
	Wire.write(on); // low byte ON
	Wire.write(on >> 8); // high byte ON
	Wire.write(off); // low byte OFF
	Wire.write(off >> 8); // high byte OFF
	Wire.endTransmission();
}
