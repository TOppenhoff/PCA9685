/*
@file	PCA9685.h
Code file for PCA9685
16-channel, 12-bit PWM Fm+ I2C-bus LED controller
See PCA9685.pdf datasheet for details

@author	Thomas Oppenhoff

https://github.com/TOppenhoff/PCA9685

Language: C++
License: GNU Public License

*/

#include <PCA9685.h>
#include <Wire.h>

#define PCA9685_BASEADR			0x40 // I2C base address

#define PCA9685_MODE1			0x00 // Mode register 1
#define PCA9685_MODE2			0x01 // Mode register 2
#define PCA9685_PRE_SCALE		0xFE // prescaler for output frequency
#define PCA9685_LED0_ON_L		0x06 // LED0 ON Low Byte

#define PCA9685_DEFAULT_PRE_SCALE		0x1E
#define PCA9685_ADR_PINS				0x3F
#define PCA9685_CLOCK					25000000

#define PCA9685_MODE1_SLEEP_BIT			0x10
#define PCA9685_LED_ON_FULL_ON_BIT		0x1000
#define PCA9685_LED_OFF_FULL_OFF_BIT	0x1000
#define PCA9685_MODE2_INVRT_BIT			0x10
#define PCA9685_MODE2_OUTDRV_BIT		0x04

PCA9685::PWMOutputPin::PWMOutputPin()
{
  _parent = NULL;
  _idx = -1;
  _value = PCA9685_FULL_OFF;
}

void PCA9685::PWMOutputPin::write() const
{
  _parent->writePWMOutputPin(*this);
}

uint16_t PCA9685::PWMOutputPin::getValue()
{
  return _value;
}

void PCA9685::PWMOutputPin::setValue(uint16_t value)
{
  if (value > PCA9685_MAX_VALUE)
    value = PCA9685_MAX_VALUE;

  _value = value;
}

void PCA9685::PWMOutputPin::setValueAndWrite(uint16_t value)
{
  setValue(value);
  write();
}

void PCA9685::PWMOutputPin::fullOn()
{
  _value = PCA9685_FULL_ON;
}

void PCA9685::PWMOutputPin::fullOnAndWrite()
{
  fullOn();
  write();
}

void PCA9685::PWMOutputPin::fullOff()
{
  _value = PCA9685_FULL_OFF;
}

void PCA9685::PWMOutputPin::fullOffAndWrite()
{
  fullOff();
  write();
}

void PCA9685::PWMOutputPin::attachParent(PCA9685* parent)
{
  _parent = parent;
}

void PCA9685::PWMOutputPin::setIdx(uint8_t idx)
{
  _idx = idx;
}


PCA9685::PCA9685()
{
  init(0, PCA9685_MODE_MOTOR_DIRECT, PCA9685_DEFAULT_PRE_SCALE);
}

PCA9685::PCA9685(uint8_t i2cSlaveAdr)
{
  init(i2cSlaveAdr, PCA9685_MODE_MOTOR_DIRECT, PCA9685_DEFAULT_PRE_SCALE);
}

PCA9685::PCA9685(uint8_t i2cSlaveAdr, uint8_t driveMode)
{
  init(i2cSlaveAdr, driveMode, PCA9685_DEFAULT_PRE_SCALE);
}

PCA9685::PCA9685(uint8_t i2cSlaveAdr, uint8_t driveMode, float frequency)
{
  init(i2cSlaveAdr, driveMode, calculatePrescale(frequency));
}

PCA9685::PWMOutputPin& PCA9685::getPin(uint8_t pin)
{
  if (pin >= PCA9685_NUM_PINS)
    pin = PCA9685_NUM_PINS - 1;

  return _pins[pin];
}


float PCA9685::getFrequency() const
{
  return (float)PCA9685_CLOCK / (float)4096 / ((float)_preScale + 1.0);
}

void PCA9685::setFrequency(float frequency)
{
  _preScale = calculatePrescale(frequency);
}

uint8_t PCA9685::getPreScale() const
{
  return _preScale;
}

void PCA9685::setPreScale(uint8_t preScale)
{
  _preScale = preScale;
}

uint8_t PCA9685::getDriveMode() const
{
  return _driveMode;
}

void PCA9685::setDriveMode(uint8_t driveMode)
{
  _driveMode = driveMode;
}


void PCA9685::setup() const
{
  sleep();

#ifdef PCA9685_SERIAL_DEBUG
  Serial.print("Enabling Auto Increment! Reg=0x");
  Serial.print(PCA9685_MODE1, HEX);
#endif
  // set auto-increment enabled
  uint8_t mode1 = read8(PCA9685_MODE1);
#ifdef PCA9685_SERIAL_DEBUG
  Serial.print(" previous=");
  Serial.print(mode1, BIN);
#endif
  mode1 |= 0x20; // set auto increment bit high
#ifdef PCA9685_SERIAL_DEBUG
  Serial.print(" new=");
  Serial.println(mode1, BIN);
#endif
  write8(PCA9685_MODE1, mode1);

#ifdef PCA9685_SERIAL_DEBUG
  Serial.print("PreScale=");
  Serial.println(_preScale, BIN);
#endif
  // set the prescale
  write8(PCA9685_PRE_SCALE, _preScale);

  // set the drive mode
#ifdef PCA9685_SERIAL_DEBUG
  Serial.print("Setting drive mode! Mode=0x");
  Serial.print(_driveMode);
  Serial.print(" reg=0x");
  Serial.print(PCA9685_MODE2);
#endif
  uint8_t mode2 = read8(PCA9685_MODE2);
#ifdef PCA9685_SERIAL_DEBUG
  Serial.print(" previous=");
  Serial.print(mode2, BIN);
#endif
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
#ifdef PCA9685_SERIAL_DEBUG
  Serial.print(" new=");
  Serial.println(mode2, BIN);
#endif
  write8(PCA9685_MODE2, mode2);

  wakeup();
}

void PCA9685::writeAllPins() const
{
  int i;
  for (i = 0; i < PCA9685_NUM_PINS; ++i)
  {
    _pins[i].write();
  }
}

void PCA9685::init(uint8_t i2cSlaveAdr, uint8_t driveMode, uint8_t preScale)
{
  _i2cArd = PCA9685_BASEADR + (i2cSlaveAdr & PCA9685_ADR_PINS);
  _preScale = preScale;
  _driveMode = driveMode;

  int i;
  for (i = 0; i < PCA9685_NUM_PINS; ++i)
  {
    _pins[i].attachParent(this);
    _pins[i].setIdx(i);
  }
}

void PCA9685::sleep() const
{
  uint8_t awake = read8(PCA9685_MODE1);
  uint8_t sleep = awake | PCA9685_MODE1_SLEEP_BIT; // set sleep bit high
  write8(PCA9685_MODE1, sleep);
  delay(5); // wait until cycle ends for sleep to be active

#ifdef PCA9685_SERIAL_DEBUG
  Serial.print("Entering sleep mode! reg=0x");
  Serial.print(PCA9685_MODE1, HEX);
  Serial.print(" previous=");
  Serial.print(awake, BIN);
  Serial.print(" new=");
  Serial.println(sleep, BIN);
#endif
}

void PCA9685::wakeup() const
{
  uint8_t sleep = read8(PCA9685_MODE1);
  uint8_t wakeup = sleep & ~PCA9685_MODE1_SLEEP_BIT; // set sleep bit low
  write8(PCA9685_MODE1, wakeup);

#ifdef PCA9685_SERIAL_DEBUG
  Serial.print("Wakeup from sleep mode! reg=0x");
  Serial.print(PCA9685_MODE1, HEX);
  Serial.print(" previous=");
  Serial.print(sleep, BIN);
  Serial.print(" new=");
  Serial.println(wakeup, BIN);
#endif
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

  uint8_t prescale = (uint8_t)(floor((PCA9685_CLOCK / 4096 / frequency) + 0.5) - 1);

#ifdef PCA9685_SERIAL_DEBUG
  Serial.print("frequency=");
  Serial.print(frequency);
  Serial.print(" prescale=");
  Serial.println(prescale);
#endif

  return prescale;
}

void PCA9685::writePWMOutputPin(const PWMOutputPin& pin) const
{
  // calculate the ON LOW register for this led
  uint8_t reg = PCA9685_LED0_ON_L + (pin._idx * 4);

  uint16_t on;
  uint16_t off;

  if (pin._value == PCA9685_FULL_OFF) // full off
  {
    on = 0x0;
    off = PCA9685_LED_OFF_FULL_OFF_BIT;
  }
  else if (pin._value >= PCA9685_FULL_ON) // full on
  {
    on = PCA9685_LED_ON_FULL_ON_BIT;
    off = 0x0;
  }
  else
  {
    // distribute load of all 16 pin so that not all pins open at the same time in the cycle
    // offset between pins of 256 (16 * 256 = 4096)
    on = pin._idx * 256;
    off = (on + pin._value) % 4096;
  }

#ifdef PCA9685_SERIAL_DEBUG
  Serial.print("WRITING LED Idx=");
  Serial.print(led._idx);
  Serial.print(" Value=");
  Serial.print(led._value);
  Serial.print(" register=0x");
  Serial.print(reg, HEX);
  Serial.print(" ON=0x");
  Serial.print(on, HEX);
  Serial.print(" OFF=0x");
  Serial.println(off, HEX);
#endif

  write32(reg, on, off);
}

uint8_t PCA9685::read8(uint8_t reg) const
{
  Wire.beginTransmission(_i2cArd);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(_i2cArd, (uint8_t)1); // request one byte of data from PCA9685
  if (Wire.available())
    return Wire.read();

  return 0x0;
}

void PCA9685::write8(uint8_t reg, uint8_t value) const
{
  Wire.beginTransmission(_i2cArd);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
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
