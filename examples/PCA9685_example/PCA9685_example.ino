#include <Wire.h>
#include <PCA9685.h>

// PCA9685 with default settings
PCA9685 driver;

void setup()
{
	// initialize TwoWire communication
	Wire.begin();

	// setup the PCA9685 
	driver.setup();
}

void loop()
{
	// turn all 4 LEDs fully on
	driver.getLed(PCA9685_LED0).fullOn();
	driver.getLed(PCA9685_LED1).fullOn();
	driver.getLed(PCA9685_LED2).fullOn();
	driver.getLed(PCA9685_LED3).fullOn();
	driver.writeAllLeds();

	// wait for half a second
	delay(500);

	// turn all 4 LEDs fully off
	driver.getLed(PCA9685_LED0).fullOff();
	driver.getLed(PCA9685_LED1).fullOff();
	driver.getLed(PCA9685_LED2).fullOff();
	driver.getLed(PCA9685_LED3).fullOff();
	driver.writeAllLeds();

	// wait for half a second
	delay(500);

	// go through all possible 4096 pwm steps in all 4 LEDs
	int i;
	for (i = 0; i < PCA9685_MAX_VALUE; ++i)
	{
		// set the pwm value of the first led
		driver.getLed(PCA9685_LED0).setValue(i);
		// write it to the PCA9685
		driver.getLed(PCA9685_LED0).write();
		// set the pwm value of the second led
		driver.getLed(PCA9685_LED1).setValue((i + 1024) % PCA9685_MAX_VALUE);
		// write it to the PCA9685
		driver.getLed(PCA9685_LED1).write();
		// set the pwm value of the third led
		driver.getLed(PCA9685_LED2).setValue((i + 2048) % PCA9685_MAX_VALUE);
		// write it to the PCA9685
		driver.getLed(PCA9685_LED2).write();
		// set the pwm value of the forth led
		driver.getLed(PCA9685_LED3).setValue((i + 3072) % PCA9685_MAX_VALUE);
		// write it to the PCA9685
		driver.getLed(PCA9685_LED3).write();
		delay(20);
	}
}
