/*
@file	PCA9685_example.ino
Arduino sample code file for PCA9685
16-channel, 12-bit PWM Fm+ I2C-bus LED controller
See PCA9685.pdf datasheet for details

@author	Thomas Oppenhoff

https://github.com/TOppenhoff/PCA9685

Language: C++
License: GNU Public License

*/

#include <Wire.h>
#include <Serial.h>
#include <PCA9685.h>

// PCA9685 with default settings
PCA9685 driver = PCA9685(0x0, PCA9685_MODE_LED_DIRECT, 800.0);

void setup()
{
  // initialize serial for debug output
  Serial.begin(9600);

  // initialize TwoWire communication
  Wire.begin();

  // setup the PCA9685 
  driver.setup();
}

void loop()
{
  // turn all 4 LEDs fully on
  Serial.println("FULL ON");
  driver.getPin(PCA9685_LED0).fullOnAndWrite();
  driver.getPin(PCA9685_LED1).fullOnAndWrite();
  driver.getPin(PCA9685_LED2).fullOnAndWrite();
  driver.getPin(PCA9685_LED3).fullOnAndWrite();

  // wait for half a second
  delay(500);

  // turn all 4 LEDs fully off
  Serial.println("FULL OFF");
  driver.getPin(PCA9685_LED0).fullOffAndWrite();
  driver.getPin(PCA9685_LED1).fullOffAndWrite();
  driver.getPin(PCA9685_LED2).fullOffAndWrite();
  driver.getPin(PCA9685_LED3).fullOffAndWrite();

  // wait for half a second
  delay(500);

  // go through all possible 512 of 4096 possible pwm steps in the first LED
  Serial.println("FADE IN LED0");
  int i;
  for (i = 0; i < PCA9685_MAX_VALUE; i = i + 8)
  {
    // set the pwm value of the first led
    driver.getPin(PCA9685_LED0).setValueAndWrite(i);
    delay(5);
  }
  Serial.println("FADE OUT LED0");
  for (i = PCA9685_MAX_VALUE; i >= 0; i = i - 8)
  {
    // set the pwm value of the first led
    driver.getPin(PCA9685_LED0).setValueAndWrite(i);
    delay(5);
  }

  // Disco Party!!
  Serial.println("DISCO DISCO PARTY!");
  randomSeed(analogRead(0));
  for (i = 0; i < 1000; ++i)
  {
    driver.getPin(PCA9685_LED0).setValueAndWrite(random(0, 5) * 1024);
    driver.getPin(PCA9685_LED1).setValueAndWrite(random(0, 5) * 1024);
    driver.getPin(PCA9685_LED2).setValueAndWrite(random(0, 5) * 1024);
    driver.getPin(PCA9685_LED3).setValueAndWrite(random(0, 5) * 1024);
    delay(5);
  }
}
