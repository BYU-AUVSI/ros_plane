/*
 * Written for AUVSI 2018
 * by Michael Eyler
 * 
 * Modified for AUVSI 2019
 * by Brandon McBride and Jacob Willis
 * 
 * Purpose:
 * Drop a payload when given a single input from A6 (the white wire) or an rc input on A4 (the blue wire)
 * Pins used from left to right -->:
 * 
 * A4 RC input (White Wire)
 * A5 PWM output (on the six pin array below)
 * A6 Odroid Input (blue wire)
 * note: The black and purple wires are both to insure common ground
 * 
 * six pin array is as follows when looking at the Arduino with the usb on the left. * 
 * __<-USB_on_Arduino__
 *     (Gnd)(5V)(PWM)
 *     (Gnd)(5V)(PWM)
 * 
 */

#include <Servo.h>

Servo h20; // Servo object
int odroidPin = 6; // Odroid input pin is A6
int high = 2100; 
int low = 900; 
double pwm_low = 0.06; // Lower bound of PWM
double pwm_high = 0.1; // Upper bound of PWM

unsigned long highTime; // Time PWM is high
unsigned long lowTime; // Time PWM is low
unsigned long cycleTime; // Total PWM cycle
double dutyCycle; // Fraction of high over total
bool isLow = false; // The odroid pin has gone high

void setup()
{
  h20.attach(A5);
  h20.writeMicroseconds(low);
//  Serial.begin(115200);
//  Serial.println("Setup completed");
}

void loop()
{
  // When it recieves 1.8V or higher on A6, open the servo (release the bottle)
  // 340 comes from 1.7/5 * 1024 (5 V max measured through 10 bits)

  // Look for a rising edge
  if (analogRead(odroidPin) < 50)
  {
    delay(50);
    if (analogRead(odroidPin) < 50)
    {
      isLow = true;
    }
  }
 
  if (isLow && analogRead(odroidPin) > 340) // < 50)
  {
//      Serial.println("Low to high transition detected");
//      Serial.println("Deploying payload");
      h20.writeMicroseconds(high);
      delay(20);
      h20.writeMicroseconds(low);
      delay(1000);
      isLow = false;
//      Serial.println("Deployment delay complete, waiting for command...");
  }
}