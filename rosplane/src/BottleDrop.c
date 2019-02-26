/*
 * Written for AUVSI 2018
 * by Michael Eyler
 * 
 * Purpose:
 * Drop a bottle when given a single input from A6 (the white wire) or an rc input on A4 (the blue wire)
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

Servo h20;//Servo object
int odroidPin = 6;//Odroid input pin is A6
int high = 2012; 
int low = 980; 
double pwm_low = 0.06;//Lower bound of PWM
double pwm_high = 0.1;//Upper bound of PWM

unsigned long highTime;//Time PWM is high
unsigned long lowTime;//Time PWM is low
unsigned long cycleTime;//Total PWM cycle
double dutyCycle;//Fraction of high over total

void setup()
{
  h20.attach(A5);
  h20.writeMicroseconds(low);
}

void loop()
{
  //Calculate dutyCycle by dividing the time it's high over the total time of the cycle
  highTime = pulseIn(A4, HIGH);
  lowTime = pulseIn(A4, LOW);
  cycleTime = highTime + lowTime;
  dutyCycle = double(highTime) / double(cycleTime);

  //When it recieves 1.8V or higher on A6, open the servo (release the bottle)
  if (analogRead(odroidPin) > 340)
  {
    h20.writeMicroseconds(high);
    delay(10);
    h20.writeMicroseconds(low);
    delay(1000);
  }
  else if (dutyCycle < pwm_low && cycleTime != 0)
  {
    h20.writeMicroseconds(high);
    delay(10);
    h20.writeMicroseconds(low);
    delay(1000);
  }

}
