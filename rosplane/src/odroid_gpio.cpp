#include <odroid_gpio.h>
#include <wiringPi.h>

namespace rosplane
{
	OdroidGPIO::OdroidGPIO():
	  nh_(ros::NodeHandle())
	{
	  gpio_0_pulse_actuate_srv_ = nh_.advertiseService("gpio_0_pulse_actuate", &rosplane::OdroidGPIO::gpio0pulseActuate, this);
	  gpio_0_high_srv_ = nh_.advertiseService("gpio_0_high", &rosplane::OdroidGPIO::gpio0high, this);
	  gpio_0_low_srv_  = nh_.advertiseService("gpio_0_low", &rosplane::OdroidGPIO::gpio0low, this);
	}
	bool OdroidGPIO::gpio0pulseActuate(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
	{
	  // Pulse 1
	  digitalWrite(0,LOW);
	  ros::Duration(0.05).sleep();
	  digitalWrite(0,HIGH);
	  ros::Duration(0.05).sleep();

	  // Pulse 2
	  digitalWrite(0,LOW);
	  ros::Duration(0.05).sleep();
	  digitalWrite(0,HIGH);
	  ros::Duration(0.05).sleep();

	  // Pulse 3
	  digitalWrite(0,LOW);
	  ros::Duration(0.05).sleep();
	  digitalWrite(0,HIGH);
	  ros::Duration(0.05).sleep();

	  digitalWrite(0,LOW);
	  ros::Duration(0.05).sleep();

	  res.success = true;
	  return true;
	}
	bool OdroidGPIO::gpio0high(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
	{
	  digitalWrite(0,HIGH);
	  res.success = true;
	  return true;
	}
	bool OdroidGPIO::gpio0low(std_srvs::Trigger::Request &req, std_srvs::Trigger:: Response &res)
	{
	  digitalWrite(0,LOW);
	  res.success = true;
	  return true;
	}
} //end namespace rosplane
int main(int argc, char **argv)
{
  ros::init(argc, argv, "odroid_gpio");

  // setup pins
  wiringPiSetup ();
  pinMode (0, OUTPUT);
  digitalWrite(0, LOW);

  // run ros
  rosplane::OdroidGPIO obj;
  ros::spin();
  return 0;
}
