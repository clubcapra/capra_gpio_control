#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "JetsonXavierGPIO/jetsonGPIO.h"

const jetsonXavierGPIONumber WRIST_LIGHT_PIN = jetsonXavierGPIONumber::gpio352;
const jetsonXavierGPIONumber BACK_LIGHT_PIN = jetsonXavierGPIONumber::gpio353;
const int ROS_SPIN_RATE = 100;
bool wrist_light_state = pinValues::off;
bool back_light_state = pinValues::off;

bool wrist_light_toggle(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (wrist_light_state == pinValues::off)
  {
    gpioSetValue(WRIST_LIGHT_PIN, pinValues::on);
    wrist_light_state = pinValues::on;
    res.message = "successfully turned wrist_light on";
    res.success = static_cast<unsigned char>(true);
    return true;
  }
  else
  {
    gpioSetValue(WRIST_LIGHT_PIN, pinValues::off);
    wrist_light_state = pinValues::off;
    res.message = "successfully turned wrist_light off";
    res.success = static_cast<unsigned char>(true);
    return true;
  }
}

bool back_light_toggle(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res)
{
  if (back_light_state == pinValues::off)
  {
    gpioSetValue(BACK_LIGHT_PIN, pinValues::on);
    back_light_state = pinValues::on;
    res.message = "successfully turned back_light on";
    res.success = static_cast<unsigned char>(true);
    return true;
  }
  else
  {
    gpioSetValue(BACK_LIGHT_PIN, pinValues::off);
    back_light_state = pinValues::off;
    res.message = "successfully turned back_light off";
    res.success = static_cast<unsigned char>(true);
    return true;
  }
}

/**
 * Initialize the configuration to control the GPIO pin.
 */
void initializeGPIO()
{
  gpioExport(WRIST_LIGHT_PIN);
  gpioSetDirection(WRIST_LIGHT_PIN, pinDirections::outputPin);
  gpioSetValue(WRIST_LIGHT_PIN, pinValues::off);

  gpioExport(BACK_LIGHT_PIN);
  gpioSetDirection(BACK_LIGHT_PIN, pinDirections::outputPin);
  gpioSetValue(BACK_LIGHT_PIN, pinValues::off);

  ROS_INFO("Succesfully initialized gpio's");
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "capra_gpio_control");
  ros::NodeHandle nh;

  initializeGPIO();

  ros::ServiceServer wristLightEnable = nh.advertiseService("wrist_light_toggle", wrist_light_toggle);

  ros::ServiceServer backLightEnable = nh.advertiseService("back_light_toggle", back_light_toggle);

  ros::Rate rate(ROS_SPIN_RATE);  // ROS Rate at 100Hz

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}