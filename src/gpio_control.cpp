#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "JetsonXavierGPIO/jetsonGPIO.h"

// const jetsonXavierGPIONumber WRIST_LIGHT_PIN = jetsonXavierGPIONumber::gpio352;
// const jetsonXavierGPIONumber BACK_LIGHT_PIN = jetsonXavierGPIONumber::gpio353;
const int ROS_SPIN_RATE = 100;
bool wrist_light_state = pinValues::off;
bool back_light_state = pinValues::off;

bool gpio_toggle(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res, jetsonXavierGPIONumber pin_num);
{
  if (wrist_light_state == pinValues::off)
  {
    gpioSetValue(pin_num, pinValues::on);
    wrist_light_state = pinValues::on;
    res.message = "successfully turned wrist_light on";
    res.success = static_cast<unsigned char>(true);
    return true;
  }
  else
  {
    gpioSetValue(pin_num, pinValues::off);
    wrist_light_state = pinValues::off;
    res.message = "successfully turned wrist_light off";
    res.success = static_cast<unsigned char>(true);
    return true;
  }
}

/**
 * Initialize the configuration to control the GPIO pin.
 */
void initializeGPIO(jetsonXavierGPIONumber pin_num)
{
  gpioExport(pin_num);

  // si c'est un output
  gpioSetDirection(pin_num, pinDirections::outputPin);
  gpioSetValue(pin_num, pinValues::off);
  ROS_INFO("Succesfully initialized gpio's");
  // sinon si cest un input
  gpioSetDirection(pin_num, pinDirections::inputPin);
  ROS_INFO("Succesfully initialized gpio's");

}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "capra_gpio_control");
  ros::NodeHandle nh;
  int int_var;
  double double_var;
  std::string string_var;

  ros::param::get("/my_integer", int_var);
  ros::param::get("/my_float", double_var);
  ros::param::get("/my_string", string_var);

  ROS_INFO("Int: %d, Float: %lf, String: %s", int_var, double_var, string_var.c_str());

  int used_pin = jetsonXavierGPIONumber::gpio352;
  initializeGPIO((jetsonXavierGPIONumber)used_pin);

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