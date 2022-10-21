#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "JetsonXavierGPIO/jetsonGPIO.h"


const jetsonXavierGPIONumber WRIST_LIGHT_PIN = jetsonXavierGPIONumber::gpio352;
const jetsonXavierGPIONumber BACK_LIGHT_PIN = jetsonXavierGPIONumber::gpio353;
const int ros_spin_rate = 100;



bool wristLightOn(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.message = "successfully turned wrist_light on";
    gpioSetValue(WRIST_LIGHT_PIN, 1);
    res.success = static_cast<unsigned char>(true);
    return true;
}


bool wristLightOff(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.message = "successfully turned wrist_light off";
    gpioSetValue(WRIST_LIGHT_PIN, 0);
    res.success = static_cast<unsigned char>(true);
    return true;
}


bool backLightOn(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.message = "successfully turned wrist_light on";
    gpioSetValue(BACK_LIGHT_PIN, 1);
    res.success = static_cast<unsigned char>(true);
    return true;
}


bool backLightOff(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    res.message = "successfully turned wrist_light off";
    gpioSetValue(BACK_LIGHT_PIN, 0);
    res.success = static_cast<unsigned char>(true);
    return true;
}

/**
 * Initialize the configuration to control the GPIO pin.
 */
void initializeGPIO()
{
    gpioExport(WRIST_LIGHT_PIN);
    gpioSetDirection(WRIST_LIGHT_PIN,outputPin);
    gpioSetValue(WRIST_LIGHT_PIN, false);

    gpioExport(BACK_LIGHT_PIN);
    gpioSetDirection(BACK_LIGHT_PIN,outputPin);
    gpioSetValue(BACK_LIGHT_PIN, false);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "capra_gpio_control" );
    ros::NodeHandle nh;

    initializeGPIO();

    ros::ServiceServer wristLightEnable = nh.advertiseService("wristLightOn", wristLightOn);
    ros::ServiceServer wristLightDisable = nh.advertiseService("wristLightOff", wristLightOff);

    ros::ServiceServer backLightEnable = nh.advertiseService("backLightOn", backLightOn);
    ros::ServiceServer backLightDisable = nh.advertiseService("backLightOff", backLightOff);
    
    ros::Rate rate(ros_spin_rate); // ROS Rate at 100Hz

    ros::spinOnce();
    rate.sleep();
    
    return 0;
}