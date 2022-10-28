# capra_gpio_control

capra_gpio_control is a node that provide features to toggle the gpio pins of the Nvidia Jetson Xaveir AGX. 

There's a useful library ! [https://github.com/clubcapra/JetsonXavierGPIO.git].

This node advertises two service wich are called by the U.I to toggle some lights on our robot.

If the light in ON, calling the service will shut it down and vice-versa.

You can use them by running the node:

roslaunch capra_gpio_control gpio_control.launch 

You can call the services  :

rosservice call /capra/wrist_light_toggle

or 

rosservice call /capra/back_light_toggle



