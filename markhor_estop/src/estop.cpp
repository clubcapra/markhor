#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "JetsonXavierGPIO/jetsonGPIO.h"

bool estop_value = true; //default value for the Estop.
const jetsonXavierGPIONumber ESTOP_PIN = jetsonXavierGPIONumber::gpio428;


/**
 * Call back function for to toggle the Estop pin. This will change the electric output on the pin and enable or disable
 * the estop.
 */
bool toggleEstopEnable(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    res.message = "successfully toggle estop to on";
    gpioSetValue(ESTOP_PIN, 1);
    res.success = static_cast<unsigned char>(true);
    return true;
}

/**
 * Call back function for to toggle the Estop pin. This will change the electric output on the pin and enable or disable
 * the estop.
 */
bool toggleEstopDisable(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    res.message = "successfully toggle estop to off";
    gpioSetValue(ESTOP_PIN, 0);
    res.success = static_cast<unsigned char>(true);
    return true;
}

/**
 * Initialize the configuration to control the GPIO pin.
 */
void initializeGPIO() {
    gpioExport(ESTOP_PIN);
    gpioSetDirection(ESTOP_PIN, 1);
    gpioSetValue(ESTOP_PIN, 1);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "markhor_estop");
    ros::NodeHandle nh;

    initializeGPIO();

    ros::ServiceServer serviceEnable = nh.advertiseService("estop_enable", toggleEstopEnable);
    ros::ServiceServer serviceDisable = nh.advertiseService("estop_disable", toggleEstopDisable);
    ros::spin();

    return 0;
}