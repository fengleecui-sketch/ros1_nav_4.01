#include "robot_usart/usart_config.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "usart");

    usartConfig usart;

    ros::AsyncSpinner spinner(2);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}