/*
 * @Author: fengkai 
 * @Date: 2019-06-21 16:10:25 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-06-21 17:37:01
 */
#include "card_detection.h"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "card_detectioning");
    CardDetection card_detection;
    card_detection.MainLoop();
    return 0;
}
