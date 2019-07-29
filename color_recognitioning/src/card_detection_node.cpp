/*
 * @Author: fengkai 
 * @Date: 2019-06-21 16:10:25 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-07-19 16:58:49
 */
#include "card_detection.h"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "card_detectioning");
    VSlam vslam;
    vslam.MainLoop();
    return 0;
}
