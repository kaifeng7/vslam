/*
 * @Author: fengkai 
 * @Date: 2019-06-21 16:10:25 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-07-31 16:53:07
 */
#include "card_detection.h"

int main(int argc,char** argv)
{
    ros::init(argc, argv, "card_detectioning");
    VSlam vslam;

    vslam.MainLoop();
    return 0;
}
