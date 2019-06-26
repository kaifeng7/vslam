/*
 * @Author: fengkai 
 * @Date: 2019-06-25 22:13:35 
 * @Last Modified by:   fengkai 
 * @Last Modified time: 2019-06-25 22:13:35 
 */
#include "map_point.h"

MapPoint::MapPoint()
{

}

MapPoint::MapPoint(const Card &card,const cv::Mat &world_position)
{
    mCard = card;
    mWorldPositionMat = world_position;
}
