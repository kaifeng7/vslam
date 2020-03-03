/*
 * @Author: fengkai 
 * @Date: 2019-06-25 22:13:35 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-08-01 15:20:02
 */
#include "map_point.h"

MapPoint::MapPoint()
{

}

MapPoint::MapPoint(const Card &card,const Eigen::Vector3d &world_position_point,const int &id)
{
    mCard = card;
    mWorldPositionPoint = world_position_point;
    mMapPointId = id;
}


MapPoint::~MapPoint()
{
}

int MapPoint::getKeyFrameCount()
{
    return(mpKeyFrames.size());
}

void MapPoint::setRefKeyFrame(KeyFrame *mKF,int n)
{
    mpKeyFrames.push_back(std::make_pair(mKF,n));
}