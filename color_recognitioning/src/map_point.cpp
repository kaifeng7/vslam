/*
 * @Author: fengkai 
 * @Date: 2019-06-25 22:13:35 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-07-12 16:21:20
 */
#include "map_point.h"

MapPoint::MapPoint()
{

}

MapPoint::MapPoint(const Card &card,const Eigen::Vector3d &world_position_point,const int &id)
{
    mCard = card;
    mWorldPositionMat = world_position_point;
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
    mpKeyFrames.insert(std::make_pair(mKF,n));
}