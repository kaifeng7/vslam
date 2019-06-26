/*
 * @Author: fengkai 
 * @Date: 2019-06-25 10:45:13 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-06-25 22:28:57
 */
#include "map.h"

SlamMap::SlamMap()
{
}

SlamMap::~SlamMap()
{
    
}

// void SlamMap::insertKeyFrame(KeyFrame *pKF)
// {
//     mspKeyFrames.insert(pKF);
// }
// void SlamMap::insertMapPoint(MapPoint *pMP)
// {
//     mspMapPoints.insert(pMP);
// }

// void SlamMap::eraseKeyFrame(KeyFrame *pKF)
// {
//     mspKeyFrames.erase(pKF);
// }

// void SlamMap::eraseMapPoint(MapPoint *pMP)
// {
//     mspMapPoints.erase(pMP);
// }

// void SlamMap::getMapPoints()
// {
//     std::set<MapPoint>::iterator iter = mspMapPoints.begin();
//     for(;iter != mspMapPoints.end();iter++)
//     {
//         mMapPoints.push_back(*iter);
//     }
// }