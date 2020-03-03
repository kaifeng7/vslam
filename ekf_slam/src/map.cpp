/*
 * @Author: fengkai 
 * @Date: 2019-06-25 10:45:13 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-08-01 16:27:15
 */
#include "map.h"

Map::Map()
{
    std::vector<KeyFrame *> kf(5120, nullptr);
    mpKeyFrames = kf;

    std::vector<MapPoint *> mp(128, nullptr);
    mpMapPoints = mp;
    
}

Map::~Map()
{
}

bool Map::insertKeyFrame(KeyFrame *pKF)
{
    mpKeyFrames.at(pKF->mKeyFrameId) = pKF;
    return true;
}
bool Map::insertMapPoint(MapPoint *pMP, KeyFrame *pKF, const int &n)
{
    if (mpMapPoints.at(pMP->mCard.card_id) == nullptr)
    {
        mpMapPoints.at(pMP->mCard.card_id) = pMP;
        mpMapPoints.at(pMP->mCard.card_id)->setRefKeyFrame(pKF, n);
        
        return true;
    }
    else
    {
        mpMapPoints.at(pMP->mCard.card_id)->setRefKeyFrame(pKF, n);
        return false;
    }
}

void Map::eraseKeyFrame(KeyFrame *pKF)
{
    delete mpKeyFrames.at(pKF->mKeyFrameId);
    mpKeyFrames.at(pKF->mKeyFrameId) = nullptr;
}

void Map::eraseMapPoint(MapPoint *pMP)
{
    delete mpMapPoints.at(pMP->mCard.card_id);
    mpMapPoints.at(pMP->mCard.card_id) = nullptr;
}

// void Map::getMapPoints()
// {
//     std::set<MapPoint>::iterator iter = mspMapPoints.begin();
//     for(;iter != mspMapPoints.end();iter++)
//     {
//         mMapPoints.push_back(*iter);
//     }
// }