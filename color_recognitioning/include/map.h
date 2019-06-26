/*
 * @Author: fengkai 
 * @Date: 2019-06-25 22:13:46 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-06-25 22:27:53
 */
#ifndef MAP_H
#define MAP_H

#include<set>

#include "key_frame.h"
#include "map_point.h"

class KeyFrame;
class MapPoint;

class SlamMap
{
public:
    //既随机访问，又随时插入删除
    //std::set<MapPoint* > mspMapPoints;
    //std::set<KeyFrame* > mspKeyFrames;

    std::vector<MapPoint> mMapPoints;
    std::vector<KeyFrame> mKeyFrames;

    //void insertKeyFrame(KeyFrame *pKF);
    //void insertMapPoint(MapPoint *pMP);
    //void eraseKeyFrame(KeyFrame *pKF);
    //void eraseMapPoint(MapPoint *pMP);

    //void getMapPoints();

    SlamMap();
    ~SlamMap();

};

#endif