/*
 * @Author: fengkai 
 * @Date: 2019-06-25 22:13:46 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-07-31 22:19:21
 */
#ifndef MAP_H
#define MAP_H

#include <set>

#include "key_frame.h"
#include "map_point.h"

class KeyFrame;
class MapPoint;

class Map
{
public:

    std::vector<MapPoint *> mpMapPoints;
    std::vector<KeyFrame *> mpKeyFrames;

    bool insertKeyFrame(KeyFrame* pKF);
    bool insertMapPoint(MapPoint* pMP,KeyFrame* pKF,const int &n);
    void eraseKeyFrame(KeyFrame* pKF);
    void eraseMapPoint(MapPoint* pMP);

    //void getMapPoints();


    Map();
    ~Map();
};

#endif