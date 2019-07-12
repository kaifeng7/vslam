/*
 * @Author: fengkai 
 * @Date: 2019-06-25 22:13:38 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-07-12 16:21:03
 */
#ifndef MAP_POINT_H
#define MAP_POINT_H
#include <opencv2/opencv.hpp>

#include "card.h"

class KeyFrame;
class Card;
class MapPoint
{

public:
    MapPoint();
    MapPoint(const Card &card,const Eigen::Vector3d &world_position_point, const int &id);
    ~MapPoint();

    Card mCard;
    int mMapPointId;

    Eigen::Matrix4d mWorldPositionMat;
    Eigen::Vector3d mWorldPositionPoint;
    
    //cv::Point3d Normal;//normal of viewing direction
        
    //观测到该点的KeyFrame
    std::map<KeyFrame *,int> mpKeyFrames;

    
    int getKeyFrameCount();
    void setRefKeyFrame(KeyFrame *mKF,int n);
};


#endif