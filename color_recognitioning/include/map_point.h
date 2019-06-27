/*
 * @Author: fengkai 
 * @Date: 2019-06-25 22:13:38 
 * @Last Modified by:   fengkai 
 * @Last Modified time: 2019-06-25 22:13:38 
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
    MapPoint(const Card &card,const cv::Mat &world_position);
    ~MapPoint();

    //void addKeyFrame(const KeyFrame &keyframe);

    Card mCard;
    cv::Point3d mCameraPositionPoint;
    cv::Mat mWorldPositionMat;
    cv::Point3d mWorldPositionPoint;
    cv::Point3d Normal;//normal of viewing direction
    
    //观测到该点的KeyFrame
    //std::map<KeyFrame *,int> my_observation;


};


#endif