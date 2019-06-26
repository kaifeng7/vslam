/*
 * @Author: fengkai 
 * @Date: 2019-06-25 22:13:43 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-06-25 22:23:59
 */
#ifndef KEY_FRAME_H
#define KEY_FRAME_H

#include <opencv2/opencv.hpp>
#include "map_point.h"
#include "card.h"

class MapPoint;
class KeyFrame
{

public:
    KeyFrame();
    KeyFrame(const Image &image,const int &key_frame_id);

    void SetPose(const cv::Mat &Twc);
    //std::vector<MapPoint> GetMapPoints();
    //void SetMapPoints();
    //void UpdatePoseMatrices();

    int mKeyFrameId;

    // Camera 
    Image mImage;

    KeyFrame *refKeyFrame;

    std::vector<MapPoint> mMapPoints;


    // SE3 Pose and camera center

    //设置KeyFrame中成员变量Tcw,Twc,Ow;
    //cv::Mat mRwc;//Rotation world2camera
    //cv::Mat mtwc;//translation
    //cv::Mat mRcw;//Rotation T
    //cv::Mat mTcw;
};

#endif