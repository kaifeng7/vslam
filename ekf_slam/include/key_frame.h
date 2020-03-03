/*
 * @Author: fengkai 
 * @Date: 2019-06-25 22:13:43 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-07-12 16:12:12
 */
#ifndef KEY_FRAME_H
#define KEY_FRAME_H

#include <opencv2/opencv.hpp>
#include "card.h"
#include "map_point.h"


class MapPoint;
class Image;
class KeyFrame
{

public:
    KeyFrame();
    KeyFrame(const Image &image,const int &key_frame_id);

    void setPose(const cv::Mat &Twc);
    //std::vector<MapPoint> GetMapPoints();
    //void UpdatePoseMatrices();

    int mKeyFrameId;

    // Camera 
    Image mImage;

    KeyFrame *refKeyFrame;

    std::vector<MapPoint *> mpMapPoints;

    int getMapPointCount();
    void setRefMapPoints(MapPoint *mMP);

    // SE3 Pose and camera center

    //设置KeyFrame中成员变量Tcw,Twc,Ow;
    //cv::Mat mRwc;//Rotation world2camera
    //cv::Mat mtwc;//translation
    //cv::Mat mRcw;//Rotation T
    //cv::Mat mTcw;
};

#endif