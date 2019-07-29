/*
 * @Author: fengkai 
 * @Date: 2019-06-25 22:13:31 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-07-18 14:42:53
 */
#include "key_frame.h"

KeyFrame::KeyFrame()
{
}

KeyFrame::KeyFrame(const Image &image,const int &key_frame_id) :mImage(image),mKeyFrameId(key_frame_id)
{
    mpMapPoints.clear();
}

// void KeyFrame::SetPose(const cv::Mat &Twc)
// {
//     cv::Mat Twc = m_Image.pose;

// }

// void KeyFrame::UpdatePoseMatrices()
// {
//     if (!Tcw.empty())
//     {
//         cv::Mat Rcw = Tcw.rowRange(0, 3).colRange(0, 3);
//         cv::Mat tcw = Tcw.rowRange(0, 3).col(3);
//         cv::Mat Rwc = Rcw.t();
//         Ow = -Rwc * tcw;

//         Twc = cv::Mat::eye(4, 4, Tcw.type());
//         Rwc.copyTo(Twc.rowRange(0, 3).colRange(0, 3));
//         Ow.copyTo(Twc.rowRange(0, 3).col(3));//Ow==twc
//     }
// }

void KeyFrame::setRefMapPoints(MapPoint* mMP)
{
    mpMapPoints.push_back(mMP);
}

int KeyFrame::getMapPointCount()
{   
    return (mpMapPoints.size());
}
