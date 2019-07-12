/*
 * @Author: fengkai 
 * @Date: 2019-06-24 10:05:50 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-06-25 10:02:40
 */
#include "camera.h"

Camera::Camera()
{

}


Camera::~Camera()
{

}


Eigen::Vector3d Camera::world2camera(const Eigen::Vector3d &point_in_world)
{
    return mTwc.inverse() * point_in_world;
}

Eigen::Vector3d Camera::camera2world(const Eigen::Vector3d &point_in_camera)
{
    return mTwc * point_in_camera; 
}

cv::Point2i Camera::camera2pixel(const Eigen::Vector3d &point_in_camera,const CameraParam &camera_param)
{
    double x,y;
    x = camera_param.mNewK.at<double>(0,0) * point_in_camera(0,0)/point_in_camera(2,0)+camera_param.mNewK.at<double>(0,2);
    y = camera_param.mNewK.at<double>(1,1) * point_in_camera(1,0)/point_in_camera(2,0)+camera_param.mNewK.at<double>(1,2);
    return cv::Point2i((int)x,(int)y);
}

Eigen::Vector3d Camera::pixel2camera(const cv::Point2i &point_in_pixel,const CameraParam &camera_param,double depth)
{
    double x,y;
    x = ((double)point_in_pixel.x-camera_param.mNewK.at<double>(0,2))*depth/camera_param.mNewK.at<double>(0,0);
    y = ((double)point_in_pixel.y-camera_param.mNewK.at<double>(1,2))*depth/camera_param.mNewK.at<double>(1,1);
    return Eigen::Vector3d(x,y,depth);
}


void CameraParam::setK()
{    
    mK = (cv::Mat_<double>(3,3)<<fx,0 ,cx,
                                 0 ,fy,cy,
                                 0 ,0 , 1);
    mNewK = mK;
}

void CameraParam::setD()
{    
    mD = (cv::Mat_<double>(5,1)<<k1,k2,p1,p2,k3);
}