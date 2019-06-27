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

Camera::Camera(const double &fx,const double &fy,const double &cx,const double &cy):mFx(fx),mFy(fy),mCx(cx),mCy(cy)
{
    mK = setK(mFx,mFy,mCx,mCy);
}

Camera::~Camera()
{

}

cv::Mat Camera::setK(const double &fx,const double &fy,const double &cx,const double &cy)
{    
    return(cv::Mat_<double>(3,3)<<fx,0, cx,
                                  0, fy,cy,
                                  0, 0, 1);
}

Eigen::Vector3d Camera::world2camera(const Eigen::Vector3d &point_in_world)
{
    return mTwc.inverse() * point_in_world;
}

Eigen::Vector3d Camera::camera2world(const Eigen::Vector3d &point_in_camera)
{
    return mTwc * point_in_camera; 
}

Eigen::Vector2d Camera::camera2pixel(const Eigen::Vector3d &point_in_camera)
{
    double x,y;
    x = mFx * point_in_camera(0,0)/point_in_camera(2,0)+mCx;
    y = mFy * point_in_camera(1,0)/point_in_camera(2,0)+mCy;
    return Eigen::Vector2d(x,y);
}

Eigen::Vector3d Camera::pixel2camera(const Eigen::Vector2d &point_in_pixel,double depth)
{
    double x,y;
    x = (point_in_pixel(0,0)-mCx)*depth/mFx;
    y = (point_in_pixel(1,0)-mCy)*depth/mFy;
    return Eigen::Vector3d(x,y,depth);
}


