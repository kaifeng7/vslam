/*
 * @Author: fengkai 
 * @Date: 2019-06-24 10:01:31 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-06-25 10:00:23
 */
#ifndef CAMERA_H
#define CAMERA_H
#include <ros/ros.h>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

class Camera
{
public:
    Camera();
    Camera(const double &fx,const double &fy,const double &cx,const double &cy);
    ~Camera();

    Eigen::Vector3d world2camera(const Eigen::Vector3d &point_in_world);
    Eigen::Vector3d camera2world(const Eigen::Vector3d &point_in_camera);
    Eigen::Vector2d camera2pixel(const Eigen::Vector3d &point_in_camera);
    Eigen::Vector3d pixel2camera(const Eigen::Vector2d &point_in_pixel,double depth);
    Eigen::Vector2d world2pixel(const Eigen::Vector3d &point_in_world);
    Eigen::Vector3d pixel2world(const Eigen::Vector2d &point_in_pixel,double depth);
    cv::Mat mK; //内参矩阵
    double mFx, mFy, mCx, mCy;
    Sophus::SE3d mTwc;
    cv::Mat setK(const double &fx, const double &fy, const double &cx, const double &cy);
};

#endif