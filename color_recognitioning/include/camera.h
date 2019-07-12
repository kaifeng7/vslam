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

class CameraParam
{
public:
    cv::Mat mK; //内参矩阵
    cv::Mat mNewK;
    double fx, fy, cx, cy;
    cv::Mat mD; //畸变矩阵
    double k1, k2, k3, p1, p2;
    void setK();
    void setD();
};

class Camera
{
public:
    Camera();
    ~Camera();

    Eigen::Vector3d world2camera(const Eigen::Vector3d &point_in_world);
    Eigen::Vector3d camera2world(const Eigen::Vector3d &point_in_camera);
    cv::Point2i camera2pixel(const Eigen::Vector3d &point_in_camera, const CameraParam &camera_param);
    Eigen::Vector3d pixel2camera(const cv::Point2i &point_in_pixel, const CameraParam &camera_param, double depth);
    cv::Point2i world2pixel(const Eigen::Vector3d &point_in_world);
    Eigen::Vector3d pixel2world(const cv::Point2i &point_in_pixel, double depth);
    Sophus::SE3d mTwc;
};

#endif