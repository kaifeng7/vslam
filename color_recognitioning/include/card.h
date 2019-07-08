/*
 * @Author: fengkai 
 * @Date: 2019-06-25 10:35:48 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-06-27 10:21:00
 */
#ifndef CARD_H
#define CARD_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include "camera.h"

struct Card
{
    std::vector<cv::Point2i> key_points;
    cv::Point2i center;
    int width,height;
    std::string code_id;
    Eigen::Vector3d pose;
};

struct Image
{
    std::vector<Card> card;
    Eigen::Matrix<double,3,4> pose;
    int image_id;
    Camera mCamera;
};



#endif