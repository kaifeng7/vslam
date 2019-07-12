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
    int card_id;
    Eigen::Vector3d pose;
    int code2card(const std::string &str)
    {
        int sum = 0;
        for(int i=0;i<str.size()-1;i++)
        {
            sum*=2;
            sum += (str.at(i)-'0');
        }
        return card_id;
    }
};

struct Image
{
    std::vector<Card> card;
    Eigen::Matrix<double,3,4> pose;
    int image_id;
    Camera mCamera;
};



#endif