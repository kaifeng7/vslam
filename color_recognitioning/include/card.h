/*
 * @Author: fengkai 
 * @Date: 2019-06-25 10:35:48 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-06-25 10:36:45
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
    std::vector<cv::KeyPoint> key_points;
    cv::KeyPoint center;
    std::string code_id;    
};

struct Image
{
    std::vector<Card> card;
    cv::Mat pose;
    int image_id;        
    Camera camera;

};


#endif