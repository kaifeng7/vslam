/*
 * @Author: fengkai 
 * @Date: 2019-06-25 10:35:48 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-07-19 18:48:12
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
    cv::Point2i center;//相对位置
    int width,height;
    std::string code_id;
    int card_id;
    Eigen::Vector3d pose; //绝对位置
    double thita;
    double r;
    int code2card(const std::string &str)
    {
        int sum = 0;
        for(int i=0;i<str.size();i++)
        {
            sum*=2;
            sum += (str.at(i)-'0');
        }
        return sum;
    }
};

struct ImagePart
{
    std::vector<Card> cards;
    Camera mCamera;
};

struct Image
{

    Eigen::Matrix<double,3,4> pose;//车的位姿
    int image_id;
    std::vector<ImagePart> image_parts;//0.left 1.front

    Image()
    {
        for(int i = 0;i<2;i++)
            image_parts.push_back(ImagePart());
    }
};



#endif