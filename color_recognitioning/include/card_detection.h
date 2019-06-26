/*
 * @Author: fengkai 
 * @Date: 2019-06-21 16:10:32 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-06-25 22:38:52
 */

#ifndef CARD_DETECTION_H
#define CARD_DETECTION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>


#include "camera.h"
#include "card.h"
#include "map.h"

class MapPoint;
class KeyFrame;
class SlamMap;

struct DetectionParam
{
    double max_area;
    double min_area;

};

class CardDetection
{
    public:
        CardDetection();
        ~CardDetection();

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;

        ros::Subscriber sub_Image;
        ros::Subscriber sub_Odom;

        ros::Publisher pub_MarkerRviz;
        ros::Publisher pub_DetectedImageRviz;
        ros::Publisher pub_CardRviz;

        bool bImage;
        bool bOdom;

        cv::Mat m_CurrentImageMat;
        Image m_CurrentImage;
        Image m_LastImage;
        
        int m_CountImageId;
        int m_CountKeyFrameId;
        DetectionParam mParam;//参数

        SlamMap mMap;

        void callbackGetImage(const sensor_msgs::ImageConstPtr &msg);
        void callbackGetOdom(const nav_msgs::OdometryConstPtr &msg);
        void triangulation(const Image& image1,const Image& image2,std::vector<MapPoint> &map_points);
        void VisCardInWorld(visualization_msgs::MarkerArray &markers);
       
        void initROS();
        void MainLoop();

};

#endif