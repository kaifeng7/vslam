/*
 * @Author: fengkai 
 * @Date: 2019-06-21 16:10:32 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-06-27 10:21:06
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
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <vslam/Viz.h>

#include "map.h"

class MapPoint;
class KeyFrame;
class SlamMap;
class Image;
class Card;

struct DetectionParam
{
    int max_area;
    int min_area;

    int max_red_h;
    int max_red_s;
    int max_red_v;
    int min_red_h;
    int min_red_s;
    int min_red_v;  

    int max_blue_h;
    int max_blue_s;
    int max_blue_v;
    int min_blue_h;
    int min_blue_s;
    int min_blue_v;  

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
        ros::Publisher pub_slam;

        bool bImage;
        bool bOdom;
        bool bInit;//初始化

        cv::Mat m_CurrentImageMat;
        Image m_CurrentImage;
        Image m_LastImage;
        geometry_msgs::Pose m_CurrentPose;

        KeyFrame m_CurrentKeyFrame;
        KeyFrame m_RefKeyFrame;
        
        int m_CountImageId;
        int m_CountKeyFrameId;
        DetectionParam mParam;//参数
        CameraParam mCameraParam;//相机参数

        SlamMap mMap;

        void callbackGetImage(const sensor_msgs::ImageConstPtr &msg);
        void callbackGetOdom(const nav_msgs::OdometryConstPtr &msg);
        void triangulation(const Image& image1,const Image& image2,std::vector<MapPoint> &map_points);
        void VisCardInWorld(visualization_msgs::MarkerArray &markers);
        void publishVslam(vslam::Viz &viz);
        void detectDistance(Image &image);
       
        void initROS();
        void MainLoop();

};

#endif