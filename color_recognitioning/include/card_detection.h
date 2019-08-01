/*
 * @Author: fengkai 
 * @Date: 2019-06-21 16:10:32 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-08-01 15:24:25
 */

#ifndef CARD_DETECTION_H
#define CARD_DETECTION_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

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
#include <tf/transform_broadcaster.h>
#include<tf/transform_listener.h>
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
struct TFParam
{    
    std::string map_tf;
    std::string base_tf;
    std::string camera1_tf;
    std::string camera2_tf;
};

class VSlam
{
    public:
        VSlam();
        ~VSlam();

        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> SyncPolicy;
        message_filters::Subscriber<sensor_msgs::Image> *sub_ImageLeft;
        message_filters::Subscriber<sensor_msgs::Image> *sub_ImageFront;
        message_filters::Synchronizer<SyncPolicy> *sync; 
        ros::Subscriber sub_Image;
        ros::Subscriber sub_Odom;

        ros::Publisher pub_CardRviz;
        ros::Publisher pub_IdRviz;
        ros::Publisher pub_DetectedImageRviz;
        ros::Publisher pub_CameraRviz;
        ros::Publisher pub_PointRviz;
        ros::Publisher pub_LoopCameraRviz;
        ros::Publisher pub_LoopCardRviz;

        ros::Publisher pub_slam;

        tf::TransformBroadcaster tf_Broadcaster;
        tf::TransformListener tf_Listener;
        tf::StampedTransform tf_base_to_camera1;
        tf::StampedTransform tf_base_to_camera2;
        tf::Transform tf_map_to_base;

        tf::Transform tf_map_to_camera1;
        tf::Transform tf_map_to_camera2;
        
        ros::Time time;


        bool bImage;
        bool bOdom;
        bool bInit;//初始化
        bool bTF;

        std::vector<int> mCountCards;


        cv::Mat m_CurrentImageMat[2];
        cv::Mat m_ImageRawMat[2];
        Image m_CurrentImage;
        Image m_LastImage;
        geometry_msgs::Pose m_CurrentPose;

        KeyFrame *mp_CurrentKeyFrame;
        KeyFrame *mp_RefKeyFrame;
        
        int m_CountImageId;
        int m_CountKeyFrameId;
        DetectionParam mParam;//参数
        CameraParam mCameraParamLeft;//相机参数
        CameraParam mCameraParamFront;//相机参数
        TFParam mTFParam;

        Map mMap;

        void callbackGetImage(const sensor_msgs::ImageConstPtr &msg_left,const sensor_msgs::ImageConstPtr &msg_front);
    
        void callbackGetOdom(const nav_msgs::OdometryConstPtr &msg);
        void triangulation(const Image& image1,const Image& image2,std::vector<MapPoint> &map_points);

        void Undistort(const cv::Mat &input,cv::Mat &output);

        double distance2(const double &x,const double &y);       
        void detectDistance(Image &image,const int &flag);
        void cardDetection(cv::Mat &mat, Image &image,const int &flag);

        void processPicture(cv::Mat &mat_cur,Image &image,const sensor_msgs::ImageConstPtr &msg,const int &flag);

        bool isKeyFrame(const Image &image);
        bool isMapPoint(const Card &card);
        void setKeyFrame(const Image &img,KeyFrame **pp_kf_cur,KeyFrame **pp_kf_ref);
        void setCameraPose(KeyFrame *mKF);
        void setMapPoint(KeyFrame *pKF,MapPoint **ppMp);
        void initROS();     
        
        void VisCardInWorld(visualization_msgs::Marker &marker,const MapPoint *mp);
        void VisIdInWorld(visualization_msgs::Marker &marker,const MapPoint *mp);
        void VisCameraWorld(visualization_msgs::Marker &marker,const KeyFrame *kf);
        void VisPointWorld(visualization_msgs::Marker &marker,const KeyFrame *kf);

        void VisLoopCameraWorld(visualization_msgs::MarkerArray &markers,const std::vector<geometry_msgs::Pose> &poses);
        void VisLoopCardWorld(visualization_msgs::MarkerArray &markers,const std::vector<geometry_msgs::Pose> &poses);

        void publishVslam(vslam::Viz &viz);
        
        void MainLoop();

};

#endif