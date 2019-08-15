/*
 * @Author: fengkai 
 * @Date: 2019-06-21 16:10:30 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-06-24 22:11:31
 */

#include "card_detection.h"

VSlam::VSlam() : pnh_("~"), bImage(false), m_CountImageId(0), m_CountKeyFrameId(0)
{
    initROS();
}

VSlam::~VSlam()
{
}

void VSlam::initROS()
{

    pub_DetectedImageRviz = nh_.advertise<sensor_msgs::Image>("/detected_image_rviz", 1);

    sub_Image = nh_.subscribe("/usb_cam2/image_raw", 1, &VSlam::callbackGetImage, this);
}



void VSlam::callbackGetImage(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr pub_ptr;
    try
    {
        bImage = true;
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        pub_ptr = cv_ptr;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    m_CurrentImageMat[0] = cv_ptr->image;
}

void VSlam::MainLoop()
{
    ROS_INFO_STREAM("card_detection start");
    ros::Rate loop_rate(20);

    double max_area = 289;
    double min_area = 100;
    double width, height, angle;
    int iLowH = 100;
        int iHighH = 140;
    
        int iLowS = 90; 
        int iHighS = 255;
 
        int iLowV = 90;
        int iHighV = 255;
    while (ros::ok())
    {

        ros::spinOnce();
        if (bImage == false)
        {
            continue;
        }
        cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
        
 
        //Create trackbars in "Control" window
        cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
        cvCreateTrackbar("HighH", "Control", &iHighH, 179);
 
        cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
        cvCreateTrackbar("HighS", "Control", &iHighS, 255);
 
        cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
        cvCreateTrackbar("HighV", "Control", &iHighV, 255);
        cv::Mat imgHSV;
        cv::Mat imgPub;
        std::vector<cv::Mat> splitHSV;
        cv::cvtColor(m_CurrentImageMat[0], imgHSV, cv::COLOR_BGR2HSV);
        cv::split(imgHSV, splitHSV);
        cv::equalizeHist(splitHSV[2], splitHSV[2]); //直方图均衡化params::src dst
        cv::merge(splitHSV, imgHSV);

        cv::Mat imgThresholdedRed1,imgThresholdedRed2;
        cv::Mat imgThresholdedBlue;

        cv::inRange(imgHSV, cv::Scalar(168, 35, 32), cv::Scalar(180, 255, 255), imgThresholdedRed1);
        cv::inRange(imgHSV, cv::Scalar(0, 35, 32), cv::Scalar(10, 255, 255), imgThresholdedRed2);
        cv::inRange(imgHSV, cv::Scalar(100, 46, 32), cv::Scalar(140, 255, 255), imgThresholdedBlue);

        cv::Mat imgThresholdedRed = imgThresholdedRed1+imgThresholdedRed2;

        cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(imgThresholdedRed, imgThresholdedRed, cv::MORPH_CLOSE, element1);
        
        cv::morphologyEx(imgThresholdedBlue, imgThresholdedBlue, cv::MORPH_CLOSE, element1);

        cv::Mat imgThresholded = imgThresholdedBlue+imgThresholdedRed;
        //pub_ptr->image = imgPub;

        //闭操作 (连接一些连通域)
        cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
        cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_CLOSE, element2);

        //开操作 (去除一些噪点)
        //cv::Mat element3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
        //cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_OPEN, element3);
        //cv::cvtColor(imgThresholded, imgPub, cv::COLOR_HSV2BGR);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(imgThresholded, contours, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        //cv::imshow("Control", imgThresholded); //show the thresholded image
        //cv::imshow("imgThresholdedBlue", imgThresholdedBlue); //show the thresholded image
        cv::imshow("imgThresholded", imgThresholded); //show the thresholded image
        cv::imshow("MAT", m_CurrentImageMat[0]); //show the thresholded image

        //cv::circle(cv_ptr->image,cv::Point(50,50),10,CV_RGB(255,0,0));
        //pub_DetectedImageRviz.publish(pub_ptr->toImageMsg());
        cv::waitKey(1);
        //loop_rate.sleep();
    }
}
