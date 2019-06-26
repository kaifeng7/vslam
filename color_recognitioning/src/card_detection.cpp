/*
 * @Author: fengkai 
 * @Date: 2019-06-21 16:10:30 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-06-25 22:37:38
 */

#include "card_detection.h"

CardDetection::CardDetection() : pnh_("~"), bImage(false), bOdom(false),m_CountImageId(0), m_CountKeyFrameId(0)
{
    initROS();
}

CardDetection::~CardDetection()
{
}

void CardDetection::initROS()
{

    pub_DetectedImageRviz = nh_.advertise<sensor_msgs::Image>("/detected_image_rviz", 1);
    pub_MarkerRviz = nh_.advertise<visualization_msgs::MarkerArray>("/marker_rviz", 10);

    sub_Image = nh_.subscribe("/usb_cam/image_raw", 1, &CardDetection::callbackGetImage, this);
    sub_Odom = nh_.subscribe("/odom/imu", 10, &CardDetection::callbackGetOdom, this);
    mParam.max_area = 289;
    mParam.min_area = 100;
}

void CardDetection::callbackGetOdom(const nav_msgs::OdometryConstPtr &msg)
{
    bOdom = true;
    geometry_msgs::Pose pose = msg->pose.pose;

    Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    Eigen::Vector3d t(pose.position.x, pose.position.y, pose.position.z);
    Sophus::SE3d se3(quat, t);
    m_CurrentImage.camera.mTwc = se3;

    Eigen::Matrix3d r = quat.toRotationMatrix();
    m_CurrentImage.pose = (cv::Mat_<double>(3, 4) << r(0, 0), r(0, 1), r(0, 2), t(0, 0),
                           r(1, 0), r(1, 1), r(1, 2), t(1, 0),
                           r(2, 0), r(2, 1), r(2, 2), t(2, 0));
}

void CardDetection::callbackGetImage(const sensor_msgs::ImageConstPtr &msg)
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
    m_CurrentImageMat = cv_ptr->image;
    cv::Mat imgHSV;
    cv::Mat imgPub;
    std::vector<cv::Mat> splitHSV;
    cv::cvtColor(m_CurrentImageMat, imgHSV, cv::COLOR_BGR2HSV);
    cv::split(imgHSV, splitHSV);
    cv::equalizeHist(splitHSV[2], splitHSV[2]); //直方图均衡化params::src dst
    cv::merge(splitHSV, imgHSV);

    cv::Mat imgThresholdedYellow;
    cv::Mat imgThresholdedBlue;
    //cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
    cv::inRange(imgHSV, cv::Scalar(0, 50, 131), cv::Scalar(31, 90, 255), imgThresholdedYellow);
    cv::inRange(imgHSV, cv::Scalar(100, 169, 30), cv::Scalar(140, 255, 255), imgThresholdedBlue);
    cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
    cv::morphologyEx(imgThresholdedYellow, imgThresholdedYellow, cv::MORPH_CLOSE, element1);
    cv::morphologyEx(imgThresholdedBlue, imgThresholdedBlue, cv::MORPH_CLOSE, element1);

    cv::Mat imgThresholded;
    imgThresholded = imgThresholdedBlue + imgThresholdedYellow;

    //闭操作 (连接一些连通域)
    cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_CLOSE, element2);

    //开操作 (去除一些噪点)
    //cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_OPEN, element);
    //cv::cvtColor(imgThresholded, imgPub, cv::COLOR_HSV2BGR);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(imgThresholded, contours, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (int i = 0; i < contours.size(); i++)
    {
        double area = cv::contourArea(contours[i]);
        if (area >= mParam.min_area && area <= mParam.max_area)
        {
            cv::RotatedRect rotated_rect;
            rotated_rect = cv::minAreaRect(contours[i]);
            cv::Rect rect;
            rect = rotated_rect.boundingRect();


            if (rect.width >= rect.height)
                continue;

            std::string code_id;
            bool flag;
            int count_yellow, count_blue, count_non;
            if ((double)rect.height / rect.width < 12 && (double)rect.height / rect.width > 3)
            {

                for (int j = 0; j < 7; j++)
                {
                    flag = false;
                    count_blue = 0;
                    count_yellow = 0;
                    count_non = 0;
                    for (int k = 0; k < rect.width; k++)
                    {
                        int x = (int)(rect.x + k);
                        int y = (int)(rect.y + rect.height / 14 * (j * 2 + 1));
                        cv::circle(m_CurrentImageMat, cv::Point(x, y), 2, cv::Scalar(255, 255, 0));
                        if (imgThresholdedBlue.at<uchar>(y, x) == 255)
                        {
                            count_blue++;
                        }
                        else if (imgThresholdedYellow.at<uchar>(y, x) == 255)
                        {
                            count_yellow++;
                        }
                        else
                        {
                            count_non++;
                        }

                        y = (int)(rect.y + rect.height / 14 * (j * 2 + 1) - 1);
                        if (imgThresholdedBlue.at<uchar>(y, x) == 255)
                        {
                            count_blue++;
                        }
                        else if (imgThresholdedYellow.at<uchar>(y, x) == 255)
                        {
                            count_yellow++;
                        }
                        else
                        {
                            count_non++;
                        }

                        y = (int)(rect.y + rect.height / 14 * (j * 2 + 1) + 1);
                        if (imgThresholdedBlue.at<uchar>(y, x) == 255)
                        {
                            count_blue++;
                        }
                        else if (imgThresholdedYellow.at<uchar>(y, x) == 255)
                        {
                            count_yellow++;
                        }
                        else
                        {
                            count_non++;
                        }

                        y = (int)(rect.y + rect.height / 14 * (j * 2 + 1) - 2);
                        if (imgThresholdedBlue.at<uchar>(y, x) == 255)
                        {
                            count_blue++;
                        }
                        else if (imgThresholdedYellow.at<uchar>(y, x) == 255)
                        {
                            count_yellow++;
                        }
                        else
                        {
                            count_non++;
                        }

                        y = (int)(rect.y + rect.height / 14 * (j * 2 + 1) + 2);
                        if (imgThresholdedBlue.at<uchar>(y, x) == 255)
                        {
                            count_blue++;
                        }
                        else if (imgThresholdedYellow.at<uchar>(y, x) == 255)
                        {
                            count_yellow++;
                        }
                        else
                        {
                            count_non++;
                        }
                    }
                    if (count_blue > count_yellow && count_blue > 0)
                    {
                        code_id.push_back('0');
                    }
                    else if (count_yellow > count_blue && count_yellow > 0)
                    {
                        code_id.push_back('1');
                    }
                    else
                    {
                        code_id.push_back('2');
                    }

                //ROS_INFO_STREAM("count_blue:" << count_blue << "count_yellow:" << count_yellow << "count_non" << count_non);
                }
                if (code_id == "1010101" || code_id == "0101010" || code_id == "0101000" ||
                    code_id == "0001010" || code_id == "0100010" || code_id == "0101110")
                {
                    ROS_INFO_STREAM(code_id);
                    m_CurrentImage.image_id = m_CountImageId++;
                    Card card;
                    card.code_id = code_id;
                    card.center.pt = rotated_rect.center;
                    cv::KeyPoint lt, rt, lb, rb;
                    lt.pt = cv::Point2i(rect.x, rect.y);
                    card.key_points.push_back(lt);

                    rt.pt = cv::Point2i(rect.x + rect.width, rect.y);
                    card.key_points.push_back(rt);

                    lb.pt = cv::Point2i(rect.x, rect.y + rect.height);
                    card.key_points.push_back(lb);

                    rb.pt = cv::Point2i(rect.x + rect.width, rect.y + rect.height);
                    card.key_points.push_back(rb);
                    m_CurrentImage.card.push_back(card);
                    m_CurrentImage.image_id = m_CountImageId++;
                    cv::rectangle(m_CurrentImageMat, rect, cv::Scalar(0, 0, 255));
                }
            }
            //cv::drawContours(m_CurrentImageMat,contours,i,cv::Scalar(0,0,255)); // 轮廓的颜色);
        }
    }
    
    pub_ptr->image = m_CurrentImageMat;
    pub_DetectedImageRviz.publish(pub_ptr->toImageMsg());
}

void CardDetection::VisCardInWorld(visualization_msgs::MarkerArray &markers)
{
    std::vector<MapPoint> map_points = mMap.mMapPoints;
    visualization_msgs::Marker marker;
    for(int i =0;i<map_points.size();i++)
    {
        geometry_msgs::Pose pose;
        cv::Mat mat = map_points.at(i).mWorldPositionMat;
        Eigen::Matrix3d eigen;        
        eigen << mat.at<double>(0,0), mat.at<double>(0,1), mat.at<double>(0,2), 
                 mat.at<double>(1,0), mat.at<double>(1,1), mat.at<double>(1,2), 
                 mat.at<double>(2,0), mat.at<double>(2,1), mat.at<double>(2,2);

        pose.position.x = mat.at<double>(0,3);
        pose.position.y = mat.at<double>(1,3);
        pose.position.z = mat.at<double>(2,3);

        Eigen::Quaterniond q(eigen);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        marker.pose = pose;
        marker.header.stamp = ros::Time();
        marker.header.frame_id = "map";
        marker.ns = "Card_Position";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 1;
        marker.scale.y = 1;
        marker.scale.z = 1;

        marker.color.a = 1;
        marker.color.b = 1;
        marker.color.g = 1;
        marker.color.r = 1;
        marker.frame_locked = false;

        markers.markers.push_back(marker);
    }
}

void CardDetection::triangulation(const Image &image1, const Image &image2, std::vector<MapPoint> &map_points)
{
    std::vector<cv::Point2d> points1, points2;
    for (int i = 0; i < image1.card.size(); i++)
    {
        for (int j = 0; j < image2.card.size(); j++)
        {
            if (image1.card.at(i).code_id == image2.card.at(j).code_id)
            {
                Eigen::Vector3d point_in_picture1, point_in_picture2; //图像坐标系下
                point_in_picture1 = m_CurrentImage.camera.pixel2camera(Eigen::Vector2d(image1.card.at(i).center.pt.x, image1.card.at(i).center.pt.y), 1);
                point_in_picture2 = m_CurrentImage.camera.pixel2camera(Eigen::Vector2d(image2.card.at(j).center.pt.x, image2.card.at(j).center.pt.y), 1);

                points1.push_back(cv::Point2d(point_in_picture1(0), point_in_picture1(1)));
                points2.push_back(cv::Point2d(point_in_picture2(0), point_in_picture2(1)));
                MapPoint map_point;
                map_point.mCard = image2.card.at(j);
                map_points.push_back(map_point);
            }
        }
    }
    cv::Mat point_4d;
    cv::triangulatePoints(image1.pose, image2.pose, points1, points2, point_4d);

    for (int i = 0; i < point_4d.cols; i++)
    {
        cv::Mat x = point_4d.col(i);
        x / x.at<double>(3, 0);
        cv::Point3d p(x.at<double>(0, 0), x.at<double>(1, 0), x.at<double>(2, 0));
        map_points.at(i).mCameraPositionPoint = p;
    }
}

void CardDetection::MainLoop()
{
    ROS_INFO_STREAM("card_detection start");
    ros::Rate loop_rate(20);

    while (ros::ok())
    {

        ros::spinOnce();
        if (bImage == false)
        {
            ROS_WARN_STREAM("wait Image msg...");
            continue;
        }
        if (bOdom == false)
        {
            ROS_WARN_STREAM("wait Odom msg...");
            continue;
        }

        if(m_CurrentImage.card.size()>0&&m_CurrentImage.image_id == 0)
        {
            m_LastImage = m_CurrentImage;
        }
        else
        {
            std::vector<MapPoint> map_points;
            triangulation(m_LastImage,m_CurrentImage,map_points);
            if(map_points.size()>0)
            {
                KeyFrame key_frame(m_CurrentImage,m_CountKeyFrameId++);
                key_frame.mMapPoints = map_points;
                mMap.mKeyFrames.push_back(key_frame);
                for(int i = 0;i<map_points.size();i++)
                    mMap.mMapPoints.push_back(map_points.at(i));
            }
            m_LastImage = m_CurrentImage;
        }

        visualization_msgs::MarkerArray markers;
        VisCardInWorld(markers);
        pub_MarkerRviz.publish(markers);

        
        loop_rate.sleep();
    }
}
