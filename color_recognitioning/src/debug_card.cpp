/*
 * @Author: fengkai 
 * @Date: 2019-06-21 16:10:30 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-06-24 22:11:31
 */

#include "card_detection.h"

CardDetection::CardDetection() : pnh_("~"), bImage(false), m_CountImageId(0), m_CountKeyFrameId(0)
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

    sub_Image = nh_.subscribe("/camera/image_raw", 1, &CardDetection::callbackGetImage, this);
    //sub_Odom = nh_.subscribe("/odom/imu", 10, &CardDetection::callbackGetOdom, this);
}

// void CardDetection::callbackGetOdom(const nav_msgs::OdometryConstPtr &msg)
// {
//     geometry_msgs::Pose pose = msg->pose.pose;

//     Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
//     Eigen::Vector3d t(pose.position.x, pose.position.y, pose.position.z);
//     Sophus::SE3d se3(quat, t);
//     m_CurrentImage.camera.mTwc = se3;

//     Eigen::Matrix3d r = quat.toRotationMatrix();
//     m_CurrentImage.pose = (cv::Mat_<double>(3, 4) << r(0, 0), r(0, 1), r(0, 2), t(0, 0),
//                            r(1, 0), r(1, 1), r(1, 2), t(1, 0),
//                            r(2, 0), r(2, 1), r(2, 2), t(2, 0));
// }

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
}

// void CardDetection::triangulation(const Image &image1, const Image &image2, std::vector<cv::Point3d> &points)
// {
//     std::vector<cv::Point2d> points1, points2;
//     for (int i = 0; i < image1.card.size(); i++)
//     {
//         for (int j = 0; j < image2.card.size(); j++)
//         {
//             if (image1.card.at(i).code_id == image2.card.at(j).code_id)
//             {
//                 Eigen::Vector3d point_in_picture1, point_in_picture2; //图像坐标系下
//                 point_in_picture1 = m_CurrentImage.camera.pixel2camera(Eigen::Vector2d(image1.card.at(i).center.pt.x, image1.card.at(i).center.pt.y), 1);
//                 point_in_picture2 = m_CurrentImage.camera.pixel2camera(Eigen::Vector2d(image2.card.at(j).center.pt.x, image2.card.at(j).center.pt.y), 1);

//                 points1.push_back(cv::Point2d(point_in_picture1(0), point_in_picture1(1)));
//                 points2.push_back(cv::Point2d(point_in_picture2(0), point_in_picture2(1)));
//             }
//         }
//     }
//     cv::Mat point_4d;
//     cv::triangulatePoints(image1.pose, image2.pose, points1, points2, point_4d);

//     for (int i = 0; i < point_4d.cols; i++)
//     {
//         cv::Mat x = point_4d.col(i);
//         x / x.at<double>(3, 0);
//         cv::Point3d p(x.at<double>(0, 0), x.at<double>(1, 0), x.at<double>(2, 0));
//         points.push_back(p);
//     }
// }

void CardDetection::MainLoop()
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
        cv::cvtColor(m_CurrentImageMat, imgHSV, cv::COLOR_BGR2HSV);
        cv::split(imgHSV, splitHSV);
        cv::equalizeHist(splitHSV[2], splitHSV[2]); //直方图均衡化params::src dst
        cv::merge(splitHSV, imgHSV);

        cv::Mat imgThresholdedRed;
        cv::Mat imgThresholdedBlue;

        cv::inRange(imgHSV, cv::Scalar(156, 90, 90), cv::Scalar(180, 255, 255), imgThresholdedRed);
        cv::inRange(imgHSV, cv::Scalar(100, 125, 32), cv::Scalar(140, 255, 255), imgThresholdedBlue);
        cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(imgThresholdedRed, imgThresholdedRed, cv::MORPH_CLOSE, element1);
        cv::morphologyEx(imgThresholdedBlue, imgThresholdedBlue, cv::MORPH_CLOSE, element1);

        cv::Mat imgThresholded;
        imgThresholded = imgThresholdedBlue+imgThresholdedRed;
        //pub_ptr->image = imgPub;

        //闭操作 (连接一些连通域)
        cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
        cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_CLOSE, element2);

        //开操作 (去除一些噪点)
        cv::Mat element3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
        cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_OPEN, element3);
        //cv::cvtColor(imgThresholded, imgPub, cv::COLOR_HSV2BGR);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(imgThresholded, contours, CV_RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // for (int i = 0; i < contours.size(); i++)
        // {
        //     double area = cv::contourArea(contours[i]);
        //     if (area >= min_area && area <= max_area)
        //     {
        //         cv::RotatedRect rotated_rect;
        //         rotated_rect = cv::minAreaRect(contours[i]);
        //         cv::Rect rect;
        //         rect = rotated_rect.boundingRect();
        //         //if((rect.br().x-rect.tl().x)>(rect.br().y-rect.tl().y))
        //         //    break;

        //         angle = rotated_rect.angle;
        //         if (rect.width < rect.height)
        //         {
        //             width = rect.width;
        //             height = rect.height;
        //         }
        //         else
        //         {
        //             continue;
        //         }

        //         std::string code_id;
        //         bool flag;
        //         int count_yellow, count_blue, count_non;
        //         if (height / width < 12 && height / width > 3)
        //         {

        //             for (int j = 0; j < 7; j++)
        //             {
        //                 flag = false;
        //                 count_blue = 0;
        //                 count_yellow = 0;
        //                 count_non = 0;
        //                 for (int k = 0; k < width; k++)
        //                 {
        //                     int x = (int)(rect.x + k);
        //                     int y = (int)(rect.y + height / 14 * (j * 2 + 1));
        //                     cv::circle(m_CurrentImageMat, cv::Point(x, y), 2, cv::Scalar(255, 255, 0));
        //                     if (imgThresholdedBlue.at<uchar>(y, x) == 255)
        //                     {
        //                         //code_id.push_back('0');
        //                         count_blue++;
        //                     }
        //                     else if (imgThresholdedYellow.at<uchar>(y, x) == 255)
        //                     {
        //                         //code_id.push_back('1');
        //                         count_yellow++;
        //                     }
        //                     else
        //                     {
        //                         count_non++;
        //                     }

        //                     y = (int)(rect.y + height / 14 * (j * 2 + 1) - 1);
        //                     if (imgThresholdedBlue.at<uchar>(y, x) == 255)
        //                     {
        //                         //code_id.push_back('0');
        //                         count_blue++;
        //                     }
        //                     else if (imgThresholdedYellow.at<uchar>(y, x) == 255)
        //                     {
        //                         //code_id.push_back('1');
        //                         count_yellow++;
        //                     }
        //                     else
        //                     {
        //                         count_non++;
        //                     }

        //                     y = (int)(rect.y + height / 14 * (j * 2 + 1) + 1);
        //                     if (imgThresholdedBlue.at<uchar>(y, x) == 255)
        //                     {
        //                         //code_id.push_back('0');
        //                         count_blue++;
        //                     }
        //                     else if (imgThresholdedYellow.at<uchar>(y, x) == 255)
        //                     {
        //                         //code_id.push_back('1');
        //                         count_yellow++;
        //                     }
        //                     else
        //                     {
        //                         count_non++;
        //                     }

        //                     y = (int)(rect.y + height / 14 * (j * 2 + 1) - 2);
        //                     if (imgThresholdedBlue.at<uchar>(y, x) == 255)
        //                     {
        //                         //code_id.push_back('0');
        //                         count_blue++;
        //                     }
        //                     else if (imgThresholdedYellow.at<uchar>(y, x) == 255)
        //                     {
        //                         //code_id.push_back('1');
        //                         count_yellow++;
        //                     }
        //                     else
        //                     {
        //                         count_non++;
        //                     }

        //                     y = (int)(rect.y + height / 14 * (j * 2 + 1) + 2);
        //                     if (imgThresholdedBlue.at<uchar>(y, x) == 255)
        //                     {
        //                         //code_id.push_back('0');
        //                         count_blue++;
        //                     }
        //                     else if (imgThresholdedYellow.at<uchar>(y, x) == 255)
        //                     {
        //                         //code_id.push_back('1');
        //                         count_yellow++;
        //                     }
        //                     else
        //                     {
        //                         count_non++;
        //                     }
        //                 }
        //                 if (count_blue > count_yellow && count_blue > 0)
        //                 {
        //                     code_id.push_back('0');
        //                 }
        //                 else if (count_yellow > count_blue && count_yellow > 0)
        //                 {
        //                     code_id.push_back('1');
        //                 }
        //                 else
        //                 {
        //                     code_id.push_back('2');
        //                 }

        //                 //ROS_INFO_STREAM("count_blue:" << count_blue << "count_yellow:" << count_yellow << "count_non" << count_non);
        //             }
        //             // if (code_id == "1010101" || code_id == "0101010" || code_id == "0101000" || 
        //             //     code_id == "0001010" || code_id == "0100010" || code_id == "0101110")
        //             {
        //                 ROS_INFO_STREAM(code_id);
        //                 m_CurrentImage.image_id = m_CountImageId++;
        //                 Card card;
        //                 card.code_id = code_id;
        //                 card.center.pt = rotated_rect.center;
        //                 cv::KeyPoint lt, rt, lb, rb;
        //                 lt.pt = cv::Point2i(rect.x, rect.y);
        //                 card.key_points.push_back(lt);

        //                 rt.pt = cv::Point2i(rect.x + rect.width, rect.y);
        //                 card.key_points.push_back(rt);

        //                 lb.pt = cv::Point2i(rect.x, rect.y + rect.height);
        //                 card.key_points.push_back(lb);

        //                 rb.pt = cv::Point2i(rect.x + rect.width, rect.y + rect.height);
        //                 card.key_points.push_back(rb);
        //                 m_CurrentImage.card.push_back(card);
        //                 cv::rectangle(m_CurrentImageMat, rect, cv::Scalar(0, 0, 255));
        //             }
        //         }
        //         //cv::drawContours(m_CurrentImageMat,contours,i,cv::Scalar(0,0,255)); // 轮廓的颜色);
        //     }
        // }

        cv::imshow("Control", imgThresholded); //show the thresholded image
        //cv::imshow("imgThresholdedBlue", imgThresholdedBlue); //show the thresholded image
        //cv::imshow("imgThresholdedYellow", imgThresholdedYellow); //show the thresholded image

        //cv::circle(cv_ptr->image,cv::Point(50,50),10,CV_RGB(255,0,0));
        //pub_DetectedImageRviz.publish(pub_ptr->toImageMsg());
        cv::waitKey(1);
        loop_rate.sleep();
    }
}
