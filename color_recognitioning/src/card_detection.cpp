/*
 * @Author: fengkai 
 * @Date: 2019-06-21 16:10:30 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-06-25 22:37:38
 */

#include "card_detection.h"

CardDetection::CardDetection() : pnh_("~"), bImage(false), bOdom(false), bInit(false), m_CountImageId(0), m_CountKeyFrameId(0)
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
    pub_slam = nh_.advertise<vslam::Viz>("/slam_viz", 10);

    sub_Image = nh_.subscribe("/camera/image_raw", 1, &CardDetection::callbackGetImage, this);
    sub_Odom = nh_.subscribe("/odom/imu", 10, &CardDetection::callbackGetOdom, this);

    pnh_.param<int>("maxRectArea", mParam.max_area, 0);
    pnh_.param<int>("minRectArea", mParam.min_area, 0);

    pnh_.param<double>("fx", mCameraParam.fx, 0);
    pnh_.param<double>("fy", mCameraParam.fy, 0);
    pnh_.param<double>("cx", mCameraParam.cx, 0);
    pnh_.param<double>("cy", mCameraParam.cy, 0);
    pnh_.param<double>("k1", mCameraParam.k1, 0);
    pnh_.param<double>("k2", mCameraParam.k2, 0);
    pnh_.param<double>("k3", mCameraParam.k3, 0);
    pnh_.param<double>("p1", mCameraParam.p1, 0);
    pnh_.param<double>("p2", mCameraParam.p2, 0);


    pnh_.param<int>("maxRedH", mParam.max_red_h, 0);
    pnh_.param<int>("minRedH", mParam.min_red_h, 0);
    pnh_.param<int>("maxRedS", mParam.max_red_s, 0);
    pnh_.param<int>("minRedS", mParam.min_red_s, 0);
    pnh_.param<int>("maxRedV", mParam.max_red_v, 0);
    pnh_.param<int>("minRedV", mParam.min_red_v, 0);

    pnh_.param<int>("maxBlueH", mParam.max_blue_h, 0);
    pnh_.param<int>("minBlueH", mParam.min_blue_h, 0);
    pnh_.param<int>("maxBlueS", mParam.max_blue_s, 0);
    pnh_.param<int>("minBlueS", mParam.min_blue_s, 0);
    pnh_.param<int>("maxBlueV", mParam.max_blue_v, 0);
    pnh_.param<int>("minBlueV", mParam.min_blue_v, 0);

    mCameraParam.setK();
    mCameraParam.setD();
}

void CardDetection::callbackGetOdom(const nav_msgs::OdometryConstPtr &msg)
{
    bOdom = true;
    m_CurrentPose = msg->pose.pose;
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

    Undistort(m_CurrentImageMat,image);

    Eigen::Quaterniond quat(m_CurrentPose.orientation.w, m_CurrentPose.orientation.x, m_CurrentPose.orientation.y, m_CurrentPose.orientation.z);
    Eigen::Vector3d t(m_CurrentPose.position.x, m_CurrentPose.position.y, m_CurrentPose.position.z);
    //Sophus::SE3d se3(quat, t);
    //m_CurrentImage.mCamera.mTwc = se3;

    Eigen::Matrix3d r = quat.toRotationMatrix();
    m_CurrentImage.pose << r(0, 0), r(0, 1), r(0, 2), t(0, 0),
        r(1, 0), r(1, 1), r(1, 2), t(1, 0),
        r(2, 0), r(2, 1), r(2, 2), t(2, 0);
    m_CurrentImage.image_id = m_CountImageId++;
    m_CurrentImage.card.clear();
    cv::Mat imgHSV;
    cv::Mat imgPub;
    std::vector<cv::Mat> splitHSV;
    cv::cvtColor(m_CurrentImageMat, imgHSV, cv::COLOR_BGR2HSV);
    cv::split(imgHSV, splitHSV);
    cv::equalizeHist(splitHSV[2], splitHSV[2]); //直方图均衡化params::src dst
    cv::merge(splitHSV, imgHSV);

    cv::Mat imgThresholdedRed;
    cv::Mat imgThresholdedBlue;
    cv::inRange(imgHSV, cv::Scalar(mParam.min_red_h, mParam.min_red_s, mParam.min_red_v), cv::Scalar(mParam.max_red_h, mParam.max_red_s, mParam.max_red_v), imgThresholdedRed);
    cv::inRange(imgHSV, cv::Scalar(mParam.min_blue_h, mParam.min_blue_s, mParam.min_blue_v), cv::Scalar(mParam.max_blue_h, mParam.max_blue_s, mParam.max_blue_v), imgThresholdedBlue);
    cv::Mat element1 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
    cv::morphologyEx(imgThresholdedRed, imgThresholdedRed, cv::MORPH_CLOSE, element1);
    cv::morphologyEx(imgThresholdedBlue, imgThresholdedBlue, cv::MORPH_CLOSE, element1);

    cv::Mat imgThresholded;
    imgThresholded = imgThresholdedBlue + imgThresholdedRed;

    //闭操作 (连接一些连通域)
    cv::Mat element2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(7, 7));
    cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_CLOSE, element2);

    //开操作 (去除一些噪点)
    cv::Mat element3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
    cv::morphologyEx(imgThresholded, imgThresholded, cv::MORPH_OPEN, element3);
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
            int count_red, count_blue, count_non;

            if ((double)rect.height / rect.width < 9 && (double)rect.height / rect.width > 5)
            {
                bool flag_2 = false;

                for (int j = 0; j < 7; j++)
                {
                    count_blue = 0;
                    count_red = 0;
                    count_non = 0;
                    for (int k = 0; k < rect.width; k++)
                    {
                        int x = (int)(rect.x + k);
                        int y = (int)(rect.y + rect.height / 14.0 * (j * 2 + 1));
                        //cv::circle(m_CurrentImageMat, cv::Point(x, y), 2, cv::Scalar(255, 255, 0));
                        for (int p = -2; p <= 2; p++)
                        {
                            int dy = std::min(std::max(y + p, 0), 720);

                            if (imgThresholdedBlue.at<uchar>(y + p, x) == 255)
                            {
                                count_blue++;
                            }
                            if (imgThresholdedRed.at<uchar>(y + p, x) == 255)
                            {
                                count_red++;
                            }
                        }
                    }
                    if (count_blue > count_red + 5)
                    {
                        code_id.push_back('0');
                    }
                    else if (count_red > count_blue + 5)
                    {
                        code_id.push_back('1');
                    }
                    else
                    {
                        code_id.push_back('2');
                        flag_2 = true;
                    }

                    //ROS_INFO_STREAM("count_blue:" << count_blue << "count_red:" << count_red << "count_non" << count_non);
                }
                if (flag_2 == false && code_id != "1111111" && code_id != "0000000")
                {
                    ROS_INFO_STREAM(code_id);
                    Card card;
                    card.code_id = code_id;
                    card.width = rect.width;
                    card.height = rect.height;
                    card.center = rotated_rect.center;

                    cv::Point2i lt, rt, lb, rb;
                    lt = cv::Point2i(rect.x, rect.y);
                    card.key_points.push_back(lt);

                    rt = cv::Point2i(rect.x + rect.width, rect.y);
                    card.key_points.push_back(rt);

                    lb = cv::Point2i(rect.x, rect.y + rect.height);
                    card.key_points.push_back(lb);

                    rb = cv::Point2i(rect.x + rect.width, rect.y + rect.height);
                    card.key_points.push_back(rb);
                    m_CurrentImage.card.push_back(card);

                    cv::rectangle(m_CurrentImageMat, rect, cv::Scalar(0, 0, 255));
                    cv::putText(m_CurrentImageMat, code_id, card.center, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
                }
            }
        }
    }

    detectDistance(m_CurrentImage);
    if (m_CurrentImage.card.size() > 0)
    {
        bInit = true;
        m_CurrentKeyFrame.mImage = m_CurrentImage;
        m_CurrentKeyFrame.mKeyFrameId = m_CountKeyFrameId++;
    }
    pub_ptr->image = m_CurrentImageMat;
    //m_CurrentImageMat = imgThresholdedRed;
    pub_DetectedImageRviz.publish(pub_ptr->toImageMsg());
}

void CardDetection::VisCardInWorld(visualization_msgs::MarkerArray &markers)
{
    std::vector<MapPoint> map_points = mMap.mMapPoints;
    visualization_msgs::Marker marker;
    ROS_INFO_STREAM(map_points.size());
    for (int i = 0; i < map_points.size(); i++)
    {
        geometry_msgs::Pose pose;
        // cv::Mat mat = map_points.at(i).mWorldPositionMat;
        // Eigen::Matrix3d eigen;
        // eigen << mat.at<double>(0,0), mat.at<double>(0,1), mat.at<double>(0,2),
        //          mat.at<double>(1,0), mat.at<double>(1,1), mat.at<double>(1,2),
        //          mat.at<double>(2,0), mat.at<double>(2,1), mat.at<double>(2,2);

        // pose.position.x = mat.at<double>(0,3);
        // pose.position.y = mat.at<double>(1,3);
        // pose.position.z = mat.at<double>(2,3);

        // Eigen::Quaterniond q(eigen);
        // pose.orientation.x = q.x();
        // pose.orientation.y = q.y();
        // pose.orientation.z = q.z();
        // pose.orientation.w = q.w();

        pose.position.x = map_points.at(i).mWorldPositionPoint.x;
        pose.position.y = map_points.at(i).mWorldPositionPoint.y;
        pose.position.z = map_points.at(i).mWorldPositionPoint.z;

        marker.pose = pose;
        marker.header.stamp = ros::Time();
        marker.header.frame_id = "odom";
        marker.ns = "Card_Position";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        marker.color.a = 1;
        marker.color.b = 1;
        marker.color.g = 0;
        marker.color.r = 0;
        marker.frame_locked = false;

        markers.markers.push_back(marker);
    }
}

void CardDetection::detectDistance(Image &image)
{
    for(int i = 0;i<image.card.size();i++)
    {
        Card card = image.card.at(i);
        Eigen::Vector3d v[4];
        for(int j = 0;j<4;j++)
        {   
            cv::Point2d point(card.key_points.at(j).x-1.0,card.key_points.at(j).y-1.0);
            v[j] = image.mCamera.pixel2camera(point,mCameraParam,1);
        }
        double width,height;
        width = (v[1]-v[0]).x();
        height = (v[2]-v[0]).y();
        double depth = 0.7/height*1;
        
        cv::putText(m_CurrentImageMat, std::to_string(depth), card.key_points.at(3), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
        //ROS_INFO_STREAM(depth);
    }
}

void CardDetection::Undistort(const cv::Mat &input,cv::Mat &output)
{
    

    cv::Mat map1, map2;
    cv::Size imageSize;
    imageSize = input.size();
    initUndistortRectifyMap(mCameraParam.setK, mCameraParam.setD(), cv::Mat(),
    getOptimalNewCameraMatrix(mCameraParam.setK, mCameraParam.setD(), imageSize, 1, imageSize, 0),
    imageSize, CV_16SC2, map1, map2);
    cv::remap(input, output, map1, map2, cv::INTER_LINEAR);
}


// void CardDetection::triangulation(const Image &image1, const Image &image2, std::vector<MapPoint> &map_points)
// {
//     /* 方法一：不准 */
//     std::vector<cv::Point2d> points1, points2;
//     for (int i = 0; i < image1.card.size(); i++)
//     {
//         for (int j = 0; j < image2.card.size(); j++)
//         {
//             if (image1.card.at(i).code_id == image2.card.at(j).code_id)
//             {
//                 Eigen::Vector3d point_in_picture1, point_in_picture2; //图像坐标系下
//                 point_in_picture1 = m_CurrentImage.mCamera.pixel2camera(image1.card.at(i).center, 1);
//                 point_in_picture2 = m_CurrentImage.mCamera.pixel2camera(image2.card.at(j).center, 1);

//                 points1.push_back(cv::Point2d(point_in_picture1(0), point_in_picture1(1)));
//                 points2.push_back(cv::Point2d(point_in_picture2(0), point_in_picture2(1)));
//                 MapPoint map_point;
//                 map_point.mCard = image2.card.at(j);
//                 map_points.push_back(map_point);
//             }
//         }
//     }
//     cv::Mat point_4d;
//     cv::Mat pose1 = (cv::Mat_<double>(3, 4) << image1.pose(0, 0), image1.pose(0, 1), image1.pose(0, 2), image1.pose(0, 3),
//                      image1.pose(1, 0), image1.pose(1, 1), image1.pose(1, 2), image1.pose(1, 3),
//                      image1.pose(2, 0), image1.pose(2, 1), image1.pose(2, 2), image1.pose(2, 3));

//     cv::Mat pose2 = (cv::Mat_<double>(3, 4) << image2.pose(0, 0), image2.pose(0, 1), image2.pose(0, 2), image2.pose(0, 3),
//                      image2.pose(1, 0), image2.pose(1, 1), image2.pose(1, 2), image2.pose(1, 3),
//                      image2.pose(2, 0), image2.pose(2, 1), image2.pose(2, 2), image2.pose(2, 3));

//     cv::triangulatePoints(pose1, pose2, cv::Mat(points1, true), cv::Mat(points2, true), point_4d);

//     for (int i = 0; i < point_4d.cols; i++) //转换非齐次坐标
//     {
//         cv::Mat x = point_4d.col(i);
//         x / x.at<double>(3, 0);
//         cv::Point3d p(x.at<double>(0, 0), x.at<double>(1, 0), x.at<double>(2, 0));
//         map_points.at(i).mCameraPositionPoint = p;
//     }

//     //方法二：参考vis
//     // for (int i = 0; i < image1.card.size(); i++)
//     // {
//     //     for (int j = 0; j < image2.card.size(); j++)
//     //     {
//     //         if (image1.card.at(i).code_id == image2.card.at(j).code_id)
//     //         {
//     //             Eigen::Vector3d point_in_picture1, point_in_picture2; //图像坐标系下
//     //             point_in_picture1 = m_CurrentImage.mCamera.pixel2camera(Eigen::Vector2d(image1.card.at(i).center.pt.x, image1.card.at(i).center.pt.y), 1);
//     //             point_in_picture2 = m_CurrentImage.mCamera.pixel2camera(Eigen::Vector2d(image2.card.at(j).center.pt.x, image2.card.at(j).center.pt.y), 1);

//     //             Eigen::Vector2d point1, point2;
//     //             point1 = Eigen::Vector2d(point_in_picture1(0), point_in_picture1(1));
//     //             point2 = Eigen::Vector2d(point_in_picture2(0), point_in_picture2(1));
//     //             MapPoint map_point;
//     //             map_point.mCard = image2.card.at(j);
//     //             Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
//     //             design_matrix.row(0) = point1[0] * image1.pose.row(2) - image1.pose.row(0);
//     //             design_matrix.row(1) = point1[1] * image1.pose.row(2) - image1.pose.row(1);
//     //             design_matrix.row(2) = point2[0] * image2.pose.row(2) - image2.pose.row(0);
//     //             design_matrix.row(3) = point2[1] * image2.pose.row(2) - image2.pose.row(1);
//     //             Eigen::Vector4d triangulated_point;
//     //             triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
//     //             cv::Point3d point_3d(triangulated_point(0) / triangulated_point(3),
//     //                                  triangulated_point(1) / triangulated_point(3),
//     //                                  triangulated_point(2) / triangulated_point(3));
//     //             map_point.mCameraPositionPoint = point_3d;
//     //             map_points.push_back(map_point);
//     //         }
//     //     }
//     // }
// }

void CardDetection::publishVslam(vslam::Viz &viz)
{
    viz.header.frame_id = "odom";
    viz.header.stamp = ros::Time::now();
    vslam::Camera camera;
    vslam::Card card;
    camera.camera_pose = m_CurrentPose;
    viz.camera = camera;
    for (int i = 0; i < mMap.mMapPoints.size(); i++)
    {
        card.card_pose.position.x = mMap.mMapPoints.at(i).mWorldPositionPoint.x;
        card.card_pose.position.y = mMap.mMapPoints.at(i).mWorldPositionPoint.y;
        card.card_pose.position.z = mMap.mMapPoints.at(i).mWorldPositionPoint.z;
        card.code_id = mMap.mMapPoints.at(i).mCard.code_id;
        card.card_id = i;
        viz.cards.push_back(card);
    }
    pub_slam.publish(viz);
}

void CardDetection::MainLoop()
{
    ROS_INFO_STREAM("card_detection start");
    ros::Rate loop_rate(5);
    vslam::Viz viz;

    while (ros::ok())
    {

        ros::spinOnce();

        //debug
        //cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"
        //cvCreateTrackbar("max", "Control", &mParam.max_area, 1000);
        //cvCreateTrackbar("min", "Control", &mParam.min_area, 1000);
        if (bImage == false)
        {
            ROS_WARN_STREAM("wait Image msg...");
            continue;
        }
        // if (bOdom == false)
        // {
        //     ROS_WARN_STREAM("wait Odom msg...");
        //     continue;
        // }

        // if (bInit && m_CurrentKeyFrame.mKeyFrameId == 0)
        // {
        //     m_RefKeyFrame = m_CurrentKeyFrame;
        // }
        // else if (bInit)
        // {
        //     std::vector<MapPoint> map_points;
        //     m_RefKeyFrame = m_CurrentKeyFrame;
        //     triangulation(m_RefKeyFrame.mImage, m_CurrentKeyFrame.mImage, map_points);
        //     for (int i = 0; i < map_points.size(); i++)
        //     {
        //         Eigen::Vector3d eigen_vector;
        //         eigen_vector(0) = map_points.at(i).mCameraPositionPoint.x;
        //         eigen_vector(1) = map_points.at(i).mCameraPositionPoint.y;
        //         eigen_vector(2) = map_points.at(i).mCameraPositionPoint.z;

        //         ROS_INFO_STREAM("carmera:" << map_points.at(i).mCameraPositionPoint);

        //         cv::Point3d cv_point;
        //         if (mMap.mMapPoints.size() == 0)
        //         {
        //             cv_point.x = m_CurrentKeyFrame.mImage.mCamera.camera2world(eigen_vector).x() + 4.0;
        //             cv_point.y = m_CurrentKeyFrame.mImage.mCamera.camera2world(eigen_vector).y() + 4.0;
        //             cv_point.z = m_CurrentKeyFrame.mImage.mCamera.camera2world(eigen_vector).z();
        //         }
        //         else if (mMap.mMapPoints.size() % 2 == 0)
        //         {
        //             cv_point.x = m_CurrentKeyFrame.mImage.mCamera.camera2world(eigen_vector).x() + 4.0;
        //             cv_point.y = m_CurrentKeyFrame.mImage.mCamera.camera2world(eigen_vector).y() - 4.0;
        //             cv_point.z = m_CurrentKeyFrame.mImage.mCamera.camera2world(eigen_vector).z();
        //         }
        //         else
        //         {
        //             cv_point.x = m_CurrentKeyFrame.mImage.mCamera.camera2world(eigen_vector).x() + 4.0;
        //             cv_point.y = m_CurrentKeyFrame.mImage.mCamera.camera2world(eigen_vector).y() + 4.0;
        //             cv_point.z = m_CurrentKeyFrame.mImage.mCamera.camera2world(eigen_vector).z();
        //         }

        //         map_points.at(i).mWorldPositionPoint = cv_point;

        //         ROS_INFO_STREAM("world:" << map_points.at(i).mWorldPositionPoint);

        //         m_CurrentKeyFrame.mMapPoints.push_back(map_points.at(i));
        //         if (mMap.mMapPoints.empty())
        //             mMap.mMapPoints.push_back(map_points.at(i));
        //         bool bPush = false;
        //         for (int j = 0; j < mMap.mMapPoints.size(); j++)
        //         {
        //             if (mMap.mMapPoints.at(j).mCard.code_id == map_points.at(i).mCard.code_id)
        //             {
        //                 bPush = true;
        //                 break;
        //             }
        //         }
        //         if (bPush == false)
        //             mMap.mMapPoints.push_back(map_points.at(i));
        //     }
        //     //mMap.mKeyFrames.push_back(m_CurrentKeyFrame);

        //     m_RefKeyFrame = m_CurrentKeyFrame;
        // }
        // visualization_msgs::MarkerArray markers;
        // VisCardInWorld(markers);
        // publishVslam(viz);
        // pub_MarkerRviz.publish(markers);

        //debug
        cv::imshow("Control", image);
        cv::waitKey(1);

        // loop_rate.sleep();
    }
}
