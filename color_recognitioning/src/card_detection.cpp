/*
 * @Author: fengkai 
 * @Date: 2019-06-21 16:10:30 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-07-20 16:53:01
 */

#include "card_detection.h"

VSlam::VSlam() : pnh_("~"), bImage(false), bOdom(false), bInit(false),
                 m_CountImageId(0), m_CountKeyFrameId(0),
                 mp_CurrentKeyFrame(nullptr), mp_RefKeyFrame(nullptr)
{
    initROS();
}

VSlam::~VSlam()
{
}

void VSlam::initROS()
{

    pub_DetectedImageRviz = nh_.advertise<sensor_msgs::Image>("/detected_image_rviz", 1);
    pub_MarkerRviz = nh_.advertise<visualization_msgs::MarkerArray>("/marker_rviz", 10);
    pub_slam = nh_.advertise<vslam::Viz>("/slam_viz", 10);

    sub_ImageLeft = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/usb_cam1/image_raw", 1);
    sub_ImageFront = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/usb_cam2/image_raw", 1);
    sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *sub_ImageLeft, *sub_ImageFront);
    sync->registerCallback(boost::bind(&VSlam::callbackGetImage, this, _1, _2));

    sub_Odom = nh_.subscribe("/odom/imu", 10, &VSlam::callbackGetOdom, this);

    pnh_.param<int>("maxRectArea", mParam.max_area, 0);
    pnh_.param<int>("minRectArea", mParam.min_area, 0);

    pnh_.param<double>("left_fx", mCameraParamLeft.fx, 0);
    pnh_.param<double>("left_fy", mCameraParamLeft.fy, 0);
    pnh_.param<double>("left_cx", mCameraParamLeft.cx, 0);
    pnh_.param<double>("left_cy", mCameraParamLeft.cy, 0);
    pnh_.param<double>("left_k1", mCameraParamLeft.k1, 0);
    pnh_.param<double>("left_k2", mCameraParamLeft.k2, 0);
    pnh_.param<double>("left_k3", mCameraParamLeft.k3, 0);
    pnh_.param<double>("left_p1", mCameraParamLeft.p1, 0);
    pnh_.param<double>("left_p2", mCameraParamLeft.p2, 0);
    pnh_.param<int>("left_width", mCameraParamLeft.width, 0);
    pnh_.param<int>("left_height", mCameraParamLeft.height, 0);

    pnh_.param<double>("front_fx", mCameraParamFront.fx, 0);
    pnh_.param<double>("front_fy", mCameraParamFront.fy, 0);
    pnh_.param<double>("front_cx", mCameraParamFront.cx, 0);
    pnh_.param<double>("front_cy", mCameraParamFront.cy, 0);
    pnh_.param<double>("front_k1", mCameraParamFront.k1, 0);
    pnh_.param<double>("front_k2", mCameraParamFront.k2, 0);
    pnh_.param<double>("front_k3", mCameraParamFront.k3, 0);
    pnh_.param<double>("front_p1", mCameraParamFront.p1, 0);
    pnh_.param<double>("front_p2", mCameraParamFront.p2, 0);
    pnh_.param<int>("front_width", mCameraParamFront.width, 0);
    pnh_.param<int>("front_height", mCameraParamFront.height, 0);

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

    pnh_.param<std::string>("base_tf", mTFParam.base_tf, "");
    pnh_.param<std::string>("camera1_tf", mTFParam.camera1_tf, "");
    pnh_.param<std::string>("camera2_tf", mTFParam.camera2_tf, "");
    pnh_.param<std::string>("map_tf", mTFParam.map_tf, "");

    mCameraParamLeft.preProcess();
    mCameraParamFront.preProcess();
}

void VSlam::callbackGetOdom(const nav_msgs::OdometryConstPtr &msg)
{
    bOdom = true;
    m_CurrentPose = msg->pose.pose;
    time = msg->header.stamp;
}

void VSlam::callbackGetImage(const sensor_msgs::ImageConstPtr &msg_left, const sensor_msgs::ImageConstPtr &msg_front)
{
    m_CountImageId++;
    bImage = true;

    m_CurrentImage = Image();

    try
    {
        tf_Listener.waitForTransform(mTFParam.base_tf, mTFParam.camera1_tf, ros::Time(0), ros::Duration(0.1));
        tf_Listener.lookupTransform(mTFParam.base_tf, mTFParam.camera1_tf, ros::Time(0), tf_base_to_camera1);
        tf_Listener.waitForTransform(mTFParam.base_tf, mTFParam.camera2_tf, ros::Time(0), ros::Duration(0.1));
        tf_Listener.lookupTransform(mTFParam.base_tf, mTFParam.camera2_tf, ros::Time(0), tf_base_to_camera2);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("Transform error in imuCB: %s", ex.what());
        return;
    }

    tf::Quaternion q_;
    tf::Vector3 v_;
    tf::quaternionMsgToTF(m_CurrentPose.orientation, q_);
    tf::pointMsgToTF(m_CurrentPose.position, v_);

    tf_map_to_base = tf::Transform(q_, v_);

    //map->camera
    tf_map_to_camera1 = tf_map_to_base * tf_base_to_camera1;
    tf_map_to_camera2 = tf_map_to_base * tf_base_to_camera2;

    processPicture(m_CurrentImageMat[0], m_CurrentImage, msg_left, 1);
    processPicture(m_CurrentImageMat[1], m_CurrentImage, msg_front, 2);

    tf_Broadcaster.sendTransform(tf::StampedTransform(tf_map_to_base, time, mTFParam.map_tf, mTFParam.base_tf));

    if (isKeyPoint(m_CurrentImage))
    {
        mp_CurrentKeyFrame = new KeyFrame();
        setKeyFrame(m_CurrentImage, mp_CurrentKeyFrame, mp_RefKeyFrame);
    }
}

void VSlam::processPicture(cv::Mat &mat_cur, Image &image, const sensor_msgs::ImageConstPtr &msg, const int &flag)
{
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr pub_ptr;
    cv::Mat mat_raw;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        pub_ptr = cv_ptr;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    mat_raw = cv_ptr->image;

    switch (flag)
    {
    case 1:
        cv::remap(mat_raw, mat_cur, mCameraParamLeft.map1, mCameraParamLeft.map2, cv::INTER_LINEAR);
        break;
    case 2:
        cv::remap(mat_raw, mat_cur, mCameraParamFront.map1, mCameraParamFront.map2, cv::INTER_LINEAR);
        break;
    default:
        break;
    }

    //Eigen::Quaterniond quat(m_CurrentPose.orientation.w, m_CurrentPose.orientation.x, m_CurrentPose.orientation.y, m_CurrentPose.orientation.z);
    //Eigen::Vector3d t(m_CurrentPose.position.x, m_CurrentPose.position.y, m_CurrentPose.position.z);
    //Sophus::SE3d se3(quat, t);
    //m_CurrentImage.mCamera.mTwc = se3;

    //Eigen::Matrix3d r = quat.toRotationMatrix();
    // m_CurrentImage.pose << r(0, 0), r(0, 1), r(0, 2), t(0, 0),
    //     r(1, 0), r(1, 1), r(1, 2), t(1, 0),
    //     r(2, 0), r(2, 1), r(2, 2), t(2, 0);

    cardDetection(m_CurrentImageMat[flag - 1], m_CurrentImage, flag);
    detectDistance(m_CurrentImage, flag);

    //pub_ptr->image = m_CurrentImageMat;
    //m_CurrentImageMat = imgThresholdedRed;
    //pub_DetectedImageRviz.publish(pub_ptr->toImageMsg());
}

bool VSlam::isKeyPoint(const Image &image)
{
    int Count = 0;
    for (int i = 0; i < image.image_parts.size(); i++)
    {
        Count += image.image_parts.at(i).cards.size();
    }

    if (Count > 1)
        return true;
    else
    {
        return false;
    }
}

double VSlam::distance2(const double &x, const double &y)
{
    return (x * x + y * y);
}

void VSlam::cardDetection(cv::Mat &mat, Image &image, const int &flag)
{
    cv::Mat imgHSV;
    std::vector<cv::Mat> splitHSV;
    cv::cvtColor(mat, imgHSV, cv::COLOR_BGR2HSV);
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

            if ((double)rect.height / rect.width < 10 && (double)rect.height / rect.width > 4)
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
                    //ROS_INFO_STREAM(code_id);
                    Card card;
                    card.code_id = code_id;
                    card.width = rect.width;
                    card.height = rect.height;
                    card.center = rotated_rect.center;
                    card.card_id = card.code2card(card.code_id);

                    cv::Point2i lt, rt, lb, rb;
                    lt = cv::Point2i(rect.x, rect.y);
                    card.key_points.push_back(lt);

                    rt = cv::Point2i(rect.x + rect.width, rect.y);
                    card.key_points.push_back(rt);

                    lb = cv::Point2i(rect.x, rect.y + rect.height);
                    card.key_points.push_back(lb);

                    rb = cv::Point2i(rect.x + rect.width, rect.y + rect.height);
                    card.key_points.push_back(rb);
                    image.image_parts.at(flag - 1).cards.push_back(card);

                    cv::rectangle(mat, rect, cv::Scalar(0, 0, 255));
                    cv::putText(mat, code_id, card.center, cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 255));
                    cv::putText(mat, std::to_string(card.card_id), card.key_points.at(1), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 0, 255));
                }
            }
        }
    }
}

void VSlam::setKeyFrame(const Image &img, KeyFrame *kf_cur, KeyFrame *kf_ref)
{
    kf_ref = kf_cur;

    kf_cur = new KeyFrame(img, m_CountKeyFrameId++);
    kf_cur->refKeyFrame = kf_ref;

    mMap.insertKeyFrame(kf_cur);
    
    setMapPoint(kf_cur);
}

void VSlam::setCameraPose(KeyFrame *pKF)
{
    if (pKF->mKeyFrameId == 0)
    {
        Eigen::Matrix3d r = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t = Eigen::Vector3d::Zero();
        Sophus::SE3d se3(r, t);

        //pKF->mImage.mCamera.mTwc = se3;
        pKF->mImage.pose << r(0, 0), r(0, 1), r(0, 2), t(0, 0),
            r(1, 0), r(1, 1), r(1, 2), t(1, 0),
            r(2, 0), r(2, 1), r(2, 2), t(2, 0);
        return;
    }
    else
    {
        for (int i = 0; i < pKF->mpMapPoints.size(); i++)
        {
            //pKF->mImage.mCamera.world2camera(pKF->mpMapPoints.at(i)->mWorldPositionPoint);
        }
    }
}

void VSlam::setMapPoint(KeyFrame *pKF)
{
    for (int i = 0; i < pKF->mImage.image_parts.at(0).cards.size(); i++)
    {
        Card card = pKF->mImage.image_parts.at(0).cards.at(i);
        MapPoint *pMP = new MapPoint(card, pKF->mImage.image_parts.at(0).mCamera.camera2world(card.pose), card.card_id);
        if (mMap.insertMapPoint(pMP, pKF, i) == false) //mapPoint在地图中已有
            delete pMP;

        pKF->mpMapPoints.push_back(mMap.mpMapPoints.at(card.card_id));
        ROS_INFO_STREAM(card.card_id);
    }
}

void VSlam::VisCardInWorld(visualization_msgs::MarkerArray &markers)
{
    std::vector<MapPoint *> map_points = mMap.mpMapPoints;
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

        pose.position.x = map_points.at(i)->mWorldPositionPoint.x();
        pose.position.y = map_points.at(i)->mWorldPositionPoint.y();
        pose.position.z = map_points.at(i)->mWorldPositionPoint.z();

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

void VSlam::detectDistance(Image &image, const int &flag)
{
    for (int i = 0; i < image.image_parts.at(flag - 1).cards.size(); i++)
    {
        Eigen::Vector3d v[5];
        CameraParam camera_param;
        tf::Transform transform;
        switch (flag)
        {
        case 1:
            camera_param = mCameraParamLeft;
            transform = tf_base_to_camera1;
            
            break;
        case 2:
            camera_param = mCameraParamFront;
            transform = tf_base_to_camera2;
            break;
        default:
            break;
        }
        for (int j = 0; j < 4; j++)
        {
            v[j] = image.image_parts.at(flag - 1).mCamera.pixel2camera(image.image_parts.at(flag - 1).cards.at(i).key_points.at(j), camera_param, 1);
        }

        v[4] = image.image_parts.at(flag - 1).mCamera.pixel2camera(image.image_parts.at(flag - 1).cards.at(i).center, camera_param, 1);

        double width, height;
        width = (v[1] - v[0]).x();
        height = (v[2] - v[0]).y();
        double depth = 0.7 / height * 1;

        tf::Point relative_point(depth,depth * v[4].x() * (-1),0);
        tf::Point absolute_point = transform * relative_point; 

        image.image_parts.at(flag - 1).cards.at(i).pose(0, 0) = absolute_point.getX();
        image.image_parts.at(flag - 1).cards.at(i).pose(1, 0) = absolute_point.getY();
        //image.image_parts.at(flag - 1).cards.at(i).pose(2, 0) = depth * v[4].y();
        image.image_parts.at(flag - 1).cards.at(i).pose(2, 0) = absolute_point.getZ();

        cv::putText(m_CurrentImageMat[flag - 1], std::to_string(image.image_parts.at(flag - 1).cards.at(i).pose(0, 0)), image.image_parts.at(flag - 1).cards.at(i).key_points.at(3), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(0, 255, 0));
    }
}

// void VSlam::triangulation(const Image &image1, const Image &image2, std::vector<MapPoint> &map_points)
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

void VSlam::publishVslam(vslam::Viz &viz)
{
    viz.header.frame_id = "odom";
    viz.header.stamp = ros::Time::now();
    vslam::Camera camera;
    vslam::Card card;
    camera.camera_pose = m_CurrentPose;
    viz.camera = camera;
    for (int i = 0; i < mMap.mpMapPoints.size(); i++)
    {
        card.card_pose.position.x = mMap.mpMapPoints.at(i)->mWorldPositionPoint.x();
        card.card_pose.position.y = mMap.mpMapPoints.at(i)->mWorldPositionPoint.y();
        card.card_pose.position.z = mMap.mpMapPoints.at(i)->mWorldPositionPoint.z();
        card.code_id = mMap.mpMapPoints.at(i)->mCard.code_id;
        card.card_id = i;
        viz.cards.push_back(card);
    }
    pub_slam.publish(viz);
}

void VSlam::MainLoop()
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
        if (bOdom == false)
        {
            ROS_WARN_STREAM("wait Odom msg...");
            continue;
        }

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
        cv::imshow("left", m_CurrentImageMat[0]);
        cv::imshow("front", m_CurrentImageMat[1]);

        cv::waitKey(1);

        // loop_rate.sleep();
    }
}
