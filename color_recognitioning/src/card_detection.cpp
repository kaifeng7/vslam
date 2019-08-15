/*
 * @Author: fengkai 
 * @Date: 2019-06-21 16:10:30 
 * @Last Modified by: fengkai
 * @Last Modified time: 2019-08-01 22:56:08
 */

#include "card_detection.h"

VSlam::VSlam() : pnh_("~"), bImage(false), bOdom(false), bInit(false), bLoop(false),
                 m_CountImageId(0), m_CountKeyFrameId(0), m_CountMapPointId(0),
                 mp_CurrentKeyFrame(nullptr), mp_RefKeyFrame(nullptr), mCountCards(128, 0)
{
    initROS();
}

VSlam::~VSlam()
{
}

void VSlam::initROS()
{

    pub_DetectedImageRviz = nh_.advertise<sensor_msgs::Image>("/detected_image_rviz", 1);
    pub_CardRviz = nh_.advertise<visualization_msgs::Marker>("/card_rviz", 10);
    pub_IdRviz = nh_.advertise<visualization_msgs::Marker>("/id_rviz", 10);
    pub_CameraRviz = nh_.advertise<visualization_msgs::Marker>("/camera_rviz", 10);
    pub_PointRviz = nh_.advertise<visualization_msgs::Marker>("/point_rviz", 10);
    pub_LoopCameraRviz = nh_.advertise<visualization_msgs::MarkerArray>("/loop_camera_rviz", 10);
    pub_CardPositionRviz = nh_.advertise<visualization_msgs::MarkerArray>("/card_position_rviz", 10);
    pub_IdPositionRviz = nh_.advertise<visualization_msgs::MarkerArray>("/id_position_rviz", 10);

    pub_PoseEKFRviz = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/ekf_pose_rviz", 10);
    pub_CardEKFRviz = nh_.advertise<visualization_msgs::MarkerArray>("/ekf_card_rviz", 10);

    pub_slam = nh_.advertise<vslam::Viz>("/slam_viz", 10);

    // sub_ImageLeft = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/usb_cam1/image_raw", 1);
    // sub_ImageFront = new message_filters::Subscriber<sensor_msgs::Image>(nh_, "/usb_cam2/image_raw", 1);
    // sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(10), *sub_ImageLeft, *sub_ImageFront);
    // sync->registerCallback(boost::bind(&VSlam::callbackGetImage, this, _1, _2));

    sub_Odom = nh_.subscribe("/odom/imu", 10, &VSlam::callbackGetOdom, this);
    sub_Image = nh_.subscribe("/usb_cam1/image_raw",5,&VSlam::callbackGetImage,this);

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

    pnh_.param<double>("K_imu", mKalman.K_imu, 0);
    pnh_.param<double>("K_wheel", mKalman.K_wheel, 0);

    pnh_.param<double>("K_r", mKalman.K_r, 0);
    pnh_.param<double>("K_thita", mKalman.K_thita, 0);

    mCameraParamLeft.preProcess();
    mCameraParamFront.preProcess();

    mKalman.mu_.resize(3, 1);
    mKalman.mu_.setZero();
    mKalman.sigma_.resize(3, 3);
    mKalman.sigma_.setZero();
}

void VSlam::callbackGetOdom(const nav_msgs::OdometryConstPtr &msg)
{
    if (bOdom == false)
    {
        m_CurrentPose = msg->pose.pose;
        m_PrePose = msg->pose.pose;
        bOdom = true;
    }
    else
    {
        m_PrePose = m_CurrentPose;
        m_CurrentPose = msg->pose.pose;
    }
    time = msg->header.stamp;
    double m_DiffDistance = sqrt((m_CurrentPose.position.x - m_PrePose.position.x)* (m_CurrentPose.position.x - m_PrePose.position.x) 
                                + (m_CurrentPose.position.y - m_PrePose.position.y) * (m_CurrentPose.position.y - m_PrePose.position.y));

    

    tf::Quaternion quat1, quat2;
    tf::quaternionMsgToTF(m_CurrentPose.orientation, quat1);
    tf::quaternionMsgToTF(m_PrePose.orientation, quat2);

    double roll1, pitch1, yaw1;
    double roll2, pitch2, yaw2;

    tf::Matrix3x3(quat1).getRPY(roll1, pitch1, yaw1); //进行转换
    tf::Matrix3x3(quat2).getRPY(roll2, pitch2, yaw2); //进行转换

    m_CurrentAngle = yaw1;
    m_PreAngle = yaw2;
    m_DiffAngle = m_CurrentAngle - m_PreAngle;

    mKalman.mu_(2, 0) += m_DiffAngle;
    normAngle(mKalman.mu_(2, 0));
    m_DiffPose.position.x = m_DiffDistance*cos(mKalman.mu_(2, 0));
    m_DiffPose.position.y = m_DiffDistance*sin(mKalman.mu_(2, 0));

    mKalman.mu_(0, 0) += m_DiffPose.position.x;
    mKalman.mu_(1, 0) += m_DiffPose.position.y;


    Eigen::Matrix3d G_xi; //位資雅克比
    G_xi << 1.0, 0.0, -m_DiffPose.position.y,
        0.0, 1.0, m_DiffPose.position.x,
        0.0, 0.0, 1.0;

    Eigen::Matrix<double, 3, 2> G_u; //控制雅克比
    G_u << cos(mKalman.mu_(2, 0)), -m_DiffPose.position.y,
        sin(mKalman.mu_(2, 0)), m_DiffPose.position.x,
        0.0, 1.0;
    int N = mKalman.mu_.rows();
    Eigen::MatrixXd F(N, 3);
    F.setZero();
    F.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();

    Eigen::MatrixXd G_t = Eigen::MatrixXd::Identity(N, N); //g(X(t-1),Ut)關於X(t-1)的雅克比

    G_t.block(0, 0, 3, 3) = G_xi;

    Eigen::Matrix2d sigma_u;
    sigma_u << mKalman.K_wheel * mKalman.K_wheel * m_DiffDistance * m_DiffDistance, 0.0,
        0.0, mKalman.K_imu * mKalman.K_imu * m_DiffAngle * m_DiffAngle;

    //更新協方差
    mKalman.sigma_ = G_t * mKalman.sigma_ * G_t.transpose() + F * G_u * sigma_u * G_u.transpose() * F.transpose();
}

void VSlam::callbackGetImage(const sensor_msgs::ImageConstPtr &msg_front)
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

    q_ = tf::createQuaternionFromYaw(mKalman.mu_(2, 0));

    geometry_msgs::Point tmp_position;
    tmp_position.x = mKalman.mu_(0, 0);
    tmp_position.y = mKalman.mu_(1, 0);
    tmp_position.z = 0;

    tf::pointMsgToTF(tmp_position, v_);

    tf_map_to_base = tf::Transform(q_, v_);

    //map->camera
    tf_map_to_camera1 = tf_map_to_base * tf_base_to_camera1;
    tf_map_to_camera2 = tf_map_to_base * tf_base_to_camera2;

    Eigen::Quaterniond quat(q_.getW(), q_.getX(), q_.getY(), q_.getZ());
    Eigen::Vector3d t(tmp_position.x, tmp_position.y, tmp_position.z);
    Sophus::SE3d se3(quat, t);
    for (int i = 0; i < m_CurrentImage.image_parts.size(); i++)
        m_CurrentImage.image_parts.at(i).mCamera.mTwc = se3;

    Eigen::Matrix3d r = quat.toRotationMatrix();
    m_CurrentImage.pose << r(0, 0), r(0, 1), r(0, 2), t(0, 0),
        r(1, 0), r(1, 1), r(1, 2), t(1, 0),
        r(2, 0), r(2, 1), r(2, 2), t(2, 0);

    //processPicture(m_CurrentImageMat[0], m_CurrentImage, msg_left, 1);
    processPicture(m_CurrentImageMat[1], m_CurrentImage, msg_front, 2);

    //tf_Broadcaster.sendTransform(tf::StampedTransform(tf_map_to_base, time, mTFParam.map_tf, mTFParam.base_tf));

    if (isKeyFrame(m_CurrentImage))
    {

        setKeyFrame(m_CurrentImage, &mp_CurrentKeyFrame, &mp_RefKeyFrame);
        //ROS_INFO_STREAM(mp_CurrentKeyFrame->mImage.pose(0,3)<<" "<<mp_CurrentKeyFrame->mImage.pose(1,3));
        visualization_msgs::Marker camera_pos;
        if (VisCameraWorld(camera_pos, mp_CurrentKeyFrame))
            pub_CameraRviz.publish(camera_pos);
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

    cardDetection(mat_cur, m_CurrentImage, flag);
    detectDistance(m_CurrentImage, flag);

    pub_ptr->image = mat_cur;
    //m_CurrentImageMat = imgThresholdedRed;
    pub_DetectedImageRviz.publish(pub_ptr->toImageMsg());
}

bool VSlam::isKeyFrame(const Image &image)
{
    int Count = 0;
    for (int i = 0; i < image.image_parts.size(); i++)
    {
        Count += image.image_parts.at(i).cards.size();
    }

    if (Count > -1)
        return true;
    else
    {
        return false;
    }
}

bool VSlam::isMapPoint(const Card &card)
{
    if (mCountCards.at(card.card_id) >= 5)
    {
        return true;
    }
    else
    {
        return false;
    }
}

double VSlam::distance2(const double &x1, const double &y1, const double &x2, const double &y2)
{
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

void VSlam::normAngle(double &angle)
{
    if (angle >= M_PI)
    {
        angle -= M_PI * 2;
    }
    if (angle <= -M_PI)
    {
        angle += M_PI * 2;
    }
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

void VSlam::setKeyFrame(const Image &img, KeyFrame **pp_kf_cur, KeyFrame **pp_kf_ref)
{
    *pp_kf_ref = *pp_kf_cur;

    *pp_kf_cur = new KeyFrame(img, m_CountKeyFrameId++);
    (*pp_kf_cur)->refKeyFrame = *pp_kf_ref;

    mMap.insertKeyFrame(*pp_kf_cur);
    std::vector<int> vi(128, 0);
    for (int i = 0; i < (*pp_kf_cur)->mImage.image_parts.size(); i++)
    {
        for (int j = 0; j < (*pp_kf_cur)->mImage.image_parts.at(i).cards.size(); j++)
        {
            vi.at((*pp_kf_cur)->mImage.image_parts.at(i).cards.at(j).card_id) = 1;
        }
    }
    for (int i = 0; i < 128; i++)
    {
        if (vi.at(i) == 1)
        {
            mCountCards.at(i)++;
        }
        else
        {
            mCountCards.at(i) = 0;
        }
    }
    MapPoint *mp = nullptr;
    setMapPoint(*pp_kf_cur, &mp);
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

void VSlam::setMapPoint(KeyFrame *pKF, MapPoint **ppMP)
{
    for (int i = 0; i < pKF->mImage.image_parts.at(1).cards.size(); i++) //前
    {

        Card card = pKF->mImage.image_parts.at(1).cards.at(i);
        if (isMapPoint(card))
        {

            Eigen::Matrix2d Q;
            Q << mKalman.K_r * mKalman.K_r * card.r * card.r, 0.0,
                0.0, mKalman.K_thita * mKalman.K_thita * card.thita * card.thita;

            if (mMap.mpMapPoints.at(card.card_id) == nullptr)
            {//新路標
            
                *ppMP = new MapPoint(card, pKF->mImage.image_parts.at(1).mCamera.camera2world(card.pose), m_CountMapPointId);
                m_CountMapPointId++;
                mMap.mMapPointFlags.push_back(card.card_id);
                mMap.insertMapPoint(*ppMP, pKF, i);

                double angle = mKalman.mu_(2, 0) + card.thita;
                normAngle(angle);
                double mx = card.r * cos(angle) + mKalman.mu_(0, 0);
                double my = card.r * sin(angle) + mKalman.mu_(1, 0);

                Eigen::Matrix3d sigma_xi = mKalman.sigma_.block(0, 0, 3, 3);
                Eigen::Matrix<double, 2, 3> G_p; //位資雅克比
                G_p << 1, 0, -card.r * sin(angle),
                    0, 1, card.r * cos(angle);
                Eigen::Matrix2d G_z; //色塊雅克比
                G_z << cos(angle), -card.r * sin(angle),
                    sin(angle), -card.r * cos(angle);

                //新地圖點協方差
                Eigen::Matrix2d sigma_m = G_p * sigma_xi * G_p.transpose() + G_z * Q * G_z.transpose();

                int N = mKalman.mu_.rows();

                Eigen::MatrixXd G_fx; //新狀態與原狀態之間的協方差
                G_fx.resize(2, N);
                G_fx.setZero();
                G_fx.block(0, 0, 2, 3) = G_p;

                //新地圖點相對於已有狀態的協方差
                Eigen::MatrixXd sigma_mx;
                sigma_mx.resize(2, N);
                sigma_mx.setZero();
                sigma_mx = G_fx * mKalman.sigma_;

                //加入地圖
                //擴展均值
                Eigen::MatrixXd tmp_mu(N + 2, 1);
                tmp_mu.setZero();
                tmp_mu << mKalman.mu_, mx, my;

                mKalman.mu_.resize(N + 2, 1);
                mKalman.mu_ = tmp_mu;

                //擴展協方差

                Eigen::MatrixXd tmp_sigma(N + 2, N + 2);
                tmp_sigma.setZero();
                tmp_sigma.block(0, 0, N, N) = mKalman.sigma_;
                tmp_sigma.block(N, N, 2, 2) = sigma_m;
                tmp_sigma.block(N, 0, 2, N) = sigma_mx;
                tmp_sigma.block(0, N, N, 2) = sigma_mx.transpose();

                mKalman.sigma_.resize(N + 2, N + 2);
                mKalman.sigma_ = tmp_sigma;
            }
            else //mapPoint在地图中已有
            {

                int N = mKalman.mu_.rows();
                Eigen::MatrixXd F(5, N);
                F.setZero();
                F.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
                F(3, 3 + 2 * mMap.mpMapPoints.at(card.card_id)->mMapPointId) = 1;
                F(4, 4 + 2 * mMap.mpMapPoints.at(card.card_id)->mMapPointId) = 1;

                double &mx = mKalman.mu_(3 + 2 * mMap.mpMapPoints.at(card.card_id)->mMapPointId, 0);
                double &my = mKalman.mu_(4 + 2 * mMap.mpMapPoints.at(card.card_id)->mMapPointId, 0);
                double &x = mKalman.mu_(0, 0);
                double &y = mKalman.mu_(1, 0);
                double &theta = mKalman.mu_(2, 0);
                double delta_x = mx - x;
                double delta_y = my - y;
                double dis2 = delta_x * delta_x + delta_y * delta_y;
                double dis = sqrt(dis2);

                //v[x,y,theta,mx,my]
                Eigen::MatrixXd H_v(2, 5); //觀測Zt相對於v的雅克比
                H_v << -dis * delta_x, -dis * delta_y, 0, dis * delta_x, dis * delta_y,
                    delta_y, -delta_x, -dis2, -delta_y, delta_x;
                if (dis2 > 0)
                    H_v = (1 / dis2) * H_v;
                Eigen::MatrixXd H_t = H_v * F; //觀測Zt相對於全空間Xt的雅克比

                //EKF
                Eigen::MatrixXd K = mKalman.sigma_ * H_t.transpose() * (H_t * mKalman.sigma_ * H_t.transpose() + Q).inverse();

                double thita_hat = atan2(delta_y, delta_x) - theta;
                normAngle(thita_hat);
                Eigen::Vector2d z_hat(dis, thita_hat);
                Eigen::Vector2d z(card.r, card.thita);
                mKalman.mu_ = mKalman.mu_ + K * (z - z_hat);
                Eigen::MatrixXd I = Eigen::MatrixXd::Identity(N, N);
                mKalman.sigma_ = (I - K * H_t) * mKalman.sigma_;
            }

            pKF->mpMapPoints.push_back(mMap.mpMapPoints.at(card.card_id));

            visualization_msgs::Marker card_pos;
            visualization_msgs::Marker card_id;

            if (VisCardInWorld(card_pos, *ppMP))
                pub_CardRviz.publish(card_pos);
            if (VisIdInWorld(card_id, *ppMP))
                pub_IdRviz.publish(card_id);
        }
    }

    // for (int i = 0; i < pKF->mImage.image_parts.at(0).cards.size(); i++) //左
    // {
    //     Card card = pKF->mImage.image_parts.at(0).cards.at(i);
    //     if (isMapPoint(card))
    //     {
    //         *ppMP = new MapPoint(card, pKF->mImage.image_parts.at(0).mCamera.camera2world(card.pose), card.card_id);
    //         if (mMap.insertMapPoint(*ppMP, pKF, pKF->mKeyFrameId) == false) //mapPoint在地图中已有
    //         {
    //             delete *ppMP;
    //             *ppMP = nullptr;
    //         }

    //         pKF->mpMapPoints.push_back(mMap.mpMapPoints.at(card.card_id));

    //         visualization_msgs::Marker card_pos;
    //         visualization_msgs::Marker card_id;

    //         if (VisCardInWorld(card_pos, *ppMP))
    //             pub_CardRviz.publish(card_pos);
    //         if (VisIdInWorld(card_id, *ppMP))
    //             pub_IdRviz.publish(card_id);
    //     }
    // }
}

bool VSlam::VisCardInWorld(visualization_msgs::Marker &marker, const MapPoint *mp)
{
    if (mp == nullptr)
    {
        return false;
    }
    geometry_msgs::Pose pose;

    pose.position.x = mp->mWorldPositionPoint.x();
    pose.position.y = mp->mWorldPositionPoint.y();
    pose.position.z = mp->mWorldPositionPoint.z();

    marker.pose = pose;
    marker.header.stamp = ros::Time();
    marker.id = mp->mMapPointId;
    marker.header.frame_id = "/odom";
    marker.ns = "Card_Position";

    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.color.a = 1;
    marker.color.b = 0;
    marker.color.g = 1;
    marker.color.r = 0;
    marker.lifetime = ros::Duration();
    marker.frame_locked = false;
    return true;
}

bool VSlam::VisIdInWorld(visualization_msgs::Marker &marker, const MapPoint *mp)
{
    if (mp == nullptr)
    {
        return false;
    }
    geometry_msgs::Pose pose;
    pose.position.x = mp->mWorldPositionPoint.x();
    pose.position.y = mp->mWorldPositionPoint.y();
    pose.position.z = mp->mWorldPositionPoint.z();

    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.r = 1.0;
    marker.color.g = 1.0;

    marker.header.stamp = ros::Time();
    marker.header.frame_id = "/odom";
    marker.ns = "Card_Id";
    marker.id = mp->mMapPointId;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.text = mp->mCard.code_id;
    marker.pose = pose;

    marker.scale.x = 0.0;
    marker.scale.y = 3.0;
    marker.scale.z = 1.0;

    marker.frame_locked = false;
    marker.lifetime = ros::Duration();
    return true;
}

bool VSlam::VisCameraWorld(visualization_msgs::Marker &marker, const KeyFrame *kf)
{
    if (kf == nullptr)
    {
        return false;
    }
    geometry_msgs::Pose pose;
    pose.position.x = kf->mImage.pose(0, 3);
    pose.position.y = kf->mImage.pose(1, 3);

    pose.position.z = 0;

    marker.color.a = 1.0;
    marker.color.b = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;

    marker.header.stamp = ros::Time();
    marker.header.frame_id = "/odom";
    marker.ns = "Camera_position";
    marker.id = kf->mKeyFrameId;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;

    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    marker.frame_locked = false;
    marker.lifetime = ros::Duration();
    return true;
}

bool VSlam::VisPointWorld(visualization_msgs::Marker &marker, const KeyFrame *kf)
{
    if (kf == nullptr)
    {
        return false;
    }
    geometry_msgs::Pose pose;
    pose.position.x = kf->mImage.pose(0, 3);
    pose.position.y = kf->mImage.pose(1, 3);
    //ROS_INFO_STREAM(kf->mImage.pose(0,3)<<" "<<kf->mImage.pose(1,3));

    pose.position.z = 0;

    marker.color.a = 1.0;
    marker.color.b = 0.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;

    marker.header.stamp = ros::Time();
    marker.header.frame_id = "/odom";
    marker.ns = "Camera_position";
    marker.id = kf->mKeyFrameId;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = pose;

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    marker.frame_locked = false;
    marker.lifetime = ros::Duration();
    return true;
}

void VSlam::VisLoopCameraWorld(visualization_msgs::MarkerArray &markers, const std::vector<geometry_msgs::Pose> &poses)
{
    for (int i = 0; i < poses.size(); i++)
    {
        visualization_msgs::Marker marker;
        marker.pose = poses.at(i);

        marker.color.a = 1.0;
        marker.color.b = 0.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;

        marker.header.stamp = ros::Time();
        marker.header.frame_id = "/odom";
        marker.ns = "Loop_Camera_position";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.frame_locked = false;
        marker.lifetime = ros::Duration();

        markers.markers.push_back(marker);
    }
}

visualization_msgs::MarkerArray VSlam::VisCardPosition()
{
    visualization_msgs::MarkerArray markers;

    for (int i = 4; i < mKalman.mu_.rows(); i += 2)
    {
        double &mx = mKalman.mu_(i - 1, 0);
        double &my = mKalman.mu_(i, 0);
        
        //構造marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.ns = "card_position";
        marker.id = i;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = mx;
        marker.pose.position.y = my;
        marker.pose.position.z = 0;

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.b = 0.0;
        marker.color.g = 1.0;

        markers.markers.push_back(marker);
    }
    return markers;
}

visualization_msgs::MarkerArray VSlam::VisIdPosition()
{
    visualization_msgs::MarkerArray markers;
   
    for (int i = 4; i < mKalman.mu_.rows(); i += 2)
    {
        double &mx = mKalman.mu_(i - 1, 0);
        double &my = mKalman.mu_(i, 0);

       
        //構造marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.ns = "id_position";
        marker.id = i;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = mx;
        marker.pose.position.y = my;
        marker.pose.position.z = 0;

        marker.text = mMap.mpMapPoints.at(mMap.mMapPointFlags.at(i/2-2))->mCard.code_id;

        marker.scale.x = 0.0;
        marker.scale.y = 3.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.b = 0.0;
        marker.color.g = 1.0;

        markers.markers.push_back(marker);
    }
    return markers;
}

geometry_msgs::PoseWithCovarianceStamped VSlam::VisPosEKF()
{
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "odom";

    msg.pose.pose.position.x = mKalman.mu_(0, 0);
    msg.pose.pose.position.y = mKalman.mu_(1, 0);
    msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(mKalman.mu_(2, 0));
    msg.pose.covariance.at(0) = mKalman.sigma_(0, 0);
    msg.pose.covariance.at(1) = mKalman.sigma_(0, 1);
    msg.pose.covariance.at(6) = mKalman.sigma_(1, 0);
    msg.pose.covariance.at(7) = mKalman.sigma_(1, 1);
    msg.pose.covariance.at(5) = mKalman.sigma_(0, 2);
    msg.pose.covariance.at(30) = mKalman.sigma_(2, 0);
    msg.pose.covariance.at(35) = mKalman.sigma_(2, 2);
    return msg;
}

visualization_msgs::MarkerArray VSlam::VisCardEKF()
{
    visualization_msgs::MarkerArray markers;
    int N = 0;
    for (int i = 4; i < mKalman.mu_.rows(); i += 2)
    {
        double &mx = mKalman.mu_(i - 1, 0);
        double &my = mKalman.mu_(i, 0);

        //計算圖點協方差
        Eigen::Matrix2d sigma_m = mKalman.sigma_.block(i - 1, i - 1, 2, 2); //協方差
        cv::Mat cv_sigma_m = (cv::Mat_<double>(2, 2) << sigma_m(0, 0), sigma_m(0, 1), sigma_m(1, 0), sigma_m(1, 1));

        cv::Mat eigen_value, eigen_vector;

        cv::eigen(cv_sigma_m, eigen_value, eigen_vector);
        double angle = atan2(eigen_vector.at<double>(0, 1), eigen_vector.at<double>(0, 0));
        double x_len = 2 * sqrt(eigen_value.at<double>(0, 0) * 5.991);
        double y_len = 2 * sqrt(eigen_value.at<double>(1.0) * 5.991);

        //構造marker
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.ns = "ekf_card";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = mx;
        marker.pose.position.y = my;
        marker.pose.position.z = 0;

        marker.pose.orientation = tf::createQuaternionMsgFromYaw(angle);
        marker.scale.x = 3 * x_len;
        marker.scale.y = 3 * y_len;
        marker.scale.z = 0.3 * (x_len + y_len);
        marker.color.a = 0.8;
        marker.color.r = 1.0;
        marker.color.b = 0.0;
        marker.color.g = 0.0;

        markers.markers.push_back(marker);
    }
    return markers;
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

        tf::Point relative_point(depth, depth * (-1) * v[4].x(), 0);
        tf::Point absolute_point = transform * relative_point;

        image.image_parts.at(flag - 1).cards.at(i).pose(0, 0) = absolute_point.getX();
        image.image_parts.at(flag - 1).cards.at(i).pose(1, 0) = absolute_point.getY();
        //image.image_parts.at(flag - 1).cards.at(i).pose(2, 0) = depth * v[4].y();
        image.image_parts.at(flag - 1).cards.at(i).pose(2, 0) = absolute_point.getZ();

        image.image_parts.at(flag - 1).cards.at(i).r = sqrt(absolute_point.getX() * absolute_point.getX() + absolute_point.getY() * absolute_point.getY());
        image.image_parts.at(flag - 1).cards.at(i).thita = atan2(absolute_point.getY(), absolute_point.getX());

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

vslam::Viz VSlam::publishVslam()
{
    vslam::Viz viz;
    viz.header.frame_id = "odom";
    viz.header.stamp = ros::Time::now();
    vslam::KeyFrame camera;
    vslam::MapPoint card;

    camera.camera_pose.position.x = mKalman.mu_(0,0);
    camera.camera_pose.position.y = mKalman.mu_(1,0);
    camera.camera_pose.position.z = 0;
    camera.camera_pose.orientation = tf::createQuaternionMsgFromYaw(mKalman.mu_(2,0));

    viz.key_frame = camera;
    std::vector<bool> flag(128,false);
    for(int i = 0;i<mp_CurrentKeyFrame->mpMapPoints.size();i++)
    {
    flag.at(mp_CurrentKeyFrame->mpMapPoints.at(i)->mCard.card_id) = true;

    }
    for (int i = 4; i < mKalman.mu_.rows(); i+=2)
    {
        card.card_pose.position.x = mKalman.mu_(i-1,0);
        card.card_pose.position.y = mKalman.mu_(i,0);
        card.card_pose.position.z = 0;
        card.code_id = mMap.mpMapPoints.at(mMap.mMapPointFlags.at(i/2-2))->mCard.code_id;
        card.card_id = mMap.mpMapPoints.at(mMap.mMapPointFlags.at(i/2-2))->mCard.card_id;
        if(flag.at(card.card_id) == true)
        {
            card.current = true;
        }
        else
        {
            card.current =false;
        }
        
        viz.map_points.push_back(card);
    }
    return viz;

}

void VSlam::MainLoop()
{
    ROS_INFO_STREAM("card_detection start");
    ros::Rate loop_rate(5);
    vslam::Viz viz;

    bool warn_image = false, warn_odom = false;

    while (ros::ok())
    {

        ros::spinOnce();

        //debug
        //cv::namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"


        if (bImage == false && warn_image == false)
        {
            ROS_WARN_STREAM("wait Image msg...");
            warn_image = true;
            continue;
        }
        else if (bImage == true && warn_image == true)
        {
            ROS_INFO_STREAM("receive image");
            warn_image = false;
        }
        else if(bImage == false && warn_image == true)
        {
            continue;
        }
        
        if (bOdom == false && warn_odom == false)
        {
            ROS_WARN_STREAM("wait Odom msg...");
            warn_odom = true;
            continue;
        }
        else if (bOdom == true && warn_odom == true)
        {
            ROS_INFO_STREAM("receive odom");
            warn_odom = false;
        }
        else if(bOdom == false && warn_odom == true)
        {
            continue;
        }

        //ROS_INFO_STREAM("Received all msg, start working");
        //int begin,end;
        // if (mMap.mpKeyFrames.at(end) != nullptr)
        // {
        //     visualization_msgs::Marker point_pose;
        //     if (VisPointWorld(point_pose, mMap.mpKeyFrames.at(end)))
        //         pub_PointRviz.publish(point_pose);
        // }
        // if (mMap.mpKeyFrames.at(begin) != nullptr)
        // {
        //     visualization_msgs::Marker point_pose;
        //     if (VisPointWorld(point_pose, mMap.mpKeyFrames.at(begin)))
        //         pub_PointRviz.publish(point_pose);
        // }
        // if (m_CountKeyFrameId > end && bLoop == false)
        // {
        //     bLoop = true;

        //     mKalman.mu_(0, 0) = mMap.mpKeyFrames.at(begin)->mImage.pose(0, 3);
        //     mKalman.mu_(1, 0) = mMap.mpKeyFrames.at(begin)->mImage.pose(1, 3);
        //     mKalman.mu_(2, 0) = mMap.mpKeyFrames.at(begin)->mImage.image_parts.at(0).mCamera.mTwc.angleZ();

        //     geometry_msgs::Pose first, last;
        //     first.position.x = mMap.mpKeyFrames.at(begin)->mImage.pose(0, 3);
        //     first.position.y = mMap.mpKeyFrames.at(begin)->mImage.pose(1, 3);
        //     first.position.z = mMap.mpKeyFrames.at(begin)->mImage.image_parts.at(0).mCamera.mTwc.angleZ(); //angle

        //     last.position.x = mMap.mpKeyFrames.at(end)->mImage.pose(0, 3);
        //     last.position.y = mMap.mpKeyFrames.at(end)->mImage.pose(1, 3);
        //     last.position.z = mMap.mpKeyFrames.at(end)->mImage.image_parts.at(0).mCamera.mTwc.angleZ(); //angle

        //     geometry_msgs::Pose diff;
        //     diff.position.x = last.position.x - first.position.x;
        //     diff.position.y = last.position.y - first.position.y;
        //     diff.position.z = last.position.z - first.position.z;

        //     double delta_x = diff.position.x / (end-first));
        //     double delta_y = diff.position.y / (end-first);
        //     double delta_z = diff.position.z / (end-first);
        //     //ROS_INFO_STREAM(first.position.z<<" "<<last.position.z);

        //     std::vector<geometry_msgs::Pose> poses;
        //     std::vector<double> dis;
        //     for (int i = begin; i <= end; i++)
        //     {
        //         double x1 = mMap.mpKeyFrames.at(i)->mImage.pose(0, 3);
        //         double y1 = mMap.mpKeyFrames.at(i)->mImage.pose(1, 3);
        //         double x2 = mMap.mpKeyFrames.at(i)->refKeyFrame->mImage.pose(0, 3);
        //         double y2 = mMap.mpKeyFrames.at(i)->refKeyFrame->mImage.pose(1, 3);
        //         dis.push_back(distance2(x1, y1, x2, y2));
        //     }
        //     for (int i = begin; i <= end; i++)
        //     {
        //         geometry_msgs::Pose pose;

        //         //mMap.mpKeyFrames.at(i)->mImage.pose(0, 3) -= delta_x * (i - begin);
        //         //mMap.mpKeyFrames.at(i)->mImage.pose(1, 3) -= delta_y * (i - begin);

        //         double angle = mMap.mpKeyFrames.at(i)->mImage.image_parts.at(0).mCamera.mTwc.angleZ();
        //         angle -= delta_z * (i - begin);

        //         //  ROS_INFO_STREAM("dis: "<<dis.at(i)<<" x1:"<<x1<<" y1:"<<y1<<" x2:"<<x2<<" y2:");

        //         mMap.mpKeyFrames.at(i)->mImage.pose(0, 3) = dis.at(i - begin) * cos(angle) + mMap.mpKeyFrames.at(i)->refKeyFrame->mImage.pose(0, 3);
        //         mMap.mpKeyFrames.at(i)->mImage.pose(1, 3) = dis.at(i - begin) * sin(angle) + mMap.mpKeyFrames.at(i)->refKeyFrame->mImage.pose(1, 3);
        //         pose.position.x = mMap.mpKeyFrames.at(i)->mImage.pose(0, 3);
        //         pose.position.y = mMap.mpKeyFrames.at(i)->mImage.pose(1, 3);
        //         pose.position.z = 0;

        //         for (int j = 0; j < mMap.mpKeyFrames.at(i)->mImage.image_parts.size(); j++)
        //         {
        //             tf::Quaternion tf_q;
        //             tf_q.setRPY(0, 0, angle);
        //             Eigen::Quaterniond quat(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());
        //             Eigen::Vector3d t(pose.position.x, pose.position.y, pose.position.z);

        //             Sophus::SE3d se3(quat, t);
        //             mMap.mpKeyFrames.at(i)->mImage.image_parts.at(j).mCamera.mTwc = se3;
        //         }

        //         poses.push_back(pose);
        //         vslam::Viz viz;
        //         publishVslam(viz, mMap.mpKeyFrames.at(i));
        //     }

        //     visualization_msgs::MarkerArray markers;
        //     VisLoopCameraWorld(markers, poses);
        //     pub_LoopCameraRviz.publish(markers);

        //     std::vector<geometry_msgs::Pose> poses2;
        //     std::vector<std::string> strs;
        //     for (int i = 0; i < 128; i++)
        //     {
        //         if (mMap.mpMapPoints.at(i) != nullptr)
        //         {
        //             KeyFrame *pKF = mMap.mpMapPoints.at(i)->mpKeyFrames.at(0).first;
        //             Eigen::Vector3d vec = pKF->mImage.image_parts.at(0).mCamera.camera2world(mMap.mpMapPoints.at(i)->mCard.pose);
        //             geometry_msgs::Pose pose;
        //             std::string str;
        //             pose.position.x = vec(0, 0);
        //             pose.position.y = vec(1, 0);
        //             pose.position.z = 0;
        //             str = mMap.mpMapPoints.at(i)->mCard.code_id;
        //             poses2.push_back(pose);
        //             strs.push_back(str);
        //         }
        //     }
        //     visualization_msgs::MarkerArray markers2;
        //     VisLoopCardWorld(markers2, poses2);
        //     pub_LoopCardRviz.publish(markers2);

        //     visualization_msgs::MarkerArray markers3;
        //     VisLoopIdWorld(markers3, poses2, strs);
        //     pub_LoopIdRviz.publish(markers3);
        // }

        pub_PoseEKFRviz.publish(VisPosEKF());
        pub_CardEKFRviz.publish(VisCardEKF());
        pub_CardPositionRviz.publish(VisCardPosition());
        pub_IdPositionRviz.publish(VisIdPosition());
        pub_slam.publish(publishVslam());


        //debug
        //cv::imshow("left", m_CurrentImageMat[0]);
        cv::imshow("front", m_CurrentImageMat[1]);

        cv::waitKey(1);

        // loop_rate.sleep();
    }
}
