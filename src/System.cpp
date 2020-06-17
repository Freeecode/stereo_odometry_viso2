/**
 * 
 */
#include "System.h"

System::System(ros::NodeHandle &nh):nh_(nh)
{
    if(readParameters())
    {
        ROS_INFO_STREAM("Load parameters successfully!");
        //std::cout << im_height << "," << im_width << "\n"
        //          << params_.calib.f << "," << params_.calib.cu << "," << params_.calib.cv << std::endl;
        Initialize();

        // register pub
        pub_odom = nh_.advertise<nav_msgs::Odometry>("Odometry", 100);
        pub_path = nh_.advertise<nav_msgs::Path>("path", 100);
        pub_pointcloud = nh.advertise<sensor_msgs::PointCloud2>("local_pointcloud", 100);
    }
    else
    {
        ROS_ERROR_STREAM("Failed load parameters!");
        ros::shutdown();
    }
}

bool System::readParameters()
{
    bool get_all = true;

    get_all &= nh_.getParam("cam_width", im_width);
    get_all &= nh_.getParam("cam_height", im_height);
    get_all &= nh_.getParam("cam_fx", params_.calib.f); // fx and fy is same after stereoRectify
    get_all &= nh_.getParam("cam_cx", params_.calib.cu);
    get_all &= nh_.getParam("cam_cy", params_.calib.cv);
    get_all &= nh_.getParam("cam_bf", params_.base);
    get_all &= nh_.getParam("nm_s", nms_);
    //params_.match.nms_n = nms_;
    //std::cout << "nm_s: " << params_.match.nms_n << std::endl; 
    return get_all;
}

void System::Initialize()
{
    ROS_INFO("Initialize VO system...");

    Sys_.reset(new VisualOdometryStereo(params_));
    Twc = Matrix::eye(4);
    //std::cout << Twc << std::endl;
    is_first_run = true;
    dims[0] = im_width;
    dims[1] = im_height;
    dims[2] = im_width;

    //ConverterToEigen();
}


void System::TrackStereo(cv::Mat &imLeft, cv::Mat &imRight, double& timestamp)
{
    if(is_first_run)
    {
        state = Sys_->process(imLeft.data, imRight.data, dims);

        is_first_run = false;    
    }
    else
    {
        state = Sys_->process(imLeft.data, imRight.data, dims);

        if(state)
        {
            Twc = Twc * Matrix::inv(Sys_->getMotion());
            
            // output some statistics
            double num_matches = Sys_->getNumberOfMatches();
            double num_inliers = Sys_->getNumberOfInliers();
            std::cout << "Matches: " << num_matches << ", inliers: " << num_inliers 
                      << ", Ratio: " << 100 * num_inliers / num_matches << " %" << std::endl; 
        }
        else
        {
            ROS_ERROR_STREAM("Tracking Failed...");
        }
    }

    timeStamp_ = timestamp;
    ConverterToEigen();
}

void System::pubOdometry()
{
    Eigen::Quaterniond q(Rwc);

    nav_msgs::Odometry odom;
    odom.header.frame_id = "world";
    odom.child_frame_id = "camera";
    odom.header.stamp = ros::Time().fromSec(timeStamp_);
    odom.pose.pose.orientation.w = q.w();
    odom.pose.pose.orientation.x = q.x();
    odom.pose.pose.orientation.y = q.y();
    odom.pose.pose.orientation.z = q.z();
    odom.pose.pose.position.x = twc.x();
    odom.pose.pose.position.y = twc.y();
    odom.pose.pose.position.z = twc.z();

    pub_odom.publish(odom);
}

void System::pubTF()
{
    Eigen::Quaterniond q(Rwc);

    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = "world";
    transform.child_frame_id = "camera";
    transform.header.stamp = ros::Time().fromSec(timeStamp_);
    transform.transform.rotation.w = q.w();
    transform.transform.rotation.x = q.x();
    transform.transform.rotation.y = q.y();
    transform.transform.rotation.z = q.z();
    transform.transform.translation.x = twc.x();
    transform.transform.translation.y = twc.y();
    transform.transform.translation.z = twc.z();

    br.sendTransform(transform);
}

void System::pubPath()
{
    Eigen::Quaterniond q(Rwc);

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time().fromSec(timeStamp_);
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.orientation.w = q.w();
    pose_stamped.pose.orientation.x = q.x();
    pose_stamped.pose.orientation.y = q.y();
    pose_stamped.pose.orientation.z = q.z();
    pose_stamped.pose.position.x = twc.x();
    pose_stamped.pose.position.y = twc.y();
    pose_stamped.pose.position.z = twc.z();

    path_.header.stamp = ros::Time().fromSec(timeStamp_);
    path_.header.frame_id = "world";
    path_.poses.push_back(pose_stamped);
    pub_path.publish(path_);
}

void System::pubPointCloud()
{
    std::vector<Matcher::p_match> matches = Sys_->getMatches();
    std::vector<int> Inlierindex = Sys_->getInlierIndices();

    pcl::PointCloud<pcl::PointXYZ> cloud;
    for(size_t i = 0; i < Inlierindex.size(); i++)
    {
        Matcher::p_match match = matches[Inlierindex[i]];  
        cv::Point2f left_uv;
        left_uv.x = match.u1c;
        left_uv.y = match.v1c;
        // d= uL-uR;
        // z = fb/d;
        float disparity = match.u1c -match.u2c;
        if(disparity > 0.0)
        {
            pcl::PointXYZ point;
            float z = params_.base / disparity;
            point.z = z;
            point.x = z * (left_uv.x - params_.calib.cu) / params_.calib.f;
            point.y = z * (left_uv.y - params_.calib.cv) / params_.calib.f;
            cloud.push_back(point);
        }
    }
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.frame_id = "camera"; // camera
    msg.header.stamp = ros::Time().fromSec(timeStamp_);
    pub_pointcloud.publish(msg);
}

void System::ConverterToEigen()
{
    Rwc << Twc.val[0][0],Twc.val[0][1],Twc.val[0][2],
           Twc.val[1][0],Twc.val[1][1],Twc.val[1][2],
           Twc.val[2][0],Twc.val[2][1],Twc.val[2][2];
    twc << Twc.val[0][3],Twc.val[1][3],Twc.val[2][3];

    //std::cout << "Rot =" << Rwc << std::endl;
    //std::cout << "tran=" << twc << std::endl; 
}

System::~System()
{
}

