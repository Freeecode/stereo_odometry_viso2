/**
 * @brief viso2 interface for ros
 */
#ifndef STEREO_ODOMETRY_H
#define STEREO_ODOMETRY_H

#include <string>
#include <iostream>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/shared_ptr.hpp>
#include <opencv2/opencv.hpp>

#include "viso_stereo.h"

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

class System
{
public:
    System(ros::NodeHandle &nh);

    bool readParameters();

    void Initialize();

    void TrackStereo(cv::Mat &imtLeft, cv::Mat &imRight, double& timestamp);

    ~System();

public:

    void pubTF();

    void pubOdometry();

    void pubPath();

    void pubPointCloud();

    void ConverterToEigen();

protected:

    boost::shared_ptr<VisualOdometryStereo> Sys_;

    VisualOdometryStereo::parameters params_;

    int nms_;

    std::string strSettingfile;

    int im_height;

    int im_width;

    int32_t dims[3];

    // Wold Pose
    Matrix Twc;

    Eigen::Matrix3d Rwc;
    Eigen::Vector3d twc;

    bool is_first_run;
    bool state;

private:
    // ROS
    double timeStamp_;
    
    ros::NodeHandle &nh_;

    nav_msgs::Path path_;

    ros::Publisher pub_path;

    ros::Publisher pub_odom;

    ros::Publisher pub_pointcloud;

    // tf 
    tf::TransformBroadcaster br;

};

#endif