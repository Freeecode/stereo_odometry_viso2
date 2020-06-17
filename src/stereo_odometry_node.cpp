/**
 * @brief ros node for viso2
 * 
 */

#include <iostream>
#include <queue>
#include <mutex>
#include "System.h"

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

cv::Mat left, right;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> approx_sync;
typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> extract_sync;

class ImageGrabber
{
public:
    ImageGrabber(System* pSystem):mpSys_(pSystem)
    {
    }

    void GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight);

    System* mpSys_;
};

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr& msgLeft,const sensor_msgs::ImageConstPtr& msgRight)
{
    cv_bridge::CvImageConstPtr left_ptr;
    cv_bridge::CvImageConstPtr right_ptr;
    try
    {
        left_ptr = cv_bridge::toCvShare(msgLeft, sensor_msgs::image_encodings::MONO8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    try
    {
        right_ptr = cv_bridge::toCvShare(msgRight, sensor_msgs::image_encodings::MONO8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    double timestamp = msgLeft->header.stamp.toSec();
    left = left_ptr->image.clone();
    right = right_ptr->image.clone();
    
    mpSys_->TrackStereo(left, right, timestamp);
    mpSys_->pubOdometry();
    mpSys_->pubTF();
    mpSys_->pubPath();
    mpSys_->pubPointCloud();
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "stereo_odometry");
    ros::NodeHandle nh("~");
    
    System StereoVO(nh);
    ImageGrabber igb(&StereoVO);

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 100);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 100);
    message_filters::Synchronizer<approx_sync> sync(approx_sync(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));
    
    ros::Rate loop(30);
    while (ros::ok())
    {
        ros::spinOnce();
        loop.sleep();  
    }

    return 0;
}