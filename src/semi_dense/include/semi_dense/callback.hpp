#ifndef CALLBACK_H
#define CALLBACK_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <image_transport/image_transport.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>
#include "semi_dense/param.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
namespace SYNC
{
    class CALLBACK : public GET_PARAM
    {
        public:
            cv::Mat Curr_D_mat;     //astra_camera/depth/image_raw
            cv::Mat Curr_Dp_mat;    //astra_camera/depth/points
            cv::Mat Curr_C_mat;     //astra_camera/color/image_raw
        private:
            ros::NodeHandle nh;
            message_filters::Subscriber<sensor_msgs::Image> Color_image_sub;
            message_filters::Subscriber<sensor_msgs::Image> Depth_image_sub;
            message_filters::Subscriber<sensor_msgs::PointCloud2> Depth_point_sub;
            message_filters::Synchronizer<MySyncPolicy> sync;

        public:
            cv_bridge::CvImagePtr Convert_Pcl2_to_XYZ(const sensor_msgs::PointCloud2ConstPtr& Input_img); 
            cv_bridge::CvImagePtr Convert_Pcl2_to_Image(const sensor_msgs::PointCloud2ConstPtr& Input_img); 
            cv_bridge::CvImagePtr Convert_Image(const sensor_msgs::ImageConstPtr& Input_img); 
            cv_bridge::CvImagePtr Convert_Image(const sensor_msgs::Image& Input_img); 

            void Synchronize(const sensor_msgs::ImageConstPtr& c_image, const sensor_msgs::ImageConstPtr& d_image, const sensor_msgs::PointCloud2ConstPtr& d_points);
        public:

            CALLBACK(ros::NodeHandle* _nh);
            ~CALLBACK(){}
    };

}

#endif