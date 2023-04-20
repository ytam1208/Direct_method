#ifndef CALLBACK_H
#define CALLBACK_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <thread>
#include "semi_dense/param.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
namespace SYNC
{
    class CALLBACK : public GET_PARAM
    {
        public:
            cv::Mat Curr_D_mat;
            cv::Mat Curr_C_mat;
        private:
            ros::NodeHandle nh;
            message_filters::Subscriber<sensor_msgs::Image> Color_image_sub;
            message_filters::Subscriber<sensor_msgs::Image> Depth_image_sub;
            message_filters::Synchronizer<MySyncPolicy> sync;

        public:
            void Synchronize(const sensor_msgs::ImageConstPtr& c_image, const sensor_msgs::ImageConstPtr& d_image);
        public:

            CALLBACK(ros::NodeHandle* _nh);
            ~CALLBACK(){}
    };

}

#endif