#ifndef CALLBACK_H
#define CALLBACK_H

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/exceptions.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <thread>



#include "semi_dense/param.hpp"
#include "semi_dense/disparity.hpp"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage> MySyncPolicy_C;

namespace SYNC
{
    class CALLBACK : public GET_PARAM
    {
        protected:
            bool Use_Depth_filter;
            bool Depth_show;
            bool tf_ready;
        public:
            cv::Mat Curr_D_mat;     //depth/image_raw
            cv::Mat Curr_Dp_mat;    //depth/points
            cv::Mat Curr_R_mat;     //right/color/image_raw
            cv::Mat Curr_L_mat;     //left/color/image_raw  
            cv_bridge::CvImagePtr output;

            std::vector<cv::Point3f> Depth_pt_v;
            std::vector<std::thread> thread_list;
        private:
            ros::NodeHandle nh;
            ros::Subscriber Image_info_sub;
            ros::Publisher Camera_info_r, Camera_info_l;
            ros::Publisher Depth_pub;
            
            tf::TransformListener listener;
            Eigen::Matrix3d R_D;
            Eigen::Vector3d T_D;

            message_filters::Subscriber<sensor_msgs::Image> LColor_image_sub, RColor_image_sub;
            message_filters::Subscriber<sensor_msgs::CompressedImage> LC_Color_image_sub, RC_Color_image_sub;
            message_filters::Subscriber<sensor_msgs::Image> Depth_image_sub;
            message_filters::Subscriber<sensor_msgs::PointCloud2> Depth_point_sub;
            
            message_filters::Synchronizer<MySyncPolicy> sync;
            message_filters::Synchronizer<MySyncPolicy_C> sync_c;
            Depth_Map DM;
        public:
            void Camera_Info_pub();
            void Camera_Info_callback(const sensor_msgs::CameraInfoConstPtr& msg);
            void Convert_Pcl2_to_XYZ(const sensor_msgs::PointCloud2ConstPtr& Input_img); 
            cv_bridge::CvImagePtr Convert_Pcl2_to_Image(const sensor_msgs::PointCloud2ConstPtr& Input_img); 
            cv_bridge::CvImagePtr Convert_Image(const sensor_msgs::ImageConstPtr& Input_img); 
            cv_bridge::CvImagePtr Convert_Image(const sensor_msgs::Image& Input_img); 
            cv::Mat Convert_Image(const sensor_msgs::CompressedImageConstPtr& Input_img);

            void Synchronize(const sensor_msgs::ImageConstPtr& l_image, 
                                const sensor_msgs::ImageConstPtr& r_image);
            void Synchronize_C(const sensor_msgs::CompressedImageConstPtr& l_image, 
                                const sensor_msgs::CompressedImageConstPtr& r_image);

            void Get_tf();
        public:

            CALLBACK(ros::NodeHandle* _nh);
            ~CALLBACK(){
                for (int i = 0; i < (int)thread_list.size(); i++)
                    thread_list[i].join();
            }
    };

}

#endif