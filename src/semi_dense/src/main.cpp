#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Dense>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "semi_dense/callback.hpp"

struct Measurement
{
    Measurement ( Eigen::Vector3d p, float g ) : pos_world ( p ), grayscale ( g ) {}
    Eigen::Vector3d pos_world;
    float grayscale;
};

struct CAMERA_INTRINSIC_PARAM
{
    float cx;
    float cy;
    float fx;
    float fy;
    float depth_scale;
}cip={325.5, 253.5, 518.0, 519.0, 1000.0};



namespace SYNC
{
    class Callback
    {
        public:
            cv::Mat Curr_D_mat;
            cv::Mat Curr_C_mat;
            ros::NodeHandle nh;
            message_filters::Subscriber<sensor_msgs::Image> Color_image_sub;
            message_filters::Subscriber<sensor_msgs::Image> Depth_image_sub;
            message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::Image> sync;
        private:

        public:
            void callback(const sensor_msgs::ImageConstPtr& c_image, const sensor_msgs::ImageConstPtr& d_image){
                cv_bridge::CvImagePtr c_imgPtr, d_imgPtr;
                cv::Mat temp_depth_img;
                ROS_INFO("HI");

                try{
                    c_imgPtr = cv_bridge::toCvCopy(*c_image, c_image->encoding);
                    d_imgPtr = cv_bridge::toCvCopy(*d_image, d_image->encoding);
                }
                catch(...){
                    ROS_INFO("No data");
                }

                if(!c_imgPtr->image.empty() || !d_imgPtr->image.empty()){
                    Curr_C_mat = c_imgPtr->image.clone();
                    Curr_D_mat = d_imgPtr->image.clone();
                    cv::imshow("Color", Curr_C_mat);
                    cv::imshow("Depth", Curr_D_mat);
                    cv::waitKey(1);
                }
                else
                    ROS_INFO("No data");
            }

        public:
            Callback(ros::NodeHandle& _nh):
            sync(Color_image_sub, Depth_image_sub, 10), nh(_nh),
            Color_image_sub(nh, "/astra_camera/color/image_raw", 10),
            Depth_image_sub(nh, "/astra_camera/depth/image_raw", 10)
            {
                ROS_INFO("Loop");
                sync.registerCallback(boost::bind(&SYNC::Callback::callback, this, _1, _2));

            }
            ~Callback(){}
    };

}

void callback(const sensor_msgs::ImageConstPtr& image1, const sensor_msgs::ImageConstPtr& image2)
{
    cv_bridge::CvImagePtr c_imgPtr, d_imgPtr;
    cv::Mat temp_depth_img;

    try{
        c_imgPtr = cv_bridge::toCvCopy(*image1, image1->encoding);
        d_imgPtr = cv_bridge::toCvCopy(*image2, image2->encoding);
    }
    catch(...){
        ROS_INFO("No data");
    }

    if(!c_imgPtr->image.empty() || !d_imgPtr->image.empty()){
        cv::Mat Curr_C_mat = c_imgPtr->image.clone();
        cv::Mat Curr_D_mat = d_imgPtr->image.clone();
        cv::imshow("Color", Curr_C_mat);
        cv::imshow("Depth", Curr_D_mat);
        cv::waitKey(1);
    }
    else
        ROS_INFO("No data");
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "semi_dense_node");
	ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> image_sub1(nh, "/astra_camera/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> image_sub2(nh, "/astra_camera/depth/image_raw", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub1, image_sub2);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // ROS_INFO("Run!");
    // SYNC::Callback mc(nh);
    // srand((unsigned int) time(0));
    // cv::Mat color, depth, gray;
    // std::vector<Measurement> measurements;
    // Eigen::Matrix3f K;
    // K<<cip.fx,
    //     0.f,
    //         cip.cx,
    //             0.f,
    //                 cip.fy,
    //                     cip.cy,
    //                         0.f,
    //                             0.f,
    //                                 1.0f;
    // Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();
	// ros::Rate loop_rate(30);
    // while(ros::ok()){
    //     cv::Mat prev_color;

    //     // ros::spinOnce();
    //     loop_rate.sleep();
    // }
    ros::spin();

    return 0;
}