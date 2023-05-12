#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "tf2_msgs/TFMessage.h"

#include <iostream>
#include <exception>
#include <fstream>
#include <vector>
#include <chrono>
#include <mutex>
#include <string>
#include <exception>

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
#include "semi_dense/Frame.hpp"
#include "semi_dense/Loader.hpp"
#include "semi_dense/EdgeSE3ProjectDirect.hpp"
#include "semi_dense/semi_dense.hpp"
#include "semi_dense/plotTrajectory.hpp"
#include "semi_dense/ROS_plotTrajectory.hpp"

#define __ROS__
// #define __View_Pangoline__
int main(int argc, char** argv)
{
    std::vector<double> cam_intrinsic = {325.5, 253.5, 518.0, 519.0, 1000.0};   //desk
    std::unique_ptr<DBLoader> mc = std::make_unique<DBLoader>("/home/cona/Direct_method/data");        
    // std::vector<double> cam_intrinsic = {319.5, 239.5, 525.0, 525.0, 1000.0};   //pioneer
    // std::unique_ptr<DBLoader> mc = std::make_unique<DBLoader>("/home/cona/rgbd_dataset_freiburg2_pioneer_slam3/");
    mc->show = 1;

#ifdef __ROS__   
    ros::init(argc, argv, "semi_dense_node");
	// ros::NodeHandle nh("~");
    ros::NodeHandle nh1;
    ros::Publisher tf_gt_pub = nh1.advertise<tf2_msgs::TFMessage>("tf", 10);
    ros::Publisher tf_ob_pub = nh1.advertise<tf2_msgs::TFMessage>("tf", 1);
    // XmlRpc::XmlRpcValue* camera_intrinsic = new XmlRpc::XmlRpcValue;
    // nh.getParam("/Set_Display", mc->show);
    // nh.getParam("/CAMERA_INTRINSIC_PARAM", *camera_intrinsic);
#endif
    try{
        Semi_Direct sd(cam_intrinsic, mc);    
        sd.runloop(mc);
#ifdef __View_Pangoline__
        std::string path = "/home/cona/Direct_method/data/test_groundtruth.txt";
        // std::string path = "/home/cona/Direct_method/data/freiburg1_xyz.txt";
        Pango::Loader ld(sd.poses, path, cam_intrinsic);
#endif
#if defined(__ROS__) && !defined(__View_Pangoline__)
        ros::Rate rate(30);
        Viewer vd;

        int idx = 0;
        int Max_size = sd.poses.size();
        while(ros::ok()){
            tf2_msgs::TFMessage tf_list1, tf_list2;

            vd("GT", mc->poses, tf_list1);
            vd("Ob", sd.poses[idx], idx, tf_list2);

            // tf_gt_pub.publish(tf_list1);
            tf_ob_pub.publish(tf_list2);

            idx++;
            ros::spinOnce();
            rate.sleep();
        }
#endif
    }
    catch(Exception& e){
        std::cerr << e.what() << std::endl;
    }

    return 0;
}