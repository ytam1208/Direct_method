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
#include <exception>
#include <fstream>
#include <vector>
#include <chrono>
#include <mutex>

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
#include "semi_dense/EdgeSE3ProjectDirect.hpp"
#include "semi_dense/semi_dense.hpp"
#include "semi_dense/plotTrajectory.hpp"

#define For_Mac
int main(int argc, char** argv)
{
#ifdef For_Window   
    ros::init(argc, argv, "semi_dense_node");
	ros::NodeHandle nh("~");
    SYNC::CALLBACK* mc = new SYNC::CALLBACK(&nh);
    XmlRpc::XmlRpcValue* camera_intrinsic = new XmlRpc::XmlRpcValue;

    nh.getParam("/Set_Display", mc->show);
    nh.getParam("/CAMERA_INTRINSIC_PARAM", *camera_intrinsic);

    std::vector<double> cam_intrinsic;
    for(int i = 0; i < camera_intrinsic[0].size(); i++){
        double test = (double)camera_intrinsic[0][i];
        cam_intrinsic.push_back(test);
    }
    Semi_Direct sd(cam_intrinsic, mc);
#endif

    std::string path = "/home/cona/Direct_method/data/freiburg1_xyz.txt";
    CAMERA_INTRINSIC_PARAM* CIP = new CAMERA_INTRINSIC_PARAM(319.5, 239.5, 525.0, 525.0, 1000.0);
    Loader ld(path, CIP);
    return 0;
}