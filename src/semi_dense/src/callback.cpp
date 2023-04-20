#include "semi_dense/callback.hpp"

SYNC::CALLBACK::CALLBACK(ros::NodeHandle* _nh):
    nh(*_nh),
    Color_image_sub(nh, "/astra_camera/color/image_raw", 1),
    Depth_image_sub(nh, "/astra_camera/depth/image_raw", 1),
    sync(MySyncPolicy(10), Color_image_sub, Depth_image_sub){
        sync.registerCallback(boost::bind(&SYNC::CALLBACK::Synchronize, this, _1, _2));
}


void SYNC::CALLBACK::Synchronize(const sensor_msgs::ImageConstPtr& c_image, const sensor_msgs::ImageConstPtr& d_image){
    cv_bridge::CvImagePtr c_imgPtr, d_imgPtr;
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
        if(show){
            cv::imshow("Color", Curr_C_mat);
            cv::imshow("Depth", Curr_D_mat);
            cv::waitKey(1);
        }
    }
    else
        ROS_INFO("No data");
}
