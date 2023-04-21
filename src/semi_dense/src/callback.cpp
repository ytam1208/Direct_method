#include "semi_dense/callback.hpp"

SYNC::CALLBACK::CALLBACK(ros::NodeHandle* _nh):
    nh(*_nh),
    Color_image_sub(nh, "/astra_camera/color/image_raw", 1),
    Depth_image_sub(nh, "/astra_camera/depth/image_raw", 1),
    Depth_point_sub(nh, "/astra_camera/depth/points", 10),
    sync(MySyncPolicy(10), Color_image_sub, Depth_image_sub, Depth_point_sub){
        sync.registerCallback(boost::bind(&SYNC::CALLBACK::Synchronize, this, _1, _2, _3));
}

cv_bridge::CvImagePtr SYNC::CALLBACK::Convert_Pcl2_to_XYZ(const sensor_msgs::PointCloud2ConstPtr& Input_img){
    try{
        if(Input_img == nullptr) throw;
        int point_size = Input_img->width * Input_img->height;
        if(point_size < 10 ) throw; 
        // std::cout << "Point size = " << point_size << std::endl;
        sensor_msgs::PointCloud2ConstIterator<float> iter_x((*Input_img), "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y((*Input_img), "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z((*Input_img), "z");
        cv::Point3f Depth_pt;
        for(int iter = 0; iter < point_size && iter_x!=iter_x.end() && iter_y!=iter_y.end() && iter_z!=iter_z.end(); ++iter, ++iter_x, ++iter_y, ++iter_z){
            Depth_pt.x = *iter_x;
            Depth_pt.y = *iter_y;
            Depth_pt.z = *iter_z;
            if(std::isnan(Depth_pt.x) || std::isnan(Depth_pt.y) || std::isnan(Depth_pt.z)) 
                continue;
            if(Depth_pt.z < depth_range_min || Depth_pt.z > depth_range_max)
                continue;

            // std::cout << "[" << iter << "] " << Depth_pt.x << ", " << Depth_pt.y << ", " << Depth_pt.z;
        }
        return nullptr;
    }
    catch(std::runtime_error e){
        ROS_ERROR("Error exception[pcl::toROSMsg, Convert_Pcl2_to_XYZ]: %s", e.what());
        return nullptr;
    }
}


cv_bridge::CvImagePtr SYNC::CALLBACK::Convert_Pcl2_to_Image(const sensor_msgs::PointCloud2ConstPtr& Input_img){
    sensor_msgs::Image d_points;
    if((Input_img->width * Input_img->height) == 0 || Input_img->is_dense == false) 
        return nullptr;
    try{
        pcl::toROSMsg (*Input_img, d_points);
    }
    catch(std::runtime_error e){
        ROS_ERROR("Error exception[pcl::toROSMsg, Convert_Pcl2_to_Image]: %s", e.what());
        return nullptr;
    }
    try{
        return cv_bridge::toCvCopy(d_points, d_points.encoding);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("Error exception[cv_bridge::toCvCopy, Convert_Pcl2_to_Image]: %s", e.what());
        return nullptr;
    }    
}

cv_bridge::CvImagePtr SYNC::CALLBACK::Convert_Image(const sensor_msgs::ImageConstPtr& Input_img){
    try{
        return cv_bridge::toCvCopy(*Input_img, Input_img->encoding);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("Error exception[cv_bridge::toCvCopy, Convert_Image_Ptr]: %s", e.what());
        return nullptr;
    }    
}

cv_bridge::CvImagePtr SYNC::CALLBACK::Convert_Image(const sensor_msgs::Image& Input_img){
    try{
        return cv_bridge::toCvCopy(Input_img, Input_img.encoding);
    }
    catch(cv_bridge::Exception& e){
        ROS_ERROR("Error exception[cv_bridge::toCvCopy, Convert_Image]: %s", e.what());
        return nullptr;
    }    
}

void SYNC::CALLBACK::Synchronize(const sensor_msgs::ImageConstPtr& c_image, const sensor_msgs::ImageConstPtr& d_image, const sensor_msgs::PointCloud2ConstPtr& d_points){
    cv_bridge::CvImagePtr c_imgPtr = Convert_Image(c_image);
    cv_bridge::CvImagePtr d_imgPtr = Convert_Image(d_image);
    cv_bridge::CvImagePtr dp_imgPtr = Convert_Pcl2_to_XYZ(d_points);

    if(c_imgPtr != nullptr && d_imgPtr != nullptr){
        Curr_C_mat = c_imgPtr->image.clone();
        Curr_D_mat = d_imgPtr->image.clone();
        // Curr_Dp_mat = dp_imgPtr->image.clone();
        if(show){
            cv::imshow("Color", Curr_C_mat);
            cv::imshow("Depth", Curr_D_mat);
            // cv::imshow("Depth_Point", Curr_Dp_mat);
            cv::waitKey(1);
        }
    }
    else
        ROS_INFO("No data");
}
