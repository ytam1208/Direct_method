#include "semi_dense/callback.hpp"

SYNC::CALLBACK::CALLBACK(ros::NodeHandle* _nh):
    nh(*_nh),
    LColor_image_sub(nh, "/pylon/left/image_raw", 1),
    RColor_image_sub(nh, "/pylon/right/image_raw", 1),
    Depth_image_sub(nh, "/camera/depth/image", 1),
    Depth_point_sub(nh, "/camera/depth/points", 1),
    sync(MySyncPolicy(10), LColor_image_sub, RColor_image_sub){
        Camera_info_r = nh.advertise<sensor_msgs::CameraInfo>("/pylon/right/camera_info", 1);
        Camera_info_l = nh.advertise<sensor_msgs::CameraInfo>("/pylon/left/camera_info", 1);
       
        Image_info_sub = nh.subscribe("/camera_info", 1, &SYNC::CALLBACK::Camera_Info_callback, this),
        sync.registerCallback(boost::bind(&SYNC::CALLBACK::Synchronize, this, _1, _2));

        Camera_Info_pub();
}

void SYNC::CALLBACK::Camera_Info_pub(){
    thread_list.push_back(std::move(std::thread([this](){
        ros::Rate rate(0.5);
        sensor_msgs::CameraInfo info_camera;
        while(ros::ok()){
            info_camera.header.stamp = ros::Time::now();
            info_camera.K.assign(0.0);
            info_camera.K[0] = 1155.4288024742;
            info_camera.K[2] = 927.60066037599;
            info_camera.K[4] = 1155.4857727463;
            info_camera.K[5] = 569.32829231548;
            Camera_info_r.publish(info_camera);
            Camera_info_l.publish(info_camera);
            ros::spinOnce();
            rate.sleep();
        }
    })));
    thread_list.push_back(std::move(std::thread([this](){
        ros::Rate rate(160);
        sensor_msgs::CameraInfo info_camera;
        while(ros::ok()){
            
            ros::spinOnce();
            rate.sleep();
        }
    })));
}

void SYNC::CALLBACK::Camera_Info_callback(const sensor_msgs::CameraInfoConstPtr& msg){
    fx = msg->K[0];
    cx = msg->K[2];
    fy = msg->K[4];
    cy = msg->K[5];
    depth_scale = 1000.0f;
    ROS_INFO("Load Camera Info[fx = %lf][fy = %lf][cx = %lf][cy = %lf]", fx, fy, cx, cy);
    Image_info_sub.shutdown();
}

void SYNC::CALLBACK::Convert_Pcl2_to_XYZ(const sensor_msgs::PointCloud2ConstPtr& Input_img){
    try{
        if(Input_img == nullptr) throw;
        int point_size = Input_img->width * Input_img->height;
        // Curr_Dp_mat = cv::Mat::zeros(Input_img->height, Input_img->width, CV_32FC1);

        if(point_size < 10) throw; 
        sensor_msgs::PointCloud2ConstIterator<float> iter_x((*Input_img), "x");
        sensor_msgs::PointCloud2ConstIterator<float> iter_y((*Input_img), "y");
        sensor_msgs::PointCloud2ConstIterator<float> iter_z((*Input_img), "z");

        cv::Point3f Depth;
        for(int iter = 0; iter < point_size && iter_x!=iter_x.end() && iter_y!=iter_y.end() && iter_z!=iter_z.end(); ++iter, ++iter_x, ++iter_y, ++iter_z){
            Depth.x = *iter_x;
            Depth.y = *iter_y;
            Depth.z = *iter_z;
            if(std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)){
                Depth.x = 0.0f;             
                Depth.y = 0.0f;             
                Depth.z = 0.0f;             
            }
            Depth_pt_v.push_back(Depth);
        }
    }
    catch(std::runtime_error e){
        ROS_ERROR("Error exception[pcl::toROSMsg, Convert_Pcl2_to_XYZ]: %s", e.what());
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

void SYNC::CALLBACK::Synchronize(const sensor_msgs::ImageConstPtr& l_image, 
                                    const sensor_msgs::ImageConstPtr& r_image){
    cv_bridge::CvImagePtr l_imgPtr = Convert_Image(l_image);
    cv_bridge::CvImagePtr r_imgPtr = Convert_Image(r_image);

    if(!l_imgPtr->image.empty() && !r_imgPtr->image.empty()){
        // Curr_L_mat = l_imgPtr->image.clone();
        // Curr_R_mat = r_imgPtr->image.clone();

        cv::resize( l_imgPtr->image, Curr_L_mat, cv::Size( 640, 480 ), 0, 0, CV_INTER_NN );
        cv::resize( r_imgPtr->image, Curr_R_mat, cv::Size( 640, 480 ), 0, 0, CV_INTER_NN );

        Curr_D_mat = DM.test(Curr_L_mat, Curr_R_mat);
    }
    else
        ROS_INFO("No data");
}
