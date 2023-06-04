#include <iostream>
#include <memory>
#include "feat/exception.hpp"
#include "feat/orb_feat.hpp"
#include "feat/Load_KITTI.hpp"
#include "feat/Odometer.hpp"

#define Mode "Mac"
// #define Mode "Window"

std::string Load_path = "none";
int main(int argc, char** argv)
{
#ifdef Mode == "Mac"
    Load_path = "/home/cona/KITTI/00";
#elif Mode == "Window"
    Load_path = "/home/cona/data_odometry_gray/dataset/sequences/00";
#endif

    ros::init(argc, argv, "feat_node");
    ros::NodeHandle nh;
    try{
        std::function<void(int, const std::string)> Get_data = std::bind(&Loader_Kitti::Get_data, &LK, std::placeholders::_1, std::placeholders::_2);
        std::function<std::vector<TEST::Frame>(Loader_Kitti&)> frames = &Loader_Kitti::frames;
        Get_data(10, Load_path);
        // Matcher mt(frames);
    }
    catch(MY::Exception& e){
        std::cerr << e.what() << std::endl;
    }
    // ros::spin();
    return 0;
}