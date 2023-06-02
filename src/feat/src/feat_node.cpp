#include <iostream>
#include <memory>
#include "feat/exception.hpp"
#include "feat/orb_feat.hpp"
#include "feat/Load_KITTI.hpp"
#include "feat/Odometer.hpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "feat_node");
    ros::NodeHandle nh;
    try{
        std::function<void(int)> Get_data = std::bind(&Loader_Kitti::Get_data, &LK, std::placeholders::_1);
        std::function<std::vector<TEST::Frame>(Loader_Kitti&)> frames = &Loader_Kitti::frames;
        Get_data(10);
        Matcher mt(frames);
    }
    catch(MY::Exception& e){
        std::cerr << e.what() << std::endl;
    }

    return 0;
}