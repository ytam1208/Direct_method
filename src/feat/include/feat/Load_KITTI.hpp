#ifndef __KITTI__
#define __KITTI__
#include <opencv2/opencv.hpp>
#include "feat/exception.hpp"
#include "feat/disparity.hpp"

namespace TEST
{
    class Frame
    {
    public:
        cv::Mat Left;
        cv::Mat right;
        cv::Mat depth;
        Frame(){}
        ~Frame(){}
    };
}

class Loader_Kitti
{
protected:
    std::string this_name;
public:
    std::vector<TEST::Frame> frames;
    Depth_Map DM;

    void Get_data(int Load_num, const std::string& path){
        if(path.compare("none") == 0)
            throw MY::Exception("Process exception[" + this_name + "][Get_data] [Error code] Fail Path");
        else
            std::cout << "Load Path [" << path << "]" << std::endl;
        int index = 0;
        int MAX_cnt = Load_num;
        frames.reserve(MAX_cnt);

        // std::string local_path = "/home/cona/data_odometry_gray/dataset/sequences/";
        for(int i = 0; i < MAX_cnt; i++, index++){
            TEST::Frame frame;
            frame.Left = cv::imread(cv::format("%s%s%06d.png", path.c_str(), "/image_0/", index));
            frame.right = cv::imread(cv::format("%s%s%06d.png", path.c_str(), "/image_1/", index));
            if(!frame.Left.empty() || !frame.right.empty()){
                DM(frame.Left, frame.right, 1, 1);
                frame.depth = DM.depth.clone();
                frames.push_back(frame);
            }
        }
    }
public:
    Loader_Kitti():this_name("Loader_Kitti"){}
    ~Loader_Kitti(){}
};
Loader_Kitti LK;
#endif