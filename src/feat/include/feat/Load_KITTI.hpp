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
public:
    std::vector<TEST::Frame> frames;
    Depth_Map DM;

    void Get_data(int Load_num){
        int index = -1;
        int MAX_cnt = Load_num;
        frames.reserve(MAX_cnt);
        std::string local_path = "/home/cona/data_odometry_gray/dataset/sequences/";
        for(int i = 0; i < MAX_cnt; i++, index++){
            TEST::Frame frame;
            frame.Left = cv::imread(cv::format("/home/cona/data_odometry_gray/dataset/sequences/00/image_0/%06d.png", index));
            frame.right = cv::imread(cv::format("/home/cona/data_odometry_gray/dataset/sequences/00/image_1/%06d.png", index));
            if(!frame.Left.empty() || !frame.right.empty()){
                // cv::imshow("Left", frame.Left);
                // cv::imshow("Right", frame.right);
                // cv::waitKey(0);
                DM(frame.Left, frame.right, 1, 1);
                frame.depth = DM.depth.clone();
                frames.push_back(frame);
            }
        }
    }
public:
    Loader_Kitti(){}
    ~Loader_Kitti(){}
};
Loader_Kitti LK;
#endif