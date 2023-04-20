#include "semi_dense/param.hpp"
#include "semi_dense/EdgeSE3ProjectDirect.hpp"
#include "semi_dense/callback.hpp"

struct Measurement
{
    Measurement ( Eigen::Vector3d p, float g ) : pos_world ( p ), grayscale ( g ) {}
    Eigen::Vector3d pos_world;
    float grayscale;
};

class Semi_Direct: public CAMERA_INTRINSIC_PARAM
{
    private:
        int id;
        cv::Mat curr_color, curr_depth, curr_gray;
        cv::Mat prev_color;
        Eigen::Matrix3f K;
        Eigen::Isometry3d Tcw;  //Camera Pose(RT)
        std::vector<Measurement> measurements;
    private:
        EdgeSE3ProjectDirect ES3;
        std::mutex data_mtx;
        std::mutex optimi_mtx;
    public:    
        void initalize(std::vector<double>& c_i);
        bool Get_data(SYNC::CALLBACK** _data);
        inline Eigen::Vector3d project2Dto3D(int x, int y, int d, float fx, float fy, float cx, float cy, float scale);
        inline Eigen::Vector2d project3Dto2D ( float x, float y, float z, float fx, float fy, float cx, float cy );

        void compute_gradient(bool* f_i);
        void Display_Feature();
        bool PoseEstimationDirect();
        void runloop(SYNC::CALLBACK** _data);

    public:
        Semi_Direct(std::vector<double>& cam_intrinsic, SYNC::CALLBACK* data);
        ~Semi_Direct(){};
};


