#ifndef __SEMIDENSE__
#define __SEMIDENSE__

#include "semi_dense/odometer.hpp"
#include "semi_dense/param.hpp"
#include "semi_dense/EdgeSE3ProjectDirect.hpp"
#include "semi_dense/callback.hpp"
#include "semi_dense/Loader.hpp"
#include "semi_dense/plotTrajectory.hpp"

#define getName(VariableName) # VariableName 

struct Measurement
{
    Measurement ( Eigen::Vector3d p, float g ) : pos_world ( p ), grayscale ( g ) {}
    Eigen::Vector3d pos_world;
    float grayscale;
};

class Semi_Direct: public CAMERA_INTRINSIC_PARAM
{
    protected:
        std::string this_name;
    private:
        int idx;
        cv::Mat curr_color, curr_depth, curr_gray;
        cv::Mat prev_color, prev_depth, prev_gray;
        Eigen::Matrix3f K;
        Eigen::Vector4f trans;
        Eigen::Quaternionf rotation;
        // Eigen::Isometry3d Tcw;  //Camera Pose(RT)
        OdomManager om;
        std::vector<Measurement> measurements;
        std::vector<std::thread> thread_list;
    public:
        std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    private:
        EdgeSE3ProjectDirect ES3;
    public:    
        inline Eigen::Vector3d project2Dto3D(int x, int y, int d, float fx, float fy, float cx, float cy, float scale);
        inline Eigen::Vector2d project3Dto2D ( float x, float y, float z, float fx, float fy, float cx, float cy );
        void compute_gradient();
        bool PoseEstimationDirect(Eigen::Isometry3d& Tcw);
        void Display_Feature(Eigen::Isometry3d& Tcw);

    public:
        bool Get_data(DF& _data);
        void runloop(std::unique_ptr<DBLoader>& _data);
    public:
        Semi_Direct(std::vector<double>& c_i, std::unique_ptr<DBLoader>& data);
        ~Semi_Direct(){};
};

#endif
