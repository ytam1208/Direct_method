#ifndef __SEMIDENSE__
#define __SEMIDENSE__

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

class Odom
{
public:
    Eigen::Isometry3d Tcw;
    double x,y,z,theta;
    void addMotion(Eigen::Isometry3d& _Tcw){
        Eigen::Quaterniond Rot(_Tcw.rotation());
        Eigen::Isometry3d _Twr(Eigen::Quaterniond(Rot.w(), Rot.x(), Rot.y(), Rot.z()));
        _Twr.pretranslate(Eigen::Vector3d(_Tcw.translation()(0), _Tcw.translation()(1), _Tcw.translation()(2)));
        Tcw = _Twr * Tcw;        
    }
    Odom(){
        Eigen::Isometry3d _Twr(Eigen::Quaterniond(-0.3909, 0.8851, 0.2362, -0.0898));
        _Twr.pretranslate(Eigen::Vector3d(1.3112, 0.8507, 1.5186));
        Tcw = _Twr;
    }
    ~Odom(){}
};


class Semi_Direct: public CAMERA_INTRINSIC_PARAM
{
    protected:
        std::string this_name;
    private:
        int idx;
        cv::Mat curr_color, curr_depth, curr_gray;
        cv::Mat prev_color;
        Eigen::Matrix3f K;
        Eigen::Vector4f trans;
        Eigen::Quaternionf rotation;
        Eigen::Isometry3d Tcw;  //Camera Pose(RT)
        std::vector<Measurement> measurements;
        std::vector<std::thread> thread_list;
    public:
        std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    private:
        Odom om;
        EdgeSE3ProjectDirect ES3;
        std::mutex data_mtx;
    public:    
        inline Eigen::Vector3d project2Dto3D(int x, int y, int d, float fx, float fy, float cx, float cy, float scale);
        inline Eigen::Vector2d project3Dto2D ( float x, float y, float z, float fx, float fy, float cx, float cy );
        void compute_gradient(bool* f_i);
        void Display_Feature();
        bool PoseEstimationDirect();

    public:
#ifdef __ROS__
        bool Get_data(SYNC::CALLBACK* _data);
        void runloop(std::unique_ptr<SYNC::CALLBACK>& _data);
        Semi_Direct(std::vector<double>& cam_intrinsic, std::unique_ptr<SYNC::CALLBACK>& data);
#else
        bool Get_data(DF& _data);
        void runloop(std::unique_ptr<DBLoader>& _data);
        Semi_Direct(std::vector<double>& c_i, std::unique_ptr<DBLoader>& data);
#endif
    public:
        ~Semi_Direct(){};
};

#endif
