#include <pangolin/pangolin.h>
#include <pangolin/display/default_font.h>
#include <Eigen/Core>
#include <unistd.h>
#include "semi_dense/param.hpp"

class Loader
{
    std::string this_name;

public:
    bool Get_data(std::string& trajectory_file, CAMERA_INTRINSIC_PARAM** input);
    void DrawNode(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& poses, int i, float scale);
    void DrawTrajectory(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses, CAMERA_INTRINSIC_PARAM*** input);
    Loader(std::string& path, CAMERA_INTRINSIC_PARAM* input);
    ~Loader(){};
};