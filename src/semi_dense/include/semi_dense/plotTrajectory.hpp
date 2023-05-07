#ifndef __PANGOLOADER__
#define __PANGOLOADER__

#include <pangolin/pangolin.h>
#include <pangolin/display/default_font.h>
#include <Eigen/Core>
#include <unistd.h>
#include "semi_dense/param.hpp"
#include "semi_dense/Loader.hpp"

namespace Pango
{
    class Loader
    {
        std::string this_name;

    public:
        bool Get_data(std::string& trajectory_file, CAMERA_INTRINSIC_PARAM** input);
        bool Get_data(std::string& trajectory_file, std::vector<double>& input);
        void DrawNode(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& poses, int i, float scale);
        void DrawTrajectory(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& poses, CAMERA_INTRINSIC_PARAM*** input);
        void DrawTrajectory(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& poses, std::vector<double>& input);
        Loader(){}
        Loader(std::string& path, CAMERA_INTRINSIC_PARAM* input);
        Loader(std::string& path, std::vector<double>& input);
        ~Loader(){}
    };
}

#endif