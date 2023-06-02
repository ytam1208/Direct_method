#ifndef __ODOM__
#define __ODOM__
#include <iostream>
#include <Eigen/Dense>

namespace TEST
{
    class OdomManager
    {
    public:
        void print_RT(Eigen::Isometry3d& _Tcw){
            Eigen::Quaterniond R_M(_Tcw.rotation());
            std::cout << "****************************************************" << std::endl;
            std::cout << "T_Matrix= " << _Tcw.translation()(0) << ", " << _Tcw.translation()(1) << ", " << _Tcw.translation()(2) << std::endl;
            std::cout << "Quaternion= " << R_M.x() << ", " << R_M.y() << ", " << R_M.z() << ", " << R_M.w() << std::endl;
            std::cout << "Rotation_Matrix= \n" << _Tcw.rotation() << std::endl;
            std::cout << "****************************************************" << std::endl;
        }

        Eigen::Isometry3d addMotion(Eigen::Isometry3d& Odom, Eigen::Isometry3d& Relative_pose){
            return (Relative_pose * Odom);        
        }
        Eigen::Isometry3d getMotion(Eigen::Isometry3d& _Tcw){
            return (_Tcw.inverse());
        }
        
        OdomManager(){}
        ~OdomManager(){}
    };
}
#endif