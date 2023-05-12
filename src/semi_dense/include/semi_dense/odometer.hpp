#ifndef __ODOMETER__
#define __ODOMETER__

#include <iostream>
#include <Eigen/Dense>

class Odom
{
public:
    double x,y,z,theta;
    Eigen::Isometry3d Tcw;
    Odom(){
        // Eigen::Isometry3d _Twr(Eigen::Quaterniond(0, 0, 0, 0));
        // _Twr.pretranslate(Eigen::Vector3d(0, 0, 0));
        Eigen::Isometry3d _Twr(Eigen::Quaterniond(-0.3909, 0.8851, 0.2362, -0.0898));
        _Twr.pretranslate(Eigen::Vector3d(1.3112, 0.8507, 1.5186));
        // Eigen::Isometry3d _Twr(Eigen::Quaterniond(-0.2887, 0.2888, 0.6078, -0.6810));
        // _Twr.pretranslate(Eigen::Vector3d(-1.8476, 2.9446, 0.5268));        
        Tcw = _Twr;
    }
    ~Odom(){}
};

class OdomManager : public Odom
{
public:
    void print_RT(Eigen::Isometry3d& _Tcw);
    Eigen::Isometry3d addMotion(Eigen::Isometry3d& Odom, Eigen::Isometry3d& Relative_pose);
    Eigen::Isometry3d getMotion(Eigen::Isometry3d& _Tcw);
    OdomManager(){}
    ~OdomManager(){}
};

#endif