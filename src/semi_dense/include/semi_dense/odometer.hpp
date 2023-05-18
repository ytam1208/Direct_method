#ifndef __ODOMETER__
#define __ODOMETER__

#include <iostream>
#include <Eigen/Dense>

class Odom
{
public:
    double x,y,z,theta;
    Eigen::Isometry3d Tcw;
    Odom(){}
    ~Odom(){}
};

class OdomManager : public Odom
{
public:
    void print_RT(Eigen::Isometry3d& _Tcw);
    Eigen::Isometry3d addMotion(Eigen::Isometry3d& Odom, Eigen::Isometry3d& Relative_pose);
    Eigen::Isometry3d getMotion(Eigen::Isometry3d& _Tcw);
    Eigen::Isometry3d inverse_addMotion(Eigen::Isometry3d& Odom, Eigen::Isometry3d& Relative_pose);
    
    OdomManager(){}
    ~OdomManager(){}
};

#endif