#ifndef LOADER_H
#define LOADER_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include <exception>
#include <string>

#include "semi_dense/param.hpp"
#include "semi_dense/Frame.hpp"

class Exception : public std::exception{
protected:
    std::string name_;
public:
    Exception(std::string name = "") : std::exception(), name_(name) {}
    virtual ~Exception() throw() {}
    virtual const char* what() const noexcept { return name_.c_str(); }
};

class DBLoader : public GET_PARAM
{
protected:
    std::string this_name;
public:
    std::vector<DF> frames;
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    void Get_ground_data(const std::string& path);
    void Get_Image_data(const int img_cnt);
    void Get_Image_data(const std::string& path);
public:
    DBLoader(const int img_cnt){
            std::cout << "Road Data [" << img_cnt << "]!" << std::endl;
            this->Get_Image_data(img_cnt);
    }
    DBLoader(const std::string& path):
    this_name("DBLoader"){
            std::cout << "DB Road processing.." << std::endl;
            this->Get_Image_data(path);
            // this->Get_ground_data(path);
    }
    ~DBLoader(){    
        std::vector<DF>().swap(frames);
    }
};

#endif