#ifndef LOADER_H
#define LOADER_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <exception>
#include <string>

#include "semi_dense/param.hpp"
#include "semi_dense/Frame.hpp"

class Exception : public std::exception {
    public:
        Exception(std::string name = "") : std::exception(), name_(name) {}
        virtual ~Exception() throw() {}
        virtual const char* what() const noexcept { return name_.c_str(); }
    protected:
        std::string name_;
};

class DBLoader : public GET_PARAM
{
private:
    std::string this_name;
    bool Load_flag;
public:
    std::vector<DF> frames;
    bool Get_Image_data(const int img_cnt);
    bool Get_Image_data(const std::string& path);
public:
    DBLoader(const int img_cnt){
            std::cout << "Road Data [" << img_cnt << "]!" << std::endl;
            Load_flag = this->Get_Image_data(img_cnt);
    }
    DBLoader(const std::string& path):
    this_name("DBLoader"), Load_flag(false){
            std::cout << "DB Road processing.." << std::endl;
            Load_flag = this->Get_Image_data(path);
            if(!Load_flag)
                throw Exception("Process exception[" + this_name + "][Get_Image_data]");
    }
    ~DBLoader(){    
        std::vector<DF>().swap(frames);
    }
};

#endif