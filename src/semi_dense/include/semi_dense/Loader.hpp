#ifndef LOADER_H
#define LOADER_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include "semi_dense/param.hpp"
#include "semi_dense/Frame.hpp"

class DBLoader : public GET_PARAM
{
        public:
            std::vector<DF> frames;
            void Get_Image_data(const int img_cnt);
        public:
            DBLoader(const int img_cnt){
                    std::cout << "Road Data [" << img_cnt << "]!" << std::endl;
                    this->Get_Image_data(img_cnt);
            }
            ~DBLoader(){    
                std::vector<DF>().swap(frames);
            }
};

#endif