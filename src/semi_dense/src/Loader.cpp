#include "semi_dense/Loader.hpp"

void DBLoader::Get_Image_data(const int img_cnt){
    std::string path_to_dataset = "/home/cona/Direct_method/data";
    std::string associate_file = path_to_dataset + "/associate.txt";
    std::string rgb_file, depth_file, time_rgb, time_depth;
    cv::Mat color, depth, gray;
    DF frame;
    frames.reserve(img_cnt);
    std::ifstream fin ( associate_file );
    for(int index = 0; index < img_cnt; index++){
        fin >> time_rgb >> rgb_file >> time_depth >> depth_file;
        frame.color = cv::imread(path_to_dataset + "/" + rgb_file);
        frame.depth = cv::imread(path_to_dataset + "/" + depth_file, -1);
        if (frame.color.data==nullptr || frame.depth.data==nullptr)
            continue; 
        cv::cvtColor (frame.color, frame.gray, cv::COLOR_BGR2GRAY);
        frames.push_back(frame);
    }
}

void DBLoader::Get_Image_data(const std::string& path){
    // std::string path_to_dataset = "/home/cona/Direct_method/data";
    std::string associate_file = path + "/associate.txt";
    std::string rgb_file, depth_file, time_rgb, time_depth;
    cv::Mat color, depth, gray;
    DF frame;
    std::ifstream fin(associate_file);
    if(fin.is_open()){
        try{
            // while(!fin.eof()){
            //     fin >> time_rgb >> rgb_file >> time_depth >> depth_file;
            //     frame.color = cv::imread(path + "/" + rgb_file);
            //     frame.depth = cv::imread(path + "/" + depth_file, -1);
            //     if (frame.color.data==nullptr || frame.depth.data==nullptr)
            //         continue; 
            //     cv::cvtColor (frame.color, frame.gray, cv::COLOR_BGR2GRAY);
            //     frames.push_back(frame);
            // }
            for(int i = 0; i < 20; i++){
                fin >> time_rgb >> rgb_file >> time_depth >> depth_file;
                frame.color = cv::imread(path + "/" + rgb_file);
                frame.depth = cv::imread(path + "/" + depth_file, -1);
                if (frame.color.data==nullptr || frame.depth.data==nullptr)
                    continue; 
                cv::cvtColor (frame.color, frame.gray, cv::COLOR_BGR2GRAY);
                frames.push_back(frame);
            }
            fin.close();
        }
        catch(std::bad_alloc& e){
            fin.close();
            throw Exception("Process exception[" + this_name + "][Get_Image_data] check your input data! [Error code] std::bad_alloc");
        }
        std::cout << "DB Data[" << frames.size() << "] Load!" << std::endl;
    }
    else
        throw Exception("Process exception[" + this_name + "][Get_Image_data] Failed associate.txt!");
    if(frames.empty())
        throw Exception("Process exception[" + this_name + "][Get_Image_data] frames_vector empty!");
}