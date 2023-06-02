#include "semi_dense/Loader.hpp"

void DBLoader::Get_ground_data(const std::string& ground_path){
    std::ifstream fin_G(ground_path);
    int cnt = 0;
    if(fin_G.is_open()){
      try{
        while(!fin_G.eof()){
            if(cnt > 100) break;
            float time, tx, ty, tz, qx, qy, qz, qw;
            fin_G >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
            Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));
            Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
            poses.push_back(Twr);
            cnt++;
        }
      }
      catch(...){
        fin_G.close();
        throw Exception("Process exception[" + this_name + "][Get_ground_data] check your input data! [Error code] std::bad_alloc");
      }
      fin_G.close();
    }
    else
        throw Exception("Process exception[" + this_name + "][Get_ground_data] File not found");  
}

void DBLoader::Get_Image_data(){
    std::string path_to_dataset = "/home/cona/Direct_method/data";
    std::string associate_file = path_to_dataset + "/associate.txt";
    std::string rgb_file, depth_file, time_rgb, time_depth;
    cv::Mat color, depth, gray;
    DF frame;
    std::ifstream fin(associate_file);
    while(!fin.eof()){
        fin >> time_rgb >> rgb_file >> time_depth >> depth_file;
        frame.color = cv::imread(path_to_dataset + "/" + rgb_file);
        frame.depth = cv::imread(path_to_dataset + "/" + depth_file, -1);
        if (frame.color.data==nullptr || frame.depth.data==nullptr)
            continue; 
        cv::cvtColor (frame.color, frame.gray, cv::COLOR_BGR2GRAY);
        frames.push_back(frame);
    }
}

void DBLoader::Get_Image_data(const std::string& ass_path, const std::string& local_path){
    std::string rgb_file, depth_file, time_rgb, time_depth;
    cv::Mat color, depth, gray;
    DF frame;
    std::ifstream fin(ass_path);
    int cnt = 0;
    if(fin.is_open()){
        try{
            while(!fin.eof()){
                if(cnt > 100) break;
                fin >> time_rgb >> rgb_file >> time_depth >> depth_file;
                frame.color = cv::imread(local_path + rgb_file);
                frame.depth = cv::imread(local_path + depth_file, -1);
                if (frame.color.data==nullptr || frame.depth.data==nullptr)
                    continue; 
                cv::cvtColor (frame.color, frame.gray, cv::COLOR_BGR2GRAY);
                frames.push_back(frame);
                cnt++;
            }
            fin.close();
        }
        catch(std::bad_alloc& e){
            fin.close();
            throw Exception("Process exception[" + this_name + "][Get_Image_data] check your input data! [Error code] std::bad_alloc");
        }
        std::cout << "DB Data[" << frames.size() << "] Load!" << std::endl;
        std::cout << "Load!!" << ass_path << std::endl;
    }
    else
        throw Exception("Process exception[" + this_name + "][Get_Image_data] Failed associate.txt!");
    if(frames.empty())
        throw Exception("Process exception[" + this_name + "][Get_Image_data] frames_vector empty!");
}