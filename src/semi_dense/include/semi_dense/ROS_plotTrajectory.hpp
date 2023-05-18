#include <iostream>
#include <string>
#include <Eigen/Dense>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <geometry_msgs/TransformStamped.h>

#include "semi_dense/Loader.hpp"

class Viewer
{
protected:
    std::string this_name;
public:
    void operator()(const std::string& tf_name, 
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& poses,
    tf2_msgs::TFMessage& tf_list){
        try{
            int node_size = poses.size();
            for(int i = 0; i < node_size; i++){
                Eigen::Quaterniond quat(poses[i].rotation());
                std::string name = tf_name;
                geometry_msgs::TransformStamped tmp;
                tmp.header.stamp = ros::Time::now();
                tmp.header.frame_id = "map";
                tmp.child_frame_id = name + std::to_string(i);

                tmp.transform.rotation.w = quat.w();
                tmp.transform.rotation.x = quat.x();
                tmp.transform.rotation.y = quat.y();
                tmp.transform.rotation.z = quat.z();

                tmp.transform.translation.x = poses[i].translation()(0);
                tmp.transform.translation.y = poses[i].translation()(1);
                tmp.transform.translation.z = poses[i].translation()(2);

                // tmp.transform.translation.x = tmp.transform.translation.x + i;
                tmp.transform.translation.z = 0.0;

                tf_list.transforms.push_back(tmp);
            }   
        }
        catch(std::bad_alloc& bad){
            throw Exception("Process exception[" + this_name + "][Motion to ROS operator] [Error code] std::bad_alloc");
        }catch(Exception e){
            throw Exception("Process exception[" + this_name + "][Motion to ROS operator] [Error code] std::exception");
        }catch(...){
            throw Exception("Process exception[" + this_name + "][Motion to ROS operator] [Error code] Other exception");
        }
    }

    Viewer():this_name("Viewer"){}
};