#include "semi_dense/semi_dense.hpp"

Semi_Direct::Semi_Direct(std::vector<double>& c_i, std::unique_ptr<DBLoader>& data, Eigen::Isometry3d& _Tcw):
this_name("Semi_Direct"), idx(0){
    cx = c_i[0];
    cy = c_i[1];
    fx = c_i[2];
    fy = c_i[3];
    depth_scale = c_i[4];
    srand((unsigned int) time(0));
    K << fx,  0.f, cx,
        0.f,  fy,  cy,
        0.f, 0.f, 1.0f;
    om.Tcw = _Tcw;
}
Semi_Direct::Semi_Direct(SYNC::CALLBACK& data):
this_name("Semi_Direct"), idx(0){
    cx = data.cx;
    cy = data.cy;
    fx = data.fx;
    fy = data.fy;
    depth_scale = data.depth_scale;
    srand((unsigned int) time(0));
    K << fx,  0.f, cx,
        0.f,  fy,  cy,
        0.f, 0.f, 1.0f;
    om.Tcw = Eigen::Isometry3d::Identity();
}

bool Semi_Direct::Get_data(DF& _data){
    if(!(_data.color.empty()) && !(_data.depth.empty())){
        curr_color = (_data).color.clone();
        curr_depth = (_data).depth.clone();
        cv::cvtColor(curr_color, curr_gray, cv::COLOR_BGR2GRAY);
        return true;
    }
    else{
        ROS_ERROR("[Semi_Direct][Get_data] No Get data!! Curr_C_mat[%d], Curr_D_mat[%d]", 
        (_data).color.empty(), (_data).depth.empty());
        return false;
    }
}

bool Semi_Direct::Get_data(SYNC::CALLBACK& data){
    if(!(data.Curr_L_mat.empty()) && !(data.Curr_D_mat.empty())){
        curr_color = data.Curr_L_mat.clone();
        curr_gray = data.Curr_L_mat.clone();
        curr_depth = data.Curr_D_mat.clone();
        return true;
    }
    else{
        ROS_ERROR("[Semi_Direct][Get_data] No Get data!! Curr_C_mat[%d], Curr_D_mat[%d]", 
        data.Curr_L_mat.empty(), data.Curr_D_mat.empty());
        return false;
    }
}

void Semi_Direct::runloop(std::unique_ptr<DBLoader>& _data){
    DBLoader* DB = _data.get();
    bool show_flag = (*DB).show;
    bool first_index = false;
    int index(0);
    // Eigen::Isometry3d _Twr(Eigen::Quaterniond(-0.3909, 0.8851, 0.2362, -0.0898));
    // _Twr.pretranslate(Eigen::Vector3d(1.3112, 0.8507, 1.5186));
    poses.push_back(om.Tcw);

    for(auto frame : DB->frames){
        Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();
        std::cout << "*********** loop " << index << " ************" << std::endl;
        if(!Get_data(frame)) 
            continue;
        else{
            if(index == 0){
                prev_color = curr_color.clone();
                prev_depth = curr_depth.clone();
                prev_gray = curr_gray.clone();
                compute_gradient();
                index++;
                continue;
            }
            else{
                if(index != 1){
                    measurements.clear();
                    compute_gradient();
                }
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                try{
                    PoseEstimationDirect(Tcw);
                }catch(std::bad_alloc& bad){
                    throw Exception("Process exception[" + this_name + "][PoseEstimationDirect] [Error code] std::bad_alloc");
                }catch(Exception& exp){
                    throw Exception("Process exception[" + this_name + "][PoseEstimationDirect] [Error code] std::excpetion");
                }catch(...){
                    throw Exception("Process exception[" + this_name + "][PoseEstimationDirect] [Error code] Other Exception");
                }
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>> (t2-t1);
                std::cout << "direct method costs time: " << time_used.count() << " seconds." << std::endl;
                Display_Feature(Tcw);

                prev_color = curr_color.clone();
                prev_depth = curr_depth.clone();
                prev_gray = curr_gray.clone();
            }
        }
        index++;
    }
}

void Semi_Direct::runloop(SYNC::CALLBACK& data){
    bool show_flag = 1;
    bool first_index = false;
    int index(0);
    poses.push_back(om.Tcw);

    ros::Rate rate(160);
    while(ros::ok()){
        Eigen::Isometry3d Tcw = Eigen::Isometry3d::Identity();
        std::cout << "*********** loop " << index << " ************" << std::endl;
        if(!Get_data(data)) 
            continue;
        else{
            if(index == 0){
                prev_color = curr_color.clone();
                prev_depth = curr_depth.clone();
                prev_gray = curr_gray.clone();
                compute_gradient();
                index++;
                continue;
            }
            else{
                if(index != 1){
                    measurements.clear();
                    compute_gradient();
                }
                std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                try{
                    PoseEstimationDirect(Tcw);
                }catch(std::bad_alloc& bad){
                    throw Exception("Process exception[" + this_name + "][PoseEstimationDirect] [Error code] std::bad_alloc");
                }catch(Exception& exp){
                    throw Exception("Process exception[" + this_name + "][PoseEstimationDirect] [Error code] std::excpetion");
                }catch(...){
                    throw Exception("Process exception[" + this_name + "][PoseEstimationDirect] [Error code] Other Exception");
                }
                std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>> (t2-t1);
                std::cout << "direct method costs time: " << time_used.count() << " seconds." << std::endl;
                Display_Feature(Tcw);

                prev_color = curr_color.clone();
                prev_depth = curr_depth.clone();
                prev_gray = curr_gray.clone();
            }
        }
        index++;
        ros::spinOnce();
        rate.sleep();
    }
}

inline Eigen::Vector3d Semi_Direct::project2Dto3D(int x, int y, int d, float fx, float fy, float cx, float cy, float scale){
    float zz = float(d) / scale;
    float xx = zz * (x-cx) / fx;
    float yy = zz * (y-cy) / fy;
    return Eigen::Vector3d (xx, yy, zz);
}

inline Eigen::Vector2d Semi_Direct::project3Dto2D ( float x, float y, float z, float fx, float fy, float cx, float cy ){
    float u = fx*x/z+cx;
    float v = fy*y/z+cy;
    return Eigen::Vector2d (u,v);
}

void Semi_Direct::compute_gradient(){
    try{
        for(int x=10; x<prev_gray.cols-10; x++)
            for(int y=10; y<prev_gray.rows-10; y++){
                Eigen::Vector2d delta(
                    prev_gray.ptr<uchar>(y)[x+1] - prev_gray.ptr<uchar>(y)[x-1], 
                    prev_gray.ptr<uchar>(y+1)[x] - prev_gray.ptr<uchar>(y-1)[x]
                );
                if(delta.norm() < 50)
                    continue;
                // ushort d = (ushort)curr_depth.at<uchar>(y,x);
                ushort d = 2000;
                if(d==0)
                    continue;
    
                Eigen::Vector3d p3d = project2Dto3D(x, y, d, fx, fy, cx, cy, depth_scale);
                float grayscale = float(prev_gray.ptr<uchar>(y)[x]);
                measurements.push_back(Measurement(p3d, grayscale));
            }
    }
    catch(std::bad_alloc& e){
        throw Exception("Process exception[" + this_name + "][compute_gradient] check your input data! [Error code] std::bad_alloc");
    }
    catch(Exception& e){
        throw Exception("Process exception[" + this_name + "][compute_gradient] check your input data! [Error code] std::exception");
    }
    catch(...){
        throw Exception("Process exception[" + this_name + "][compute_gradient] check your input data! [Error code] Other Exception");
    }
}

bool Semi_Direct::PoseEstimationDirect(Eigen::Isometry3d& Tcw){
    if(curr_gray.empty())
        throw Exception("Process exception[" + this_name + "][PoseEstimationDirect] check your input data! [Error code] std::exception");
    std::cout << "in" << std::endl;
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;  
    DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense<DirectBlock::PoseMatrixType>();
    DirectBlock* solver_ptr = new DirectBlock (linearSolver);
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::move(solver_ptr) ); // G-N
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr)); // L-M

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setEstimate (g2o::SE3Quat(Tcw.rotation(), Tcw.translation()));
    pose->setId(0);
    optimizer.addVertex(pose);

    int id(1);
    for (Measurement m: measurements){
        EdgeSE3ProjectDirect* edge = new EdgeSE3ProjectDirect(
            m.pos_world,
            K(0,0), K(1,1), K(0,2), K(1,2), &curr_gray
        );
        edge->setVertex (0, pose);
        edge->setMeasurement (m.grayscale);
        edge->setInformation (Eigen::Matrix<double,1,1>::Identity());
        edge->setId (id++);
        optimizer.addEdge (edge);
    }    
    // std::cout << "edges in graph: "<<optimizer.edges().size() << std::endl;
    optimizer.initializeOptimization();
    optimizer.optimize(30);
    Tcw = pose->estimate();

    // Eigen::Isometry3d relative_pose = om.getMotion(Tcw);
    Eigen::Isometry3d Update_odom = om.addMotion(om.Tcw, Tcw);
    om.Tcw = Update_odom;
    om.print_RT(Tcw);
    // om.print_RT(om.Tcw);
    poses.push_back(om.Tcw);   
    return true;
}

void Semi_Direct::Display_Feature(Eigen::Isometry3d& Tcw){
    try{
        if(prev_color.empty() || curr_color.empty() || curr_depth.empty())    
            throw Exception("Process exception[" + this_name + "][Display_Feature] [Error_code] std::bad_alloc");
        cv::Mat img_show(curr_color.rows*2, curr_color.cols, CV_8UC3 );
        prev_color.copyTo(img_show(cv::Rect(0,0,curr_color.cols, curr_color.rows)));
        curr_color.copyTo(img_show(cv::Rect(0,curr_color.rows, curr_color.cols, curr_color.rows)));

        for(Measurement m : measurements){
            Eigen::Vector3d prev_p = m.pos_world;
            Eigen::Vector2d pixel_prev = project3Dto2D (prev_p(0,0), prev_p(1,0), prev_p(2,0), fx, fy, cx, cy);
            Eigen::Vector3d curr_p = Tcw*m.pos_world;
            Eigen::Vector2d pixel_curr = project3Dto2D (curr_p(0,0), curr_p(1,0), curr_p(2,0), fx, fy, cx, cy);
            if(pixel_curr(0,0) < 0 || pixel_curr(0,0) >= curr_color.cols || pixel_curr(1,0) <0 || pixel_curr(1,0) >= curr_color.rows)
                continue;

            img_show.ptr<uchar>(pixel_prev(1,0))[int(pixel_prev(0,0))*3] = 0;
            img_show.ptr<uchar>(pixel_prev(1,0))[int(pixel_prev(0,0))*3+1] = 250;
            img_show.ptr<uchar>(pixel_prev(1,0))[int(pixel_prev(0,0))*3+2] = 0;

            img_show.ptr<uchar>(pixel_curr(1,0)+curr_color.rows)[int(pixel_curr(0,0))*3] = 0;
            img_show.ptr<uchar>(pixel_curr(1,0)+curr_color.rows)[int(pixel_curr(0,0))*3+1] = 250;
            img_show.ptr<uchar>(pixel_curr(1,0)+curr_color.rows)[int(pixel_curr(0,0))*3+2] = 0;

            cv::circle(img_show, cv::Point2d (pixel_prev(0,0), pixel_prev(1,0)), 1, cv::Scalar(0,250,0), -1);
            cv::circle(img_show, cv::Point2d (pixel_curr(0,0), pixel_curr(1,0)+curr_color.rows), 1, cv::Scalar(0,250,255), -1);
        }
        cv::putText(img_show, "Prev", cv::Point(10,20), 1, 1, cv::Scalar(0,0,255), 2, 8);
        cv::putText(img_show, "Current", cv::Point(10,20+curr_color.rows), 1, 1, cv::Scalar(0,0,255), 2, 8);
        cv::imshow("Result", img_show);
        cv::waitKey(1);
    }
    catch(std::bad_alloc& e){
        throw Exception("Process exception[" + this_name + "][Display_Feature] check your input data! [Error code] std::bad_alloc");
    }
    catch(Exception& e){
        throw Exception("Process exception[" + this_name + "][Display_Feature] check your input data! [Error code] std::exception");
    }
    catch(...){
        throw Exception("Process exception[" + this_name + "][Display_Feature] check your input data! [Error code] Other Exception");
    }
}