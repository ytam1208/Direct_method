#include "semi_dense/semi_dense.hpp"

#ifdef __ROS__
Semi_Direct::Semi_Direct(std::vector<double>& cam_intrinsic, std::unique_ptr<SYNC::CALLBACK>& data):id(1){
    initalize(cam_intrinsic);
    runloop(data);
}

bool Semi_Direct::Get_data(SYNC::CALLBACK* _data){
    if(!(_data)->Curr_C_mat.empty() && !(_data)->Curr_D_mat.empty()){
        data_mtx.lock();
        curr_color = (_data)->Curr_C_mat.clone();
        curr_depth = (_data)->Curr_D_mat.clone();
        cv::cvtColor(curr_color, curr_gray, cv::COLOR_BGR2GRAY);
        // measurements.clear();
        data_mtx.unlock();
        return true;
    }
    else{
        ROS_ERROR("[Semi_Direct][Get_data] No Get data!! Curr_C_mat[%d], Curr_D_mat[%d]", 
        (_data)->Curr_C_mat.empty(), (_data)->Curr_D_mat.empty());
        return false;
    }
}

void Semi_Direct::runloop(std::unique_ptr<SYNC::CALLBACK>& _data){
    bool show_flag = (_data)->show;
    bool first_index = false;

    ros::Rate Direct_Lr(30);
    std::thread Direct_Method([&](){
        while(ros::ok()){
                if(!Get_data(_data.get())){}
                else{
                    compute_gradient(&first_index);
                    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
                    PoseEstimationDirect();
                    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
                    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>> (t2-t1);
                    std::cout << "direct method costs time: " << time_used.count() << " seconds." << std::endl;
                    if(show_flag) Display_Feature();
                }
            ros::spinOnce();
            Direct_Lr.sleep();
        }
    });  

    // ros::Rate Visual_Lr(10);
    // std::thread Visualize([&](){
    //     while(ros::ok()){
    //         ros::spinOnce();
    //         Visual_Lr.sleep();
    //     }
    // });  

    Direct_Method.join();
    // Visualize.join();
}
#else
Semi_Direct::Semi_Direct(std::vector<double>& c_i, std::unique_ptr<DBLoader>& data):
this_name("Semi_Direct"), idx(0){
    cx = c_i[0];
    cy = c_i[1];
    fx = c_i[2];
    fy = c_i[3];

    srand((unsigned int) time(0));
    K << fx,  0.f, cx,
        0.f,  fy,  cy,
        0.f, 0.f, 1.0f;
    Tcw = Eigen::Isometry3d::Identity();
}

void Semi_Direct::runloop(std::unique_ptr<DBLoader>& _data){
    DBLoader* DB = _data.get();
    bool show_flag = (*DB).show;
    bool first_index = false;
    int index(0);
    for(auto frame : DB->frames){
        Get_data(frame);
        std::cout << index << std::endl;
        if(index == 0)
            compute_gradient(nullptr);
        else{
            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
            PoseEstimationDirect();
            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>> (t2-t1);
            std::cout << "direct method costs time: " << time_used.count() << " seconds." << std::endl;
        }
        index++;
    }
}

bool Semi_Direct::Get_data(DF& _data){
    if(!(_data.color.empty()) && !(_data.depth.empty())){
        data_mtx.lock();
        curr_color = (_data).color.clone();
        curr_depth = (_data).depth.clone();
        cv::cvtColor(curr_color, curr_gray, cv::COLOR_BGR2GRAY);
        // measurements.clear();
        data_mtx.unlock();

        cv::imshow("color", curr_color);
        cv::imshow("gray", curr_gray);
        cv::imshow("depth", curr_depth);
        cv::waitKey(0);
        return true;
    }
    else{
        ROS_ERROR("[Semi_Direct][Get_data] No Get data!! Curr_C_mat[%d], Curr_D_mat[%d]", 
        (_data).color.empty(), (_data).depth.empty());
        return false;
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
    return Eigen::Vector2d ( u,v );
}

void Semi_Direct::compute_gradient(bool* f_i){
    try{
        for(int x=10; x<curr_gray.cols-10; x++)
            for(int y=10; y<curr_gray.rows-10; y++){
                Eigen::Vector2d delta(
                    curr_gray.ptr<uchar>(y)[x+1] - curr_gray.ptr<uchar>(y)[x-1], 
                    curr_gray.ptr<uchar>(y+1)[x] - curr_gray.ptr<uchar>(y-1)[x]
                );
                if(delta.norm() < 50)
                    continue;
                ushort d = curr_depth.at<ushort>(y,x);
                // ushort d = curr_depth.ptr<ushort> (y)[x];
                if(d==0.0f)
                    continue;
                Eigen::Vector3d p3d = project2Dto3D(x, y, d, fx, fy, cx, cy, depth_scale);
                float grayscale = float(curr_gray.ptr<uchar>(y)[x]);
                measurements.push_back(Measurement(p3d, grayscale));
            }
        prev_color = curr_color.clone();
    }
    catch(std::bad_alloc& e){
        throw Exception("Process exception[" + this_name + "][compute_gradient] check your input data! [Error code] std::bad_alloc");
    }
}

bool Semi_Direct::PoseEstimationDirect(){
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

    int id(0);
    for (Measurement m: measurements){
        EdgeSE3ProjectDirect* edge = new EdgeSE3ProjectDirect(
            m.pos_world,
            K( 0,0 ), K( 1,1 ), K( 0,2 ), K( 1,2 ), &curr_gray
        );
        edge->setVertex (0, pose);
        edge->setMeasurement (m.grayscale);
        edge->setInformation (Eigen::Matrix<double,1,1>::Identity());
        edge->setId (id++);
        optimizer.addEdge (edge);
    }    
    // std::cout << "edges in graph: "<<optimizer.edges().size() << std::endl;
    optimizer.initializeOptimization();
    optimizer.optimize(100);
    Tcw = pose->estimate();
    poses.push_back(Tcw);
    // trans = Eigen::Vector4f(Tcw.translation()(0), Tcw.translation()(1), Tcw.translation()(2), 1.0f);

    std::cout << "******************************" << std::endl;
    std::cout << "[ID:" << id << "] Translation= \n" << Tcw.translation() << std::endl;
    std::cout << "******************************" << std::endl;
    std::cout << "[ID:" << id << "] Rotation= \n" << Tcw.rotation() << std::endl;
    std::cout << "******************************" << std::endl;
    return true;
}

#endif