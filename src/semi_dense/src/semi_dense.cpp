#include "semi_dense/semi_dense.hpp"

Semi_Direct::Semi_Direct(std::vector<double>& cam_intrinsic, SYNC::CALLBACK* data):id(1){
    initalize(cam_intrinsic);
    runloop(&data);
}

void Semi_Direct::initalize(std::vector<double>& c_i){
    cx = c_i[0];
    cy = c_i[1];
    fx = c_i[2];
    fy = c_i[3];
    depth_scale = c_i[4];
    ROS_INFO("Initalize Cam_Param Get!!!");
    ROS_INFO("cx=%lf, cy=%lf, fx=%lf, fy=%lf, depth_Scale=%lf", c_i[0], c_i[1], c_i[2], c_i[3], c_i[4]);

    srand((unsigned int) time(0));
    K << fx,  0.f, cx,
        0.f,  fy,  cy,
        0.f, 0.f, 1.0f;

    Tcw = Eigen::Isometry3d::Identity();
}

bool Semi_Direct::Get_data(SYNC::CALLBACK** _data){
    if(!(*_data)->Curr_C_mat.empty() && !(*_data)->Curr_D_mat.empty()){
        data_mtx.lock();
        curr_color = (*_data)->Curr_C_mat.clone();
        curr_depth = (*_data)->Curr_D_mat.clone();
        cv::cvtColor(curr_color, curr_gray, cv::COLOR_BGR2GRAY);
        // measurements.clear();
        data_mtx.unlock();
        return true;
    }
    else{
        ROS_ERROR("[Semi_Direct][Get_data] No Get data!!");
        return false;
    }
}

inline Eigen::Vector3d Semi_Direct::project2Dto3D(int x, int y, int d, float fx, float fy, float cx, float cy, float scale){
    float zz = float(d) / scale;
    float xx = zz * (x-cx) / fx;
    float yy = zz * (y-cy) / fy;
    return Eigen::Vector3d (xx, yy, zz);
}

inline Eigen::Vector2d Semi_Direct::project3Dto2D ( float x, float y, float z, float fx, float fy, float cx, float cy )
{
    float u = fx*x/z+cx;
    float v = fy*y/z+cy;
    return Eigen::Vector2d ( u,v );
}

void Semi_Direct::compute_gradient(bool* f_i){
    if(!(*f_i)){    
        for(int x=10; x<curr_gray.cols-10; x++)
            for(int y=10; y<curr_gray.rows-10; y++){
                Eigen::Vector2d delta(
                    curr_gray.ptr<uchar>(y)[x+1] - curr_gray.ptr<uchar>(y)[x-1], 
                    curr_gray.ptr<uchar>(y+1)[x] - curr_gray.ptr<uchar>(y-1)[x]
                );
                if(delta.norm() < 50)
                    continue;
                ushort d = curr_depth.ptr<ushort> (y)[x];
                if(d==0)
                    continue;
                Eigen::Vector3d p3d = project2Dto3D(x, y, d, fx, fy, cx, cy, depth_scale);
                float grayscale = float(curr_gray.ptr<uchar>(y)[x]);
                measurements.push_back(Measurement(p3d, grayscale));
            }
        prev_color = curr_color.clone();
        *f_i = true;
    }  
}

void Semi_Direct::Display_Feature(){
    cv::Mat prev_img(prev_color.rows, prev_color.cols, CV_8UC3);
    cv::Mat curr_img(prev_color.rows, prev_color.cols, CV_8UC3);

    prev_color.copyTo(prev_img);
    curr_color.copyTo(curr_img);

    for(Measurement m : measurements){
        Eigen::Vector3d prev_p = m.pos_world;
        Eigen::Vector2d pixel_prev = project3Dto2D (prev_p(0,0), prev_p(1,0), prev_p(2,0), fx, fy, cx, cy);
        Eigen::Vector3d curr_p = Tcw*m.pos_world;
        Eigen::Vector2d pixel_curr = project3Dto2D (curr_p(0,0), curr_p(1,0), curr_p(2,0), fx, fy, cx, cy);

        uchar b = 0;
        uchar g = 250;
        uchar r = 0;

        prev_img.ptr<uchar>(pixel_prev(1,0))[int(pixel_prev(0,0))*3] = b;
        prev_img.ptr<uchar>(pixel_prev(1,0))[int(pixel_prev(0,0))*3+1] = g;
        prev_img.ptr<uchar>(pixel_prev(1,0))[int(pixel_prev(0,0))*3+2] = r;

        curr_color.ptr<uchar>(pixel_curr(1,0))[int(pixel_curr(0,0))*3] = b;
        curr_color.ptr<uchar>(pixel_curr(1,0))[int(pixel_curr(0,0))*3+1] = g;
        curr_color.ptr<uchar>(pixel_curr(1,0))[int(pixel_curr(0,0))*3+2] = r;
            
        cv::circle (prev_img, cv::Point2d (pixel_prev(0,0), pixel_prev(1,0)), 4, cv::Scalar(b,g,r), 2);
        cv::circle (curr_color, cv::Point2d (pixel_curr(0,0), pixel_curr(1,0)), 4, cv::Scalar(b,g,r), 2);
    }
    cv::imshow ( "prev_img", prev_img );
    cv::imshow ( "curr_img", curr_color );

    cv::waitKey ( 1 );
}

bool Semi_Direct::PoseEstimationDirect(){
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<6,1>> DirectBlock;  // 求解的向量是6＊1的
    DirectBlock::LinearSolverType* linearSolver = new g2o::LinearSolverDense< DirectBlock::PoseMatrixType > ();
    DirectBlock* solver_ptr = new DirectBlock ( linearSolver );
    // g2o::OptimizationAlgorithmGaussNewton* solver = new g2o::OptimizationAlgorithmGaussNewton( std::move(solver_ptr) ); // G-N
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( std::move(solver_ptr)); // L-M

    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );
    optimizer.setVerbose( true );

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    pose->setEstimate ( g2o::SE3Quat ( Tcw.rotation(), Tcw.translation() ) );
    pose->setId (0);
    optimizer.addVertex ( pose );

    for (Measurement m: measurements)
    {
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

    std::cout << "******************************" << std::endl;
    std::cout << "[ID:" << id << "] Translation= \n" << Tcw.translation() << std::endl;
    std::cout << "******************************" << std::endl;
    std::cout << "[ID:" << id << "] Rotation= \n" << Tcw.rotation() << std::endl;
    std::cout << "******************************" << std::endl;

    return true;
}

void Semi_Direct::runloop(SYNC::CALLBACK** _data){
    ros::Rate loop_rate(10);
    bool first_index = false;
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
        
        if(!Get_data(_data)) continue;
        compute_gradient(&first_index);
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
        PoseEstimationDirect();
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>> (t2-t1);
        std::cout << "direct method costs time: " << time_used.count() << " seconds." << std::endl;
        Display_Feature();
    }
}