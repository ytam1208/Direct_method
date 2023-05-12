#include "semi_dense/plotTrajectory.hpp"

Pango::Loader::Loader(std::string& path, CAMERA_INTRINSIC_PARAM* input):
this_name("Loader"),Pango_col(1024),Pango_row(768)
{
    Get_data(path);
    DrawTrajectory(poses, &input);
}
Pango::Loader::Loader(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& poses, std::string& path, std::vector<double>& input):
this_name("Loader"),Pango_col(1024),Pango_row(768)
{
    Get_data(path);
    DrawTrajectory(poses, input);
}

bool Pango::Loader::Get_data(std::string& trajectory_file)
{
    std::ifstream fin_G(trajectory_file);
    if(fin_G.is_open()){
      int cnt = 0;
      try{
        while(!fin_G.eof()){
            float time, tx, ty, tz, qx, qy, qz, qw;
            fin_G >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
            Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));
            Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
            poses.push_back(Twr);
        }
        fin_G.close();
      }
      catch(...){
        fin_G.close();
        throw Exception("Process exception[" + this_name + "][Get_data] check your input data! [Error code] std::bad_alloc");
      }
    }
    else
        throw Exception("Process exception[" + this_name + "][Get_data] File not found");  
    return 0;
}

void Pango::Loader::DrawNode(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& poses, int i, float scale)
{
  Eigen::Vector3d Ow = poses[i].translation();
  Eigen::Vector3d Xw = poses[i] * (scale * Eigen::Vector3d(1, 0, 0));
  Eigen::Vector3d Yw = poses[i] * (scale * Eigen::Vector3d(0, 1, 0));
  Eigen::Vector3d Zw = poses[i] * (scale * Eigen::Vector3d(0, 0, 1));

  glBegin(GL_LINES);
  glColor3f(1.0, 0.0, 0.0); //red
  glVertex3d(Ow[0], Ow[1], Ow[2]);
  glVertex3d(Xw[0], Xw[1], Xw[2]);
  glColor3f(0.0, 1.0, 0.0); //green
  glVertex3d(Ow[0], Ow[1], Ow[2]);
  glVertex3d(Yw[0], Yw[1], Yw[2]);
  glColor3f(0.0, 0.0, 1.0); //blue
  glVertex3d(Ow[0], Ow[1], Ow[2]);
  glVertex3d(Zw[0], Zw[1], Zw[2]);
  glEnd();
}

void Pango::Loader::DrawTrajectory(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& poses, CAMERA_INTRINSIC_PARAM** input){
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
 
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, (*input)->fx, (*input)->fy, (*input)->cx, (*input)->cy, 0.1, 1000),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 
                              0, 0, 0, 
                              0.0, -1.0, 0.0)
  );
  const int UI_WIDTH = 20 * pangolin::default_font().MaxWidth();

  pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));

  pangolin::CreatePanel("ui")
      .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
  pangolin::Var<bool> Start_checkbox("ui.Start",false,true);
  pangolin::Var<double> Back_G("ui.BackGround_Color", 0.0, 0.0, 1.0);
  pangolin::Var<int> First("ui.First_node", 1, 2, 10);
  pangolin::Var<int> Last("ui.Last_node", 1, 2, 10);
  pangolin::Var<int> Ober("ui.Observer_node", 1, 2, 10);
  pangolin::Var<int> Other("ui.Other_node", 1, 2, 10);  
  pangolin::Var<bool> Reference_checkbox("ui.Reference",false,true);
  pangolin::Var<bool> Cam_View_reset("ui.View_reset",false,true);

  int count = 0;
  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if(Cam_View_reset){
      s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, (*input)->fx, (*input)->fy, (*input)->cx, (*input)->cy, 0.1, (*input)->depth_scale));
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(
                              0,   -0.1, -1.8, 
                              0,      0,    0, 
                              0.0, -1.0, 0.0));
      Cam_View_reset = false;
    }  
    d_cam.Activate(s_cam);
    glClearColor(Back_G, Back_G, Back_G, Back_G);
    if(Start_checkbox){
      if(Reference_checkbox){
        glLineWidth(Other);
        for (size_t i = 0; i < poses.size(); i++)
          DrawNode(poses, i, 0.1);
      }
        
      if(count > poses.size()-1) count = 0;

      glLineWidth(Ober);
      DrawNode(poses, count, 0.1);
      count++;
    };
    glLineWidth(First);
    DrawNode(poses, 0, 0.1);

    glLineWidth(Last);
    DrawNode(poses, poses.size()-1, 0.1);

    glLineWidth(1);
    for (size_t i = 0; i < poses.size(); i++) {
      glBegin(GL_LINES);
      glColor3f(1.0f-(float)Back_G, 1.0f-(float)Back_G, 1.0f-(float)Back_G);
      auto p1 = poses[i], p2 = poses[i + 1];
      glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
      glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      glEnd();
    }
    pangolin::FinishFrame();
    // usleep(5000);   // sleep 5 ms
  }
}
void Pango::Loader::DrawTrajectory(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& poses, std::vector<double>& input){
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, input[2], input[3], input[0], input[1], 0.1, input[4]),
    pangolin::ModelViewLookAt(0, -0.1, -1.8, 
                              0, 0, 0, 
                              0.0, -1.0, 0.0)
  );
  const int UI_WIDTH = 20 * pangolin::default_font().MaxWidth();
  pangolin::View &d_cam = pangolin::CreateDisplay()
    .SetBounds(0.0, 1.0, pangolin::Attach::Pix(UI_WIDTH), 1.0, -1024.0f / 768.0f)
    .SetHandler(new pangolin::Handler3D(s_cam));
  pangolin::CreatePanel("ui")
      .SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(UI_WIDTH));
  pangolin::Var<bool> Start_checkbox("ui.Start",false,true);
  pangolin::Var<double> Back_G("ui.BackGround_Color", 0.0, 0.0, 1.0);
  pangolin::Var<int> First("ui.First_node", 1, 2, 10);
  pangolin::Var<int> Last("ui.Last_node", 1, 2, 10);
  pangolin::Var<int> Ober("ui.Observer_node", 1, 2, 10);
  pangolin::Var<int> Other("ui.Other_node", 1, 2, 10);  
  pangolin::Var<bool> Reference_checkbox("ui.Reference",false,true);
  pangolin::Var<bool> Cam_View_reset("ui.View_reset",false,true);
  std::cout << "DrawTrajectory! Ground_Poses size = " << this->poses.size() << std::endl;
  std::cout << "DrawTrajectory! Data_Poses size = " << poses.size() << std::endl;
  int count = 0;
  while (pangolin::ShouldQuit() == false) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if(Cam_View_reset){
      s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, input[2], input[3], input[0], input[1], 0.1, input[4]));
      s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(
                              0,   -0.1, -1.8, 
                              0,      0,    0, 
                              0.0, -1.0, 0.0));
      Cam_View_reset = false;
    }  
    d_cam.Activate(s_cam);
    glClearColor(Back_G, Back_G, Back_G, Back_G);
    if(Start_checkbox){
      if(Reference_checkbox){
        glLineWidth(Other);
        for (size_t i = 0; i < this->poses.size(); i++)
          DrawNode(this->poses, i, 0.01);
      }
        
      if(count > poses.size()-1) count = 0;

      glLineWidth(Ober);
      DrawNode(poses, count, 0.1);
      count++;
    };
    glLineWidth(First);
    DrawNode(this->poses, 0, 0.01);

    glLineWidth(Last);
    DrawNode(this->poses, this->poses.size()-1, 0.01);

    glLineWidth(1);
    for(size_t i = 0; i < this->poses.size(); i++){
      glBegin(GL_LINES);
      glColor3f(1.0f-(float)Back_G, 1.0f-(float)Back_G, 1.0f-(float)Back_G);
      // auto p1;
      if(i < this->poses.size()-1){
        auto p1 = this->poses[i], p2 = this->poses[i+1];
        glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
        glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      }
      else{
          auto p1 = this->poses[0], p2 = this->poses[this->poses.size()-1];
          glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
          glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      }
      glEnd();
    }
    pangolin::FinishFrame();
    // usleep(50000);   // sleep 5 ms
  }
}