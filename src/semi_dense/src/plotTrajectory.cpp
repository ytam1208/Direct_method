#include "semi_dense/plotTrajectory.hpp"

Pango::Loader::Loader(std::string& path, CAMERA_INTRINSIC_PARAM* input):
this_name("Loader"),Pango_col(1024),Pango_row(768)
{
    // DrawTrajectory(poses, &input);
}
Pango::Loader::Loader(std::string& path, std::vector<double>& input,
std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& ob_poses, 
std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& ref_poses):
this_name("Loader"),Pango_col(1024),Pango_row(768)
{
    DrawTrajectory(ob_poses, ref_poses, input);
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
void Pango::Loader::DrawTrajectory(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& ob_poses, 
                                    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& ref_poses, 
                                      std::vector<double>& input){
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
  std::cout << "DrawTrajectory! Ground_Poses size = " << ref_poses.size() << std::endl;
  std::cout << "DrawTrajectory! Obser_Poses size = " << ob_poses.size() << std::endl;
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
        for (size_t i = 0; i < ref_poses.size(); i++)
          DrawNode(ref_poses, i, 0.01);
      }
        
      if(count > ob_poses.size()-1) count = 0;

      glLineWidth(Ober);
      DrawNode(ob_poses, count, 0.01);
      count++;
    }
    glLineWidth(First);
    DrawNode(ref_poses, 0, 0.01);

    glLineWidth(Last);
    DrawNode(ref_poses, ref_poses.size()-1, 0.01);

    glLineWidth(1);
    for(size_t i = 0; i < ref_poses.size(); i++){
      glBegin(GL_LINES);
      glColor3f(1.0f-(float)Back_G, 1.0f-(float)Back_G, 1.0f-(float)Back_G);
      // auto p1;
      if(i < ref_poses.size()-1){
        auto p1 = ref_poses[i], p2 = ref_poses[i+1];
        glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
        glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      }
      else{
          auto p1 = ref_poses[0], p2 = ref_poses[ref_poses.size()-1];
          glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
          glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
      }
      glEnd();
    }
    pangolin::FinishFrame();
    usleep(5000);   // sleep 5 ms
  }
}