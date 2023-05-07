#include "semi_dense/plotTrajectory.hpp"

Pango::Loader::Loader(std::string& path, std::vector<double>& input):this_name("Loader")
{
    Get_data(path, input);
}
Pango::Loader::Loader(std::string& path, CAMERA_INTRINSIC_PARAM* input):this_name("Loader")
{
    Get_data(path, &input);
}

bool Pango::Loader::Get_data(std::string& trajectory_file, CAMERA_INTRINSIC_PARAM** input)
{
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    std::ifstream fin(trajectory_file);
    if(!fin){
        throw Exception("Process exception[" + this_name + "][Get_data]");    
    }
    while (!fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));
        Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
        poses.push_back(Twr);
    }
    DrawTrajectory(poses, &input);
    return 0;
}

bool Pango::Loader::Get_data(std::string& trajectory_file, std::vector<double>& input)
{
    std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>> poses;
    std::ifstream fin(trajectory_file);
    if(!fin){
        std::cout << "Gotcha" << std::endl;
        throw Exception("Process exception[" + this_name + "][Get_data]");
    }
    while (!fin.eof()) {
        double time, tx, ty, tz, qx, qy, qz, qw;
        fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw;
        Eigen::Isometry3d Twr(Eigen::Quaterniond(qw, qx, qy, qz));
        Twr.pretranslate(Eigen::Vector3d(tx, ty, tz));
        poses.push_back(Twr);
    }
    fin.close();
    std::cout << "pose size = " << poses.size() << std::endl;
    // Eigen::Isometry3d Twr0(Eigen::Quaterniond(1.3112, 0.8507, 1.5186, 0.8851));
    // Twr0.pretranslate(Eigen::Vector3d(0.2362, -0.0898, -0.3909));
    // poses.push_back(Twr0);

    // Eigen::Isometry3d Twr1(Eigen::Quaterniond(0.8884, 0.2171, -0.0899, -0.3943));
    // Twr1.pretranslate(Eigen::Vector3d(1.3263, 0.8375, 1.5226));
    // poses.push_back(Twr1);

    // Eigen::Isometry3d Twr2(Eigen::Quaterniond(0.8893, 0.2037, -0.0885, -0.3998));
    // Twr2.pretranslate(Eigen::Vector3d(1.3370, 0.8274, 1.5265));
    // poses.push_back(Twr2);

    // Eigen::Isometry3d Twr3(Eigen::Quaterniond(0.8900, 0.1933, -0.0877, -0.4036));
    // Twr3.pretranslate(Eigen::Vector3d(1.3454, 0.8190, 1.5298));
    // poses.push_back(Twr3);

    // Eigen::Isometry3d Twr4(Eigen::Quaterniond(0.8887, 0.1852, -0.0888, -0.4099));
    // Twr4.pretranslate(Eigen::Vector3d(1.3531, 0.8097, 1.5335));
    // poses.push_back(Twr4);

    // Eigen::Isometry3d Twr5(Eigen::Quaterniond(0.8865, 0.1763, -0.0915, -0.4180));
    // Twr5.pretranslate(Eigen::Vector3d(1.3627, 0.7957, 1.5391));
    // poses.push_back(Twr5);

    // Eigen::Isometry3d Twr6(Eigen::Quaterniond(0.8853, 0.1717, -0.0922, -0.4222));
    // Twr6.pretranslate(Eigen::Vector3d(1.3693, 0.7843, 1.5435));
    // poses.push_back(Twr6);

    // Eigen::Isometry3d Twr7(Eigen::Quaterniond(0.8826, 0.1686, -0.0948, -0.4285));
    // Twr7.pretranslate(Eigen::Vector3d(1.3745, 0.7721, 1.5475));
    // poses.push_back(Twr7);

    // Eigen::Isometry3d Twr8(Eigen::Quaterniond(0.8871, 0.2258, 0.0894, 0.3926));
    // Twr8.pretranslate(Eigen::Vector3d(1.3190, 0.8446, 1.5203));
    // poses.push_back(Twr8);

    DrawTrajectory(poses, input);
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

void Pango::Loader::DrawTrajectory(std::vector<Eigen::Isometry3d, Eigen::aligned_allocator<Eigen::Isometry3d>>& poses, CAMERA_INTRINSIC_PARAM*** input){
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
 
  pangolin::OpenGlRenderState s_cam(
    pangolin::ProjectionMatrix(1024, 768, (**input)->fx, (**input)->fy, (**input)->cx, (**input)->cy, 0.1, 1000),
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
      s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024, 768, (**input)->fx, (**input)->fy, (**input)->cx, (**input)->cy, 0.1, (**input)->depth_scale));
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
          DrawNode(poses, i, 0.01);
      }
        
      if(count > poses.size()-1) count = 0;

      glLineWidth(Ober);
      DrawNode(poses, count, 0.01);
      count++;
    };
    glLineWidth(First);
    DrawNode(poses, 0, 0.01);

    glLineWidth(Last);
    DrawNode(poses, poses.size()-1, 0.01);

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
  std::cout << "DrawTrajectory!" << std::endl;
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
        for (size_t i = 0; i < poses.size(); i++)
          DrawNode(poses, i, 0.01);
      }
        
      if(count > poses.size()-1) count = 0;

      glLineWidth(Ober);
      DrawNode(poses, count, 0.01);
      count++;
    };
    glLineWidth(First);
    DrawNode(poses, 0, 0.01);

    glLineWidth(Last);
    DrawNode(poses, poses.size()-1, 0.01);

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
    usleep(50000);   // sleep 5 ms
  }
}