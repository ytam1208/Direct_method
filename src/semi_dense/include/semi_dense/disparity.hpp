#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/ximgproc.hpp>

#include <image_geometry/stereo_camera_model.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

struct Matcher_Param
{
    int minDisparity = -3;
    int numDisparities = 96; //16 ~ 96          
    int blockSize = 7;
    int P1 = 60;
    int P2 = 2400;
    int disp12MaxDiff = 90;
    int preFilterCap = 16;
    int uniquenessRatio = 1; //63, 5~15
    int speckleWindowSize = 60; //10, 50~200
    int speckleRange = 20; //100, 1, 2
    bool fullDP=true; //cv::StereoSGBM::MODE_SGBM; MODE_HH MODE_SGBM_3WAY MODE_HH4
};
/*
    minDisparity : 가능한 최소한의 disparity 값이다. 보통 0으로 설정하지만 조정 알고리즘이 이미지를 이동시킬 수 있어서 이 파라미터는 알맞게 조정되어야 한다.
    numDisparities : 최대 disparity 빼기 최소 disparity. 항상 0보다 큰 값을 갖는다.
    blockSize : 매칭된 블록 사이즈이다. 1보다 큰 홀수여야한다. 보통 3~11의 값을 갖는다.
    P1 : diparity smoothness를 조절하는 첫 파라미터이다.
    P2 : diparity smoothness를 조절하는 두 번째 파라미터이다. 값이 커지면, 더 부드러워진다. P1은 인접 픽셀 사이를 +1 or -1로 바꿔주는 disparity 패널티이다(P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels). P2는 인접 픽셀 사이에서 1보다 크게 변화하는 것에 대한 패널티이다.(P1 is the penalty on the disparity change by plus or minus 1 between neighbor pixels). P2 > P1이어야 한다.
    disp12MAxDiff : 좌-우 disparity 체크에서 허용된 최대 차이이다. 비활성화 하려면 음수를 설정하면 된다.
    preFilterCap : 필터링된 이미지 픽셀을 위한 Truncation(절단) 값이다.
    uniquenessRatio : 보통 5~15의 값을 갖는게 좋다. 1등 매치(cost function 비교)와 2등 매치간의 마진을 퍼센트로 나타낸 것이다.
    speckleWindowSize : 노이즈 반점(speckle) 및 무효화를 고려하는 smooth disparity 영역의 최대 크기이다. 0로 두면 실행 안함. 보통 50-200의 범위를 가진다
    speckleRange : 각 연결된 요소들 내에서 최대 disparity 변동이다. speckle 필터링을 한다면, 양수로 설정하고, 이는 16과 곱해질 것이기다. 보통 1,2면 좋다.
    mode : cv2.STEREO_SGBM_MODE_HH를 사용하여 전체 스케일에 대해 동적 프로그래밍 알고리즘을 실행한다. 이는 기본값이다.
*/

class Depth_Map
{
private:
    Matcher_Param mp;
    cv::Ptr<cv::StereoSGBM> stereo;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
    cv::Ptr<cv::StereoMatcher> sm;
    double lambda;
    double sigma;
    cv::Mat left_disparity, right_disparity;
    cv::Mat filtered_disparity, show_disparity, showFilteredDisparity;
    cv::Mat depth;

    pcl::PCLPointCloud2 pc2;
    sensor_msgs::PointCloud2 point_cloud;
    float pixel2mm = 200.0f;

public:
    void initalize(){
        stereo = cv::StereoSGBM::create(mp.minDisparity,  mp.numDisparities,         mp.blockSize, 
                                                  mp.P1,              mp.P2,     mp.disp12MaxDiff,
                                        mp.preFilterCap, mp.uniquenessRatio, mp.speckleWindowSize,
                                        mp.speckleRange,          mp.fullDP);
        // filter
        wls_filter = cv::ximgproc::createDisparityWLSFilter(stereo);
        sm = cv::ximgproc::createRightMatcher(stereo);
        // filter param
        lambda = 8000.0;
        sigma = 1.5;
    }
    void Filter_disparity_map(cv::Mat& left, cv::Mat& right){
        sm->compute(right, left, this->right_disparity);
        wls_filter->setLambda(lambda);
        wls_filter->setSigmaColor(sigma);
        wls_filter->filter(this->left_disparity, left, this->filtered_disparity, this->right_disparity);
        this->filtered_disparity.convertTo(this->showFilteredDisparity, CV_8U, 255 / (mp.numDisparities*16.));
        //result[filtered_disparity, showFilteredDisparity]
    }
    void calculate_disparity_map(cv::Mat& left, cv::Mat& right){
        stereo->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
        stereo->compute(left, right, this->left_disparity);
        cv::normalize(this->left_disparity, this->show_disparity, 0, 255, CV_MINMAX, CV_8U);
        //result[left_disparity, show_disparity, filtered_disparity, showFilteredDisparity]
    }
    void Display(cv::Mat& left, cv::Mat& right, bool Use_filter){
        cv::imshow("L", left);
        cv::imshow("R", right);
        cv::imshow("disparity", this->show_disparity);
        if(Use_filter)
            cv::imshow("Filtered Disparity", this->showFilteredDisparity);
        cv::waitKey(1);
    }
    inline bool isValidpoint(const float pt){
        return pt!=image_geometry::StereoCameraModel::MISSING_Z && !std::isinf(pt);
    }
    void operator()(cv::Mat& left, cv::Mat& right, bool Use_filter, bool show_flag){
        calculate_disparity_map(left, right);
        if(Use_filter)
            Filter_disparity_map(left, right);
        if(show_flag)
            Display(left, right, Use_filter);
    }
    sensor_msgs::PointCloud2Ptr operator()(cv::Mat& base, float focal_length, float base_line, float fx, float fy, float cx, float cy, Eigen::Matrix3d& R_D, Eigen::Vector3d& T_D){
        this->depth = this->showFilteredDisparity.clone();

        sensor_msgs::PointCloud2Ptr points_msg = boost::make_shared<sensor_msgs::PointCloud2>();

        points_msg->header.stamp = ros::Time::now();
        points_msg->header.frame_id = "camera_link";
        points_msg->width = depth.cols;
        points_msg->height = depth.rows;
        points_msg->is_dense = false;
        points_msg->is_bigendian = false;
        sensor_msgs::PointCloud2Modifier pcd_modifier(*points_msg);
        pcd_modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(*points_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*points_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*points_msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(*points_msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(*points_msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(*points_msg, "b");


        float bad_point = std::numeric_limits<float>::quiet_NaN();
        for(int v = 0; v < depth.rows; ++v)
            for(int u = 0; u < depth.cols; ++u, ++iter_x, ++iter_y, ++iter_z){
                float disparity = depth.at<uchar>(v,u);
                if(isValidpoint((float)depth.at<uchar>(v,u))){
                    *iter_x = v / pixel2mm;
                    *iter_y = u / pixel2mm;
                    *iter_z = disparity / pixel2mm;
                    // Eigen::Vector3d input_p(*iter_x, *iter_y, *iter_z);
                    // Eigen::Vector3d reprj_p(R_D * input_p + T_D);
                    // *iter_x = reprj_p[0];
                    // *iter_y = reprj_p[1];
                    // *iter_z = reprj_p[2];
                    // std::cout << *iter_x << ", " << *iter_y << ", " << *iter_z << std::endl;
                }
                else{
                    *iter_x = *iter_y = *iter_z = bad_point;
                }
            }
        const std::string encoding = "mono8";
        if(encoding == sensor_msgs::image_encodings::MONO8){
            for(int v = 0; v < base.rows; ++v){
                for(int u = 0; u < base.cols; ++u, ++iter_r, ++iter_g, ++iter_b){
                    uint8_t g = base.at<uchar>(v,u);
                    // uint8_t g = depth.at<uchar>(v,u);
                    *iter_r = *iter_g = *iter_b = g;
                }
            }
        }

        return points_msg;
    }
    Depth_Map(){
        initalize();
    }
    ~Depth_Map(){}
};