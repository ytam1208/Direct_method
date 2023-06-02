#include <ros/ros.h>
#include <opencv2/opencv.hpp>
// #include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <memory>
#include <vector>
#include "feat/exception.hpp"
#include "feat/Load_KITTI.hpp"
#include "feat/Odometer.hpp"
#include <opencv2/core/eigen.hpp>
class Matcher
{
protected:
    std::string this_name;
private:
    cv::Mat src, target;
    TEST::OdomManager om;
    Eigen::Isometry3d absol_odom;
    std::vector<cv::Point3f> ref_3d_pts;
    cv::Mat descriptors_ref_;
    cv::Mat K;

    bool compute_orb(cv::Mat& prev, cv::Mat& now, cv::Mat& prev_depth, cv::Mat& now_depth){
        try{
            if(prev.empty() || now.empty())
                ROS_ERROR("No Synchronize data");
            else{
                cv::Mat dst;
                this->src = cv::Mat::zeros(prev.rows, prev.cols, CV_8UC1);
                this->target = cv::Mat::zeros(now.rows, now.cols, CV_8UC1);

                this->src = prev.clone();
                this->target = now.clone();

                cv::Ptr<cv::Feature2D> feature = cv::ORB::create();
                std::vector<cv::KeyPoint> target_kps, src_kps;

                cv::Mat trgDesc, srcDesc;
                feature->detectAndCompute(now, cv::Mat(), target_kps, trgDesc);
                feature->detectAndCompute(prev, cv::Mat(), src_kps, srcDesc);

                cv::Ptr<cv::DescriptorMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING);
                std::vector<cv::DMatch> matches;
                matcher->match(trgDesc, srcDesc, matches);
                std::sort(matches.begin(), matches.end());
                std::vector<cv::DMatch> good_matches(matches.begin(), matches.end());
                cv::drawMatches(this->target, target_kps, this->src, src_kps, good_matches, dst,
                cv::Scalar::all(-1), cv::Scalar(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
                
                cv::Mat total(prev.rows*2, prev.cols, CV_8UC3);
                prev.copyTo(total(cv::Rect(0,0, prev.cols, prev.rows)));
                now.copyTo(total(cv::Rect(0, prev.rows, prev.cols, prev.rows)));

                cv::putText(total, "Prev", cv::Point(10,20), 1, 1, cv::Scalar(0,0,255), 2, 8);
                cv::putText(total, "Current", cv::Point(10,20+prev.rows), 1, 1, cv::Scalar(0,0,255), 2, 8);
                cv::imshow("dst", dst);
                cv::imshow("total", total);

                if(cv::waitKey(0) == 27)
                    return true;
            
                Motion_Estimate(src_kps, target_kps, good_matches);
            }
            return false;
        }
        catch(cv::Exception& e){
            throw MY::Exception("Process exception[" + this_name + "][compute_orb] [Error code] cv::Exception");
        }catch(std::bad_alloc& e){//메모리 할당 실패시 예외 발생
            throw MY::Exception("Process exception[" + this_name + "][compute_orb] [Error code] std::bad_alloc");
        }catch(MY::Exception& e){
            throw MY::Exception("Process exception[" + this_name + "][compute_orb] [Error code] std::exception");
        }catch(...){
            throw MY::Exception("Process exception[" + this_name + "][compute_orb] [Error code] Other exception");
        }
    }
    void Motion_Estimate(std::vector<cv::KeyPoint>& src, std::vector<cv::KeyPoint>& target, std::vector<cv::DMatch>& good_matches){
        std::vector<int> points1, points2;
        for(auto iter = good_matches.begin(); iter != good_matches.end(); ++iter){
            points1.push_back(iter->queryIdx);
            points2.push_back(iter->trainIdx);
        }

        // cv::Mat cameraMatrix = cv::Mat::eye(3,3, CV_64F);
        std::vector<cv::Point2f> selPoints1, selPoints2;
        cv::KeyPoint::convert(src, selPoints1, points1);
        cv::KeyPoint::convert(target, selPoints2, points2); 
        cv::Mat inliers;
        cv::Mat essential = cv::findEssentialMat(cv::Mat(selPoints1), cv::Mat(selPoints2), K, cv::RANSAC, 0.999, 1.0, inliers);

        cv::Mat R,t;
        cv::recoverPose(essential, selPoints1, selPoints2, K, R, t, inliers);
        std::cout << "trans = " << t << std::endl;
        // Get_Motion(R, t);

        // cv::Mat relative_pose = cv::Mat::zeros(3, 4, CV_64F);
        // R.copyTo(relative_pose(cv::Rect(0,0,3,3)));
        // t.copyTo(relative_pose.colRange(3,4));

        // absol_odom = om.addMotion(absol_odom, relative_pose);
        // std::cout << "Transformation = \n" << relative_pose << std::endl;
        // std::cout << "Absol Transformation = \n" << absol_odom << std::endl;

    }
    void Get_Motion(cv::Mat& R, cv::Mat T){
        Eigen::Matrix3d Rotation_Matrix;
        Eigen::Vector3d Translation_Matrix;
        cv::cv2eigen(R, Rotation_Matrix);
        cv::cv2eigen(T.t(), Translation_Matrix);

        Eigen::Quaterniond Q_r(Rotation_Matrix);
        Eigen::Isometry3d Twr(Q_r);
        Twr.pretranslate(Translation_Matrix);
        this->absol_odom = om.addMotion(this->absol_odom, Twr);
        
        // om.print_RT(Twr);
        om.print_RT(this->absol_odom);
    }
    double Get_Depth(const cv::KeyPoint& kp, cv::Mat& depth){
        int x = cvRound(kp.pt.x);
        int y = cvRound(kp.pt.y);
        float d = depth.ptr<float>(y)[x];
        if(d!=0.0)
            return d/1000.0f;
        else{
            int dx[4] = {-1, 0, 1, 0};
            int dy[4] = {0, -1, 0, 1};
            for(int i = 0; i < 4; i++){
                d = depth.ptr<float>(y+dy[i])[x+dx[i]];
                if(d!=0.0)
                    return d/1000.0f;
            }
        }
        return -1.0;
    }
    Eigen::Vector3d pixel2camera(const Eigen::Vector2d& p_p, double depth){
        return Eigen::Vector3d(
            (p_p(0,0)-K.at<float>(0,2)) * depth/K.at<float>(0,0),
            (p_p(1,0)-K.at<float>(1,2)) * depth/K.at<float>(1,1),
            depth
        );
    }
    void SetRef3DPoints(std::vector<cv::KeyPoint>& curr_pt, cv::Mat& curr_depth, cv::Mat& srcDesc){
        ref_3d_pts.clear();
        descriptors_ref_ = cv::Mat();
        for(int i = 0; i < curr_pt.size(); i++){
            double d = Get_Depth(curr_pt[i], curr_depth);
            if(d > 0.0f){
                Eigen::Vector3d p_cam = pixel2camera(
                                                        Eigen::Vector2d(curr_pt[i].pt.x, curr_pt[i].pt.y), d
                                                    );
                ref_3d_pts.push_back(cv::Point3f(p_cam(0,0), p_cam(1,0), p_cam(2,0)));
                descriptors_ref_.push_back(srcDesc.row(i));
            }
        }
    }
    void featureMatching(){
     //https://github.com/gaoxiang12/slambook/blob/master/project/0.2/src/visual_odometry.cpp#L32
     //https://github.com/gaoxiang12/slambook/blob/master/project/0.2/include/myslam/visual_odometry.h 
    }
    // void poseEstimationPnP(){
    //     std::vector<cv::Point3f> pts3d;
    //     std::vector<cv::Point2d> pts2d;
    //     for(cv::DMatch m : feature_)
    // }
    void runloop(std::function<std::vector<TEST::Frame>(Loader_Kitti&)>& frames){
        bool flag;
        // for(auto frame : frames(LK))
            // compute_orb(frame.Left, frame.right)

        int size = frames(LK).size();
        for(int i = 0; i < frames(LK).size(); i++){
            if(i != size)
                if(compute_orb(frames(LK)[i].Left, frames(LK)[i+1].Left, frames(LK)[i].depth, frames(LK)[i+1].depth))
                    break;
        }
    }
public:
    Matcher(std::function<std::vector<TEST::Frame>(Loader_Kitti&)>& frames):
    this_name("Matcher"){
        absol_odom = Eigen::Isometry3d::Identity();
        K = (cv::Mat_<float>(3,3) << 718.856f,     0.0f, 607.192f,
                                         0.0f, 718.856f, 185.215f,
                                         0.0f,     0.0f,     1.0f);

        // runloop(frames);
    }
    ~Matcher(){}    
    // void operator()(std::shared_ptr& ptr){}
};