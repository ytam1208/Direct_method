#ifndef EdgeSE3_H
#define EdgeSE3_H

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

class EdgeSE3ProjectDirect: public g2o::BaseUnaryEdge<1, double, g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    EdgeSE3ProjectDirect();
    EdgeSE3ProjectDirect (Eigen::Vector3d point, float fx, float fy, float cx, float cy, cv::Mat* image);

    virtual void computeError();
    // plus in manifold
    virtual void linearizeOplus();
    // dummy read and write functions because we don't care...
    virtual bool read ( std::istream& in ) {return true;}
    virtual bool write ( std::ostream& out ) const {return true;}

protected:
    // get a gray scale value from reference image (bilinear interpolated)
    inline float getPixelValue (float x, float y);
public:
    Eigen::Vector3d x_world_;   // 3D point in world frame
    float cx_=0.0f, cy_=0.0f, fx_=0.0f, fy_=0.0f; // Camera intrinsics
    cv::Mat* image_=nullptr;    // reference image
};

#endif