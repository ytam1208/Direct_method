#include "semi_dense/EdgeSE3ProjectDirect.hpp"

EdgeSE3ProjectDirect::EdgeSE3ProjectDirect(){}

EdgeSE3ProjectDirect::EdgeSE3ProjectDirect( Eigen::Vector3d point, float fx, float fy, float cx, float cy, cv::Mat* image ):
x_world_(point), fx_(fx), fy_(fy), cx_(cx), cy_(cy), image_(image)
{}

void EdgeSE3ProjectDirect::computeError(){
    const g2o::VertexSE3Expmap* v  = static_cast<const g2o::VertexSE3Expmap*> ( _vertices[0] );
    Eigen::Vector3d x_local = v->estimate().map ( x_world_ );
    float x = x_local[0]*fx_/x_local[2] + cx_;
    float y = x_local[1]*fy_/x_local[2] + cy_;
    // check x,y is in the image
    if (x-4 < 0 
        || (x+4) > image_->cols 
        || (y-4) < 0 
        || (y+4) > image_->rows)
    {
        _error ( 0,0 ) = 0.0;
        this->setLevel ( 1 );
    }
    else
    {
        _error ( 0,0 ) = getPixelValue ( x,y ) - _measurement;
    }
}

void EdgeSE3ProjectDirect::linearizeOplus(){
    if ( level() == 1 )
    {
        _jacobianOplusXi = Eigen::Matrix<double, 1, 6>::Zero();
        return;
    }
    g2o::VertexSE3Expmap* vtx = static_cast<g2o::VertexSE3Expmap*> ( _vertices[0] );
    Eigen::Vector3d xyz_trans = vtx->estimate().map ( x_world_ );   // q in book

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double invz = 1.0/xyz_trans[2];
    double invz_2 = invz*invz;

    float u = x*fx_*invz + cx_;
    float v = y*fy_*invz + cy_;

    // jacobian from se3 to u,v
    // NOTE that in g2o the Lie algebra is (\omega, \epsilon), where \omega is so(3) and \epsilon the translation
    Eigen::Matrix<double, 2, 6> jacobian_uv_ksai;

    jacobian_uv_ksai ( 0,0 ) = - x*y*invz_2 *fx_;
    jacobian_uv_ksai ( 0,1 ) = ( 1+ ( x*x*invz_2 ) ) *fx_;
    jacobian_uv_ksai ( 0,2 ) = - y*invz *fx_;
    jacobian_uv_ksai ( 0,3 ) = invz *fx_;
    jacobian_uv_ksai ( 0,4 ) = 0;
    jacobian_uv_ksai ( 0,5 ) = -x*invz_2 *fx_;

    jacobian_uv_ksai ( 1,0 ) = - ( 1+y*y*invz_2 ) *fy_;
    jacobian_uv_ksai ( 1,1 ) = x*y*invz_2 *fy_;
    jacobian_uv_ksai ( 1,2 ) = x*invz *fy_;
    jacobian_uv_ksai ( 1,3 ) = 0;
    jacobian_uv_ksai ( 1,4 ) = invz *fy_;
    jacobian_uv_ksai ( 1,5 ) = -y*invz_2 *fy_;

    Eigen::Matrix<double, 1, 2> jacobian_pixel_uv;

    jacobian_pixel_uv ( 0,0 ) = ( getPixelValue ( u+1,v )-getPixelValue ( u-1,v ) ) /2;
    jacobian_pixel_uv ( 0,1 ) = ( getPixelValue ( u,v+1 )-getPixelValue ( u,v-1 ) ) /2;

    _jacobianOplusXi = jacobian_pixel_uv*jacobian_uv_ksai;
}

inline float EdgeSE3ProjectDirect::getPixelValue (float x, float y){
    uchar* data = & image_->data[ int ( y ) * image_->step + int ( x ) ];
    float xx = x - floor ( x );
    float yy = y - floor ( y );
    return float (
               ( 1-xx ) * ( 1-yy ) * data[0] +
               xx* ( 1-yy ) * data[1] +
               ( 1-xx ) *yy*data[ image_->step ] +
               xx*yy*data[image_->step+1]
    );
}