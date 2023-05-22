#ifndef PARAM_H
#define PARAM_H

class CAMERA_INTRINSIC_PARAM
{
    public:
        float cx;
        float cy; 
        float fx;
        float fy;
        float depth_scale;
    CAMERA_INTRINSIC_PARAM()
    :cx(0.0f), cy(0.0f), fx(0.0f), fy(0.0f), depth_scale(0.0f){}
    CAMERA_INTRINSIC_PARAM(float _cx, float _cy, float _fx, float _fy, float _depth)
    :cx(_cx), cy(_cy), fx(_fx), fy(_fy), depth_scale(_depth){}

    ~CAMERA_INTRINSIC_PARAM(){}
}; //{325.5, 253.5, 518.0, 519.0, 1000.0};

class GET_PARAM : public CAMERA_INTRINSIC_PARAM
{
    public:
        int show;
        float depth_range_min = 0.0f;
        float depth_range_max = 5.0f;
    public:
        GET_PARAM():
        show(false){}
        ~GET_PARAM(){}
};

#endif