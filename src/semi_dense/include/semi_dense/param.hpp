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
    CAMERA_INTRINSIC_PARAM():
        cx(0.0f), 
        cy(0.0f), 
        fx(0.0f), 
        fy(0.0f), 
        depth_scale(0.0f){}
    ~CAMERA_INTRINSIC_PARAM(){}
}; //{325.5, 253.5, 518.0, 519.0, 1000.0};

class GET_PARAM : public CAMERA_INTRINSIC_PARAM
{
    public:
        int show;
        
    public:
        GET_PARAM():
        show(false){}
        ~GET_PARAM(){}
};

#endif