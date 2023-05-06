#ifndef FRAME_H
#define FRAME_H

namespace Direct
{
    class Frame
    {
    public:
        cv::Mat color;
        cv::Mat depth;
        cv::Mat gray;
        Frame(){}
        ~Frame(){}
    };
}
typedef Direct::Frame DF;

#endif