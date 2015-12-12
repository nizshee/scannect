
%module KinectWrapper

%{
#define SWIG_FILE_WITH_INIT
#include "kinect.h"
%}

class Device {
private:

public:
    Device(int);
    void open();
    void stop();
    void start(int frame_count);
    int get_width();
    int get_heigth();
    int get_pixel(int width, int height);
    void save(char* filename);
};