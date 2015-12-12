#pragma once

#include <iostream>
#include <vector>
#include <map>
#include <unistd.h>
#include <cstdio>
#include <fstream>

#include "libfreenect.h"


class Data {
public:
    int frame_size;
    int frame_count;
    std::vector<uint16_t*> depth_frames;

public:
    Data(int frame_size);
    void reset(int frame_count);
    int is_complete();
    uint16_t* get_frame();
    uint16_t* get_result();
};


class Device {

private:
    freenect_context* f_ctx;
    freenect_device* f_dev;
    freenect_resolution resolution;
    freenect_depth_format format;
    
    Data* data;
    int dev_num;
    int height;
    int width;
    int size;

public:
    static std::map<freenect_device*, Data*> dev_data;

    Device(int);
    void open();
    void stop();
    void start(int frame_count);
    int get_width();
    int get_heigth();
    uint16_t* get_result();
    int get_pixel(int width, int height);
    void save(char* filename);
};