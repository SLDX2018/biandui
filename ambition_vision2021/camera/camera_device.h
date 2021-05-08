#ifndef CAMERA_DEVICE_H
#define CAMERA_DEVICE_H

#include <opencv2/opencv.hpp>
using namespace cv;

class CameraInit
{
public:
    CameraInit(const int device);

public:
    VideoCapture cap;
    Mat image;
};



#endif // CAMERA_DEVICE_H
