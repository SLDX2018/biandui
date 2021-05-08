#ifndef HIKVISION_H
#define HIKVISION_H

#include "CameraParams.h"
#include "MvCameraControl.h"
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <iostream>

using namespace cv;

class HIKvison{
    public:
    float framerate;
    float exposure;
    int height;
    int width;
    MVCC_INTVALUE stParam;
    MV_CC_DEVICE_INFO_LIST list;
    MV_FRAME_OUT_INFO_EX stImageinfo;

  //  ~HIKvison(){close(Handle);}



public:

    //HIKvison(MV_CC_DEVICE_INFO_LIST llist);
    enum CONVERT_TYPE
    {
        OpenCV_Mat = 0,    // Most of the time, we use 'Mat' format to store image data after OpenCV V2.1
        OpenCV_IplImage = 1,   //we may also use 'IplImage' format to store image data, usually before OpenCV V2.1
    };
    Mat Image;
    unsigned int nIndex = 0;
    void* Handle = NULL;
    unsigned int g_nPayloadSize = 0;
     bool reset(MV_CC_DEVICE_INFO_LIST llist,int width,int height,float frame,float exposuretime,float gain,float gamma);
    bool visoncamera_found_decive(MV_CC_DEVICE_INFO_LIST stDeviceList);
    bool visoncamera_open(IN void* handle,MV_CC_DEVICE_INFO_LIST& stDeviceList);
    bool visoncamera_set(void*handle,int width,int height,float frame,float exposuretime,float gain,float gamma);
    bool visoncamera_start_grab(void* handle,MV_FRAME_OUT_INFO_EX stImageInfo);
    bool visoncamera_get_image(void* handle,MV_FRAME_OUT_INFO_EX stImageInfo,float gain);
    bool Convert2Mat(MV_FRAME_OUT_INFO_EX* pstImageInfo, unsigned char* pData);
    bool close(void *handle);
};

#endif // HIKVISION_H
