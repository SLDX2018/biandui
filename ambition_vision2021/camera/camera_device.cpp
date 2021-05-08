#include "camera_device.h"

CameraInit::CameraInit(const int device)
{
        //cap.open("/home/coumputer/桌面/炮台素材红车旋转-ev--3.MOV");

    cap.open(0);
    //设置参数的顺序一定是格式、帧率、宽 / 高
    cap.set(CV_CAP_PROP_FOURCC, CV_FOURCC('M', 'J', 'P', 'G'));//?????MJPG???
    cap.set(CV_CAP_PROP_FPS, 60);
    //cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    //cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 1280);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 960);

    cap.set(CV_CAP_PROP_EXPOSURE, 0);//曝光
}

