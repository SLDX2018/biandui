#ifndef THREAD_CONTROL_H
#define THREAD_CONTROL_H

#include <opencv2/opencv.hpp>
#include "../../base/base.h"
#include "../../camera/HIKvison.h"

#include <ros/ros.h>
#include "roborts_msgs/AmbitionVision.h"

using namespace cv;
/**
 * @brief 线程管理类
 * 负责图像生成、图像处理、串口数据接收
 */
class ThreadControl
{
public:
    MV_CC_DEVICE_INFO_LIST ilist;
    ThreadControl();          // 线程管理构造函数，用于线程中变量初始化
    void ImageProduce();      // 短焦摄像头获取图像线程
    void ImageProcess(ros::Publisher pub_);      // 图像处理线程，用于自瞄，能量机关识别
    void ImageWrite();        // 用于图像保存线程
    void vision_ImageProduce(HIKvison myvision,float gain);
    void serial_receive();
private:
    Mat IMG;
    Mat image_;
    OtherParam other_param;

    bool end_thread_flag = false; // 设置结束线程标志位，负责结束所有线程。
};
void receive();

#endif // THREAD_CONTROL_H
