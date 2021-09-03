#include <opencv2/opencv.hpp>
#include "armor_detection/armor_detect.h"
#include "common/serial/serial_port.h"
#include "common/thread_control/thread_control.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <thread>

#include "camera/HIKvison.h"

#include <ros/ros.h>
#include "roborts_msgs/AmbitionVision.h"

using namespace cv;
using namespace std;


//#define video_save


int main(int argc, char **argv) {

    ros::init(argc, argv, "ambition_vision2021_node");
    ros::NodeHandle ros_nh_;
    ros::Publisher ros_ambition_vision_pub_;
    ros_ambition_vision_pub_ = ros_nh_.advertise<roborts_msgs::AmbitionVision>("ambition_vision", 1);

    ThreadControl ImageControl;

    HIKvison myvision;

    if(myvision.reset(ImageControl.ilist,1280,960,500.0,5000.0,12,1)==0)//分辨率1280*960；帧率最高500，曝光时间6000us;gamma0.47;
    {cout<<"feizhengc"<<endl;
        return  0;}
    // 开启相关线程

    // 图像生成线程
    //parameter 线程函数 ImageControl的地址作为指针对象提供给函数
    //std::thread produce_task(&ThreadControl::ImageProduce, &ImageControl);  // & == std::ref()

    //工业相机线程
    std::thread vision_proceduce_task(&ThreadControl::vision_ImageProduce, &ImageControl,myvision,1.00);

    // 图像处理线程（自瞄、打符、串口）
    std::thread process_task(&ThreadControl::ImageProcess, &ImageControl, ros_ambition_vision_pub_);

    //图像生成线程（保存录像）
#ifdef video_save
    std::thread save_video(&ThreadControl::ImageWrite,&ImageControl);
#endif
    //串口接收线程
    std::thread serial_task(&ThreadControl::serial_receive,&ImageControl);

    //produce_task.join();
    vision_proceduce_task.join();
    process_task.join();
#ifdef video_save
    save_video.join();
#endif
    serial_task.join();
    myvision.close(myvision.Handle);

    return 1;
}
