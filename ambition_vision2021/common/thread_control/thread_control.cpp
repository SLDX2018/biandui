#include "thread_control.h"
#include "../../camera/camera_device.h"
#include "../../armor_detection/armor_detect.h"
#include "../../common/serial/serial_port.h"
#include "../../base/base.h"
#include "../../buff_detection/buff_detect.h"
#include "../../camera/save_video.h"
#include <iostream>
#include <unistd.h>
#include <chrono>

#define BUFF_VIDEO_PATH "/home/sldx/桌面/机器人视角红色背景暗24.mp4"
#define ARMOR_VIDEO_PATH "/home/coumputer/桌d面/炮台素材红车旋转-ev--3.MOV"

// #define IMAGESHOW  //展示图片
#define SHOW_FRAME //显示帧率
//#define DEBUG_VIDEO 1  //暂时没用到，有bug

//#define DEBUG_IMAGE  //大风车程序
#define SAVE_VIDEO_THREAD
//#define dubug_recive
using namespace std;


static volatile unsigned int produce_index;     // 图像生成序号，用于线程之间逻辑
static volatile unsigned int gimbal_data_index;     // gimbal data 生成序号，用于线程之间逻辑
static volatile unsigned int consumption_index; // 图像消耗序号
static volatile unsigned int save_image_index;  // 保存图像序号

int serial_1 = -1;//句柄
#ifdef GET_STM32_THREAD
SerialPort Serial = SerialPort(serial_1,"/dev/serial_sdk",115200);                 // pc与stm32之间的串口通信
#endif
float a_coefficient=0;



ThreadControl::ThreadControl()
{
    cout << "THREAD TASK ON !!!" << endl;
}


//工业相机生成线程
void ThreadControl::vision_ImageProduce(HIKvison myvision,float gain ){

    while(1){

        while(produce_index - consumption_index >= BUFFER_SIZE)
            END_THREAD;
          myvision.visoncamera_get_image(myvision.Handle,myvision.stImageinfo,gain);
          image_=myvision.Image;
           ++produce_index;
          }
     END_THREAD;
}

 //图像生成线程
void ThreadControl::ImageProduce()
{
    cout << " ------ camera produce task on !!! ------ " << endl;
    CameraInit camera(0);
    while(1)
    {
        // 等待图像进入处理瞬间，再去生成图像
        while(produce_index - consumption_index >= BUFFER_SIZE)
            END_THREAD;
        camera.cap >>image_;
//        resize(image_, image_, Size(1280, 720), INTER_AREA);
        ++produce_index;
    }
    END_THREAD;
}

// 图像处理线程
void ThreadControl::ImageProcess()
{


    cout << " ------ image process task on !!! ------" << endl;
    ArmorDetector armor_detector ;//自瞄检测容器
    BuffDetector buff_detector;//buff检测容器
    VisionData  tx_data;       // 串口发送stm32数据结构initial
    {
        tx_data.mode='6';
        tx_data.mode='6';
        tx_data.pitch_angle.f=6;//测试接受的
        tx_data.yaw_angle.f = 6;
        tx_data.distance.f = 6;
        tx_data.command = 6;
    }

#ifdef ARMOR_TRACK_BAR
    // 装甲板调试参数
    //颜色包围灰度
#ifdef imblue
    armor_detector.color_th_ = 60;//调小
    armor_detector.gray_th_ = 180;//调大
#endif
#ifndef imblue
    armor_detector.color_th_=70;
    armor_detector.gray_th_=180;
#endif

    namedWindow("ArmorParam");
    createTrackbar("armor_gray_th", "ArmorParam", &armor_detector.gray_th_, 255);
    createTrackbar("armor_color_th", "ArmorParam", &armor_detector.color_th_, 255);
    createTrackbar("short_offset_x","ArmorParam",&armor_detector.short_offset_x_,200);
    createTrackbar("short_offset_y","ArmorParam",&armor_detector.short_offset_y_,200);
#endif

#ifdef BUFF_TRACK_BAR   //061987685262769
    // 能量机关调试参数
    namedWindow("BuffParam");
    createTrackbar("buff_gray_th", "BuffParam", &buff_detector.gray_th_, 255);
    createTrackbar("buff_color_th", "BuffParam", &buff_detector.color_th_, 255);
    createTrackbar("buff_offset_x_","BuffParam",&buff_detector.buff_offset_x_,200);
    createTrackbar("buff_offset_y_","BuffParam",&buff_detector.buff_offset_y_,200);
//    createTrackbar("world_offset_x","BuffParam",&buff_detector.world_offset_x_,1000);
//    createTrackbar("fire_max_cnt","BuffParam",&buff_detector.auto_control.fire_task.max_cnt_,200);
//    createTrackbar("reset_cnt","BuffParam",&buff_detector.auto_control.reset_task.max_cnt_,200);
//    createTrackbar("repeat_time","BuffParam",&buff_detector.auto_control.fire_task.repeat_time,2000);
//    createTrackbar("fire","BuffParam",&buff_detector.auto_control.fire_task.fire_flag,1);
//    createTrackbar("repeat fire","BuffParam",&buff_detector.auto_control.fire_task.repeat_fire_flag,1);
#endif

#ifdef DEBUG_VIDEO  //暂时没用到，有bug
 if(DEBUG_VIDEO == 0)
    VideoCapture cap(ARMOR_VIDEO_PATH);
 else
//    VideoCapture cap(BUFF_VIDEO_PATH);
    int index;
//    int total_frame=(int)cap.get(CV_CAP_PROP_FRAME_COUNT);
#endif
    //VideoCapture cap(BUFF_VIDEO_PATH);
    Mat image;
    float angle_x = 0.0, angle_y = 0.0;//定义局部变量
    float distance =  0.0;
    int command = 0;

    int last_angle_x=0;

    while(1)
    {
//#ifdef DEBUG_IMAGE
//        if(other_param.mode==0)//风车模式
//          {whilalphae(produce_index - consumption_index <= 0){
//            END_THREAD;
//          }
//        cap.read(image);}
//#else
//        // 等待图像生成后进行处理
//        else if(other_param.mode==1)//自瞄模式
//        {
#ifdef SHOW_FRAME
        auto t1=chrono::high_resolution_clock::now();
#endif
          while(produce_index - consumption_index <= 0){
           END_THREAD;
          }

        image_.copyTo(image);
    //}
//endif

        if(other_param.mode == 0)
        {
             //***************************auto_mode***********************************
            consumption_index++;
            //            TIME_START(t);
            command = armor_detector.ArmorDetectTask(image, other_param);
            //            TIME_END(t);
            armor_detector.getAngle(angle_x, angle_y,distance);

            angle_x=(angle_x)*(a_coefficient+1);

            //last_angle_x=angle_x;
        }
        else
        {
            //VideoCapture cap(BUFF_VIDEO_PATH);
           // cap.read(image);
            //***************************buff_mode***********************************
            consumption_index++;
            command = buff_detector.BuffDetectTask(image, other_param);
            buff_detector.getAngle(angle_x, angle_y,distance);
        }
        //tx_data赋值
        {
            if(other_param.mode == 0)
                tx_data.mode = 0x00;
            else
                tx_data.mode= 0x01;

            tx_data.yaw_angle.f = angle_x;
            tx_data.pitch_angle.f =angle_y;
            tx_data.distance.f = distance;
//                       tx_data.yaw_angle.d = 15;
//                       tx_data.pitch_angle.d = 15;
//                       tx_data.distance.f = 20.0;
            if(command ==0)
                tx_data.command = 0x00;
            if(command ==1)
                tx_data.command = 0x01;
        }
       // Serial.read_data();
       // Serial.deal_date();
        {
//            cout<<"debug distance: "<<tx_data.distance.f<<endl;
//            cout<<"debug angle_x:"<<tx_data.yaw_angle.d<<endl;
//            cout<<"debug angle_y:"<<tx_data.pitch_angle.d<<endl;
        }
        static int flag=0;
        if(command==1)
        {
            flag=0;
            Serial.send_data(serial_1,tx_data);
 #ifdef dubug_recive
            {
                int num;
                unsigned char Rdata[18]; //test rx_data initial
                VisionData enemy;
                {
                    enemy.mode='6';
                    enemy.pitch_angle.d=6;//测试接受的
                    enemy.yaw_angle.d = 6;
                    enemy.distance.f = 6;
                    enemy.command = 6;
                }
                Serial.getVisionData(Rdata,enemy);
                num=read(serial_1, Rdata, 18);
            //debug 接受数据
            cout<<"receive num bit:"<<num<<endl;
            printf("yaw_anglereceive mode:%x\n",enemy.mode);
            cout<<"receive mode:"<<hex<<enemy.mode<<endl;
            cout<<"receive yaw:" <<enemy.yaw_angle.d << endl;
            cout<<"receive pitch:" <<enemy.pitch_angle.d << endl;
            cout<<"receive dis:"<<enemy.distance.f<<endl;
            printf("receive command:%x\n",enemy.command);
            cout<<"receive command‵:"<<enemy.command<<endl<<endl;
               }
#endif
        }else if(command==0){
            flag=1;
            tx_data.yaw_angle.f = 200;
            tx_data.pitch_angle.f =200;
            Serial.send_data(serial_1,tx_data);
        }
#ifdef SHOW_FRAME
        auto t2=chrono::high_resolution_clock::now();
        double time=(static_cast<chrono::duration<double,std::milli>>(t2 - t1)).count();
        double FPS=1000/time;
        cout<<"FPS:"<<FPS<<'\n';
#endif
//IMG=image;
#ifdef WAITKEY
  #ifdef IMAGESHOW
        imshow("image", image);
  #endif
        char key;   //按键设置
        bool key_q=(key=='q'||key=='Q');
        bool key_f=(key=='f'||key=='F');//0x66
        bool key_s=(key=='s'||key=='S');
        bool key_m=(key=='m'||key=='M');//0x6d
        key = waitKey(WAITKEY);
        if(key_q){
            end_thread_flag = true;}
        if(key_s){
           waitKey(0);
        }else if(key_f||Serial.mood=='f'){
            other_param.mode = 1;
        }else if(key_m||Serial.mood=='m'){
            other_param.mode = 0;
        }
#endif
                END_THREAD;
    }
}


void ThreadControl::serial_receive(){
    Serial.read_data();
    Serial.deal_date(other_param);

}



#ifdef SAVE_VIDEO_THREAD
void ThreadControl::ImageWrite()
{
    cout << " ------ IMAGE WRITE TASK ON !!! ------" << endl;
    SaveVideo writer;
    while(writer.getState()){
        while(static_cast<int>(produce_index - save_image_index) <= 0){

            END_THREAD;
        }
        //cout<<1<<endl;
        Mat img_tmp;
        //resize(IMG,IMG,Size(640,480),0,0,INTER_LINEAR);
        //cout<<2<<endl;
        image_.copyTo(img_tmp);

        resize(img_tmp,img_tmp,Size(640,480),0,0,INTER_LINEAR);
//        if(img_tmp.rows == 960)
//            copyMakeBorder(img_tmp, img_tmp, 0, 120, 0, 0, BORDER_CONSTANT, Scalar::all(0));
          writer.updateImage(img_tmp);
        save_image_index++;
    }
}
#endif















