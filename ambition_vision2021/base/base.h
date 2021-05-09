#ifndef BASE
#define BASE

//#define imblue

struct OtherParam
{
# ifdef imblue
    int color = 0;       // 我方车辆颜色，0是蓝色，1是红色。用于图像预处理
#endif
#ifndef imblue
    int color=1;
#endif
    float time;         //赛场时间
    double bullet_speed=12;  //子弹射速
    int mode = 0;        // 视觉模式，0是自瞄模式，1是能量机关模式
    int hero_mood=0;     //1大枪模式，0小枪模式
    int cap_mode = 0;    // 摄像头类型，0是短焦摄像头，1是长焦摄像头
    float gimbal_data;
    float buff_offset_x;
    float buff_offset_y;

};
#define WAITKEY 1
// ****** common ******//
#define END_THREAD if(end_thread_flag) return;

//****** 摄像头信息 *****
#define VIDEO_WIDTH 640
#define VIDEO_HEIGHT 360
#define BUFFER_SIZE 1

//****** 线程使能 *****
#define GET_STM32_THREAD

//****** 大风车识别配置 *****
//#define BUFF_TRACK_BAR //uuse 装甲板滑动条
#define COLOR_TH 20   //颜色阈值
#define DEBUG_BUFF_DETECT  //比赛要调
//****** 装甲板识别配置 *****
// #define ARMOR_TRACK_BAR //uuse 装甲板滑动条
#define DEBUG_ARMOR_DETECT  //调试装甲板绘制信息

#define ROI_ENABLE      // 启用roi


// 固定状态
#define BULLET_SPEED 28.5  //射速
#define BUFF_H 619
#define BUFF_DISTANCE 7140
//****** 角度解算配置 *****
//#define SET_ZEROS_GRAVITY
//#define SIMPLE_SOLVE_ANGLE_FOR_ARMOR_DETECT
// 摄像头坐标系到云台坐标系
#define LONG_X 0.0f
#define LONG_Y 40.7f
#define LONG_Z -123.0f
#define PTZ_TO_BARREL 0.0f   // 补兵激光在２３ｍｍ下方

#endif // BASE


