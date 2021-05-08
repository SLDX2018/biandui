#ifndef BUFF_DETECT_H
#define BUFF_DETECT_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include "../base/base.h"
#include "../common/solve_angle/solve_angle.h"
using namespace cv;
using namespace std;

/**
 * @brief debug
 */
#define BUFF_DETECT_DEBUG

#ifdef BUFF_DETECT_DEBUG
// ------ debug buff ------
#define FUSION_MINAREA_ELLIPASE  //使用最小面积矩形拟合
#define DEBUG_DRAW_CONTOURS  //调试扇叶内外轮廓
#define IMSHOW_2_ROI   //展示两块roi的强度
#define DEBUG_PUT_TEST_ANGLE  //测试击打点位置
#define DEBUG_PUT_TEST_TARGET  //测试击打点
#define DEBUG_DRAW_TARGET   //画出装甲板target
#define DIRECTION_FILTER  //方向滤波器
//#define AREA_LENGTH_ANGLE 3 // 1:area 2:length 3:diff_angle  //调试扇叶内外轮廓相关信息
// ------ debug buff ------
#endif

#define TEST_OTSU  //最大类间方差,自适应阈值,所有参数调小，二值化图更细节,

typedef enum{UNKOWN,INACTION,ACTION}ObjectType;  //未知，未激活，激活
/**
 * @brief 矩形类物体属性
 * 在逻辑识别部分需要修改原有旋转矩形属性
 * 在计算0-360角度上需要用到旋转矩形原始参数
 */
class Object
{
public:
    Object(){}
    void DrawTarget(Mat &img)
    {
        if(type_ == INACTION)
            circle(img, small_rect_.center, 3, Scalar(0, 0, 255), -1);
        else if(type_ == ACTION)
            circle(img, small_rect_.center, 3, Scalar(255, 255, 255), -1);
        else
            circle(img, small_rect_.center, 3, Scalar(255, 255, 255), 1);
    }

    void UpdateOrder(); // 更新能量机关装甲板的绝对位置
    void KnowYourself(Mat &img);    //判断能量机关扇叶的状态（激活　未激活）

    RotatedRect small_rect_;    // 能量机关扇叶内轮廓
    RotatedRect big_rect_;  // 能量机关扇叶外轮廓
    vector<Point2f> points_2d_; // ｐｎｐ角度解算的四个点,装甲板四个点
    float angle_;
    float diff_angle;//装甲板好扇叶的角度差
    int type_ = UNKOWN;
};

/**
 * @brief BuffDetectTask 能量机关识别总任务，每帧调用
 * @param img 摄像头获取的RGB图像
 * @return 1是发现目标，0是未发现目标
 */
class BuffDetector
{
public:
    BuffDetector(){
        solve_angle_long_ = SolveAngle(LONG_X, LONG_Y, LONG_Z, PTZ_TO_BARREL);

    }
    ~BuffDetector(){}

    int BuffDetectTask(Mat& img, OtherParam param);
    void getAngle(float &yaw, float &pitch ,float &distance){
        yaw = (angle_x_);
        pitch =(angle_y_);
        distance=distance_;
    }

    /**
     * @brief 辨别能量机关旋转方向
     * 根据每次识别得到的角度进行滤波，判断能量机关旋转方向
     */
    float getDistance(){
        return distance_;
    }

    /**
     * @brief 判断方向
     * @param 输入能量机关角度
     * @return
     */
    int getSimpleDirection(float angle);
    int Get_Pre_Rect(Mat &img, RotatedRect small_rect,Point2f rect_center);
    int PreTask(Mat &img);

private:
    bool DetectBuff(Mat& img, OtherParam other_param);

    // 外部参数
private:
    int color_;
    float gimbal;

    //预测参数
public:
     vector<Point2f> points; //装甲板中心点，拟合圆使用
      double radium;
      Point2f cente;
      Point2f centter;
      Object target;
      vector<Point2f> pre_points_2d;
      Point2f last_point;



    // debug参数
public:
    int buff_offset_x_ = 101;//id:3 112;// id:2 130;
    int buff_offset_y_ = 108;//id:3 69;// id:2 135;
//    int begin_offset_x_ = BUFF_OFFSET_x;
//    int begin_offset_y_ = BUFF_OFFSET_y;


//    int world_offset_x_ = WORLD_OFFSET_X;
    int world_offset_y_ = 500;
    int pitch_offset = 2000;
    int color_th_ = COLOR_TH;
    int gray_th_ = 50;
    float buff_angle_ = 0;
    float diff_angle_ = 0;
    int area_ratio_ = 500;

    //相关类申明
//    AutoControl auto_control;
private:
    SolveAngle solve_angle_long_;
//    MainWindow *w_;

private:
    float angle_x_ = 0;
    float angle_y_ = 0;
    float last_angle_x_ = 0;
    float last_angle_y_ = 0;
    float distance_ = 0;
    vector<Point2f> points_2d;
    int action_cnt_ = 0;
    vector<float> vec_diff_angle;

private:
    float d_angle_ = 0;
    float r = 0.1; //刷新率
    int last_angle_;
    int find_cnt = 0;
    int direction_tmp=0;

public:
    int waitkey_flag = 1;
    int imshow_flag = 1;


    int command = 0;
};



#endif // BUFF_DETECT_H
