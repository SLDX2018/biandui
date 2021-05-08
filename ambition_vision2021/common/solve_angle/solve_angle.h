#ifndef SOLVE_ANGLE_H
#define SOLVE_ANGLE_H

#include <opencv2/opencv.hpp>
#include "../../base/base.h"

using namespace std;
using namespace cv;
class SolveAngle
{
public:
    SolveAngle(){}
    SolveAngle(float c_x,float c_y,float c_z,float barrel_y);
    //普通角度解算
    void getAngle(vector<Point2f> &image_point, float ballet_speed, float &angle_x,float &angle_y,float &dist,Mat &img);
    void getBuffAngle(bool flag, vector<Point2f>& image_point, float ballet_speed, float buff_angle, float pre_angle, float gimbal_pitch, float &angle_x, float &angle_y, float &dist);
    //------------Ambition---------
    void getAngle_Ambition(vector<Point2f> &image_point,float ballet_speed,float &angle_x,float &angle_y,float &dist);
    void Generate3DPoints(uint mode,Point2f offset_point);
    float getBuffPitch(float dist, float tvec_y, float ballet_speed);
    void compensateOffset(float &angle_x, float &angle_y, float &dist);
    void compensateGravity(float ballet_speed,float &angle_x, float &angle_y, float &dist);
    //------------Ambition----------
    Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
    Mat distCoeffs = Mat::zeros(5, 1, CV_64F);
//    Mat rVec;//init rvec 旋转向量
//    Mat tVec;//init tvec 平移向量
    Mat rVec = cv::Mat::zeros(3, 1, DataType<double>::type);//旋转向量
    Mat tVec = cv::Mat::zeros(3, 1, DataType<double>::type);//平移向量
    vector<Point3f> objectPoints;//真实点坐标
    Mat object_point_mat;

    float distance;
    float height_world = 80;//small
    int f_=1500;
    float overlap_dist = 100000.0;
    float barrel_ptz_offset_y = 20; //mm +pyz is up barrel
        float barrel_ptz_offset_x = -0;
    //base ptz position
    float ptz_camera_x=0;  //+right
    double ptz_camera_y=-48.5; //+lower
    float ptz_camera_z=+135; //+front;  //+ camera is front ptz
    float scale = 0.99f;


public:
    float buff_h;
    float H = 59*0.8;
};



#endif // SOLVE_ANGLE_H



