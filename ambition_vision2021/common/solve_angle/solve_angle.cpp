#include "solve_angle.h"

using namespace cv;

SolveAngle::SolveAngle(float c_x,float c_y,float c_z,float barrel_y)
{
    //相关坐标转化偏移数据
//    barrel_ptz_offset_y=barrel_y;
//    ptz_camera_x = c_x;
//    ptz_camera_y = c_y;
//    ptz_camera_z = c_z;

    //相机内参和畸变矩阵

    cameraMatrix.at<double>(0, 0) = 1.770531438196095e+03;
    cameraMatrix.at<double>(0, 1) = 0;
    cameraMatrix.at<double>(0, 2) = 6.947535889724644e+02;
    cameraMatrix.at<double>(1, 1) = 1.769972907864963e+03;
    cameraMatrix.at<double>(1, 2) = 5.696223248924433e+02;

    distCoeffs.at<double>(0, 0) = -0.068292377580031;
    distCoeffs.at<double>(1, 0) = 0.250857313517954;
    distCoeffs.at<double>(2, 0) = 0;
    distCoeffs.at<double>(3, 0) = 0;
    distCoeffs.at<double>(4, 0) = 0;

    Generate3DPoints(0,Point2f(0,0));
    Mat(objectPoints).convertTo(object_point_mat,CV_32F);
//  Mat rVec = cv::Mat::zeros(3, 1, DataType<double>::type);
//  Mat tVec = cv::Mat::zeros(3, 1, DataType<double>::type);


}

void SolveAngle::getAngle(vector<Point2f> &image_point, float ballet_speed, float &angle_x, float &angle_y, float &dist,Mat &img)
{
    //姿态解算
    solvePnP(objectPoints,image_point,cameraMatrix,distCoeffs,rVec,tVec,false, SOLVEPNP_P3P);

    double x=(tVec.at<double>(0,0));
    double y=(tVec.at<double>(1,0));
    double z=(tVec.at<double>(2,0));
    char a[50];
    char b[50];
    char c[50];
    sprintf(a,"yaw:%lf",x);
    sprintf(b,"pithc:%lf",y);
    sprintf(c,"distance:%lf",z);
    putText(img,a,Point(100,30),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,255));
    putText(img,b,Point(100,40),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,255));
    putText(img,c,Point(100,50),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,255));


//    if(tVec.at<double>(2, 0) >= 4000)
//    {
//        float height = image_point.at(2).y-image_point.at(1).y;
//        double fy = cameraMatrix.at<double>(1,1);//fy
//        dist = static_cast<float>(fy*H/height);
//        tVec.at<double>(2,0) = dist;
//        float y_offset_px = (image_point.at(2).y+image_point.at(1).y)/2-img.size().height/2;
//        float y_offset_real = y_offset_px/cameraMatrix.at<double>(1,1)*dist;
//        float x_offset_px = (image_point.at(2).x+image_point.at(3).x)/2-img.size().width/2;
//        float x_offset_real = x_offset_px/cameraMatrix.at<double>(0,0)*dist;
//        cout<<"y_offset_real"<<y_offset_real<<endl;
//        cout<<"x_offset_real"<<x_offset_real<<endl;
//        tVec.at<double>(0,0) = x_offset_real;
//        tVec.at<double>(1,0) = y_offset_real;
//    }
   // cout<<"distance: "<<tVec.at<double>(2, 0)<<endl;
    float dh = ((image_point.at(3).y-image_point.at(0).y) + (image_point.at(2).y-image_point.at(1).y))/2;
    float state_dist = height_world * f_/dh;//z
     distance=pow((x*x+y*y+z*z),0.5)/1000;

    // 坐标系转换 -摄像头坐标到云台坐标
    double theta = -atan(static_cast<double>(-ptz_camera_y + barrel_ptz_offset_y))/static_cast<double>(overlap_dist);
//    float overlap_dist = 100000.0;
    double r_data[] = {1,0,0,0,cos(theta),sin(theta),0,-sin(theta),cos(theta)};
    double t_data[] = {static_cast<double>(ptz_camera_x),static_cast<double>(ptz_camera_y),static_cast<double>(ptz_camera_z)};
    //cout<<"ptzy"<<ptz_camera_y<<endl;

    Mat r_camera_ptz(3,3,CV_64FC1,r_data);
    Mat t_camera_ptz(3,1,CV_64FC1,t_data);
    Mat position_in_ptz;
    position_in_ptz = /*r_camera_ptz **/ tVec+ t_camera_ptz;//x+0;y-52.5;z+135
    cout<<position_in_ptz<<'\n';

    //计算子弹下坠补偿
    double bullet_speed = static_cast<double>( ballet_speed);
    const double *_xyz = (const double *)position_in_ptz.data;
    double down_t = 0.0;
    if(bullet_speed > 10e-3)
        down_t = _xyz[2] /1000.0 / bullet_speed;//(mm2m)/speed
    double offset_gravity = 0.5 * 9.8 * down_t*down_t * 1000;//m2mm
#ifdef SET_ZEROS_GRAVITY
    offset_gravity = 0;
#endif
    //版本一
//    {
//    double tan_pitch = _xyz[1] / sqrt(_xyz[0]*_xyz[0] + _xyz[2] * _xyz[2]);
//    double tan_yaw = _xyz[0] / _xyz[2];
//    angle_y = atan(tan_pitch) * 180 / CV_PI;
//    angle_x = atan(tan_yaw) * 180 / CV_PI;
//    float camera_target_height = distance * sin(angle_y / 180 * CV_PI);
//    float gun_target_height = camera_target_height;
//    float gun_pitch_tan = gun_target_height / (distance * cos(angle_y / 180 * CV_PI));
//    angle_y = atan(gun_pitch_tan) / CV_PI * 180;
//    putText(img,"angle_Y1:"+to_string(angle_y),Point(100,120),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,255));

//    float compensateGravity_pitch_tan = tan(angle_y/180*CV_PI) - (0.5*9.8*(distance / ballet_speed)*(distance / ballet_speed)); // cos(angle_y/180*CV_PI);
//    angle_y = atan(compensateGravity_pitch_tan)/CV_PI*180;
//    putText(img,"angle_X:"+to_string(angle_x),Point(100,140),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,255));
//    putText(img,"angle_Y:"+to_string(angle_y),Point(100,160),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,255));
//    putText(img,"distance:"+to_string(distance),Point(100,180),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,255));
//    }

     //计算角度
    double xyz[3] = {_xyz[0], _xyz[1] - offset_gravity, _xyz[2]};
    if(barrel_ptz_offset_y != 0)
    {
        double alpha = 0.0, Beta = 0.0;
        alpha = asin(static_cast<double>(barrel_ptz_offset_y)/sqrt(xyz[1]*xyz[1] + xyz[2]*xyz[2]));

        if(xyz[1] < 0)
        {
            Beta = atan(-xyz[1]/xyz[2]);
            angle_y = static_cast<float>(-(alpha+Beta)); //camera coordinate
            char d[50];
            sprintf(d,"yangle1:%f",angle_y);
            putText(img,d,Point(100,100),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,255));

        }else if(xyz[1] < static_cast<double>(barrel_ptz_offset_y))
        {
            Beta = atan(xyz[1]/xyz[2]);
            angle_y = static_cast<float>(-(alpha - Beta));
            char e[50];
            sprintf(e,"yangle2:%lf",y);
            putText(img,e,Point(100,120),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,255));

        }else
        {
            Beta = atan(xyz[1]/xyz[2]);
            angle_y = static_cast<float>((Beta-alpha));   // camera coordinate
            char f[50];
            sprintf(f,"yangle3:%f",angle_y);
            putText(img,f,Point(100,140),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,255));
        };
    }else
    {
        angle_y = static_cast<float>(atan2(xyz[1],xyz[2]));
        char g[50];
        sprintf(g,"yangle4:%f",angle_y);
        putText(img,g,Point(100,160),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,255));

    }

    if(barrel_ptz_offset_x != 0)
    {
        double alpha = 0.0, Beta = 0.0;
        alpha = asin(static_cast<double>(barrel_ptz_offset_x)/sqrt(xyz[0]*xyz[0] + xyz[2]*xyz[2]));
        if(xyz[0] > 0)
        {
            Beta = atan(-xyz[0]/xyz[2]);
            angle_x = static_cast<float>(-(alpha+Beta)); //camera coordinate

        }else if(xyz[0] < static_cast<double>(barrel_ptz_offset_x))
        {
            Beta = atan(xyz[0]/xyz[2]);
            angle_x = static_cast<float>(-(alpha - Beta));

        }else
        {
            Beta = atan(xyz[0]/xyz[2]);
            angle_x = static_cast<float>(Beta-alpha);   // camera coordinate

        }
    }else{
        angle_x = static_cast<float>(atan2(xyz[0],xyz[2]));

    }
      double offset_angle=0.5*asin(_xyz[2]/(bullet_speed*bullet_speed*9.8*1000));
    angle_x = static_cast<float>(angle_x) * 180/CV_PI;
    angle_y = static_cast<float>(angle_y) * 180/CV_PI;

    char g[50];
    sprintf(g,"yangle4:%f",angle_y);
    putText(img,g,Point(100,160),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,255));
    dist = static_cast<float>(_xyz[2]);
    char h[50];
    sprintf(h,"x_angle:%f",angle_x);
    putText(img,h,Point(100,180),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,255));

    cout<<"angle_x:"<<angle_x<<endl;
    cout<<"angle_y:"<<angle_y<<endl;

//      waitKey(0);
    if(dist >= 8500)
    {

    }

}
void SolveAngle::getBuffAngle(bool flag, vector<Point2f> &image_point, float ballet_speed
                              , float buff_angle, float pre_angle, float gimbal_pitch
                              , float &angle_x, float &angle_y, float &dist)
{
    // 姿态结算
    solvePnP(objectPoints, image_point, cameraMatrix, distCoeffs, rVec, tVec);
    // 距离解算 参考能量机关尺寸
    float H = BUFF_H;   // 大能量机关最底部装甲板桥面地面高度
    float h = 430;      // 步兵枪口距离桥面高度mm
    float D = BUFF_DISTANCE;    //步兵距离能量机关水平距离
    float delta_h = H - h;
    float predict_buff_angle = buff_angle + pre_angle;
    buff_h = 700*sin(predict_buff_angle *CV_PI/180)+700;   // 计算风车相对最底面装甲高度　０－１4００
    float target_h = delta_h + buff_h;
    dist = sqrt(pow(target_h, 2) + pow(D, 2));

    tVec.at<double>(2,0) = dist;
    // 坐标系转换 -摄像头坐标到云台坐标
    double theta = -atan(static_cast<double>(ptz_camera_y + barrel_ptz_offset_y))/static_cast<double>(overlap_dist);
    double r_data[] = {1,0,0,0,cos(theta),sin(theta),0,-sin(theta),cos(theta)};
    double t_data[] = {static_cast<double>(ptz_camera_x),static_cast<double>(ptz_camera_y),static_cast<double>(ptz_camera_z)};
    Mat t_camera_ptz(3,1,CV_64FC1,t_data);
    Mat r_camera_ptz(3,3,CV_64FC1,r_data);
    Mat position_in_ptz;
    position_in_ptz = /*r_camera_ptz **/ tVec;// + t_camera_ptz;

    const double *_xyz = (const double *)position_in_ptz.data;

    // 计算角度
    double xyz[3] = {_xyz[0], _xyz[1], dist};

    angle_x = static_cast<float>(atan2(xyz[0],xyz[2]));
    angle_x = static_cast<float>(angle_x) * 180/CV_PI;
    float gimbal_y = dist * sin(gimbal_pitch*CV_PI/180);
    float thta = -static_cast<float>(atan2(xyz[1],dist)); // 云台与目标点的相对角度
    float balta = static_cast<float>(atan2(target_h,dist)) - thta; // 云台与桥面的相对角度

#ifdef SET_ZEROS_GRAVITY
    angle_y = static_cast<float>(atan2(xyz[1],xyz[2]));
#else

#ifdef USE_GIMBAL_OFFSET
    //    if(flag)
    angle_y = -getBuffPitch(dist/1000, (gimbal_y - xyz[1] )/1000, ballet_speed);
    angle_y += gimbal_pitch*3.1415926f/180;
#else
    //    else
    angle_y = -getBuffPitch(dist/1000, (target_h)/1000, ballet_speed);
    angle_y += balta;
#endif
#endif
    angle_y = static_cast<float>(angle_y) * 180/CV_PI ;
}

float SolveAngle::getBuffPitch(float dist, float tvec_y, float ballet_speed)
{
    // 申明临时y轴方向长度,子弹实际落点，实际落点与击打点三个变量不断更新（mm）
    float y_temp, y_actual, dy;
    // 重力补偿枪口抬升角度
    float a = 0.0;
    float GRAVITY = 9.7887f; //shenzhen 9.7887  zhuhai
    y_temp = tvec_y;
    // 迭代求抬升高度
    for (int i = 0; i < 10; i++) {
        // 计算枪口抬升角度
        a = (float) atan2(y_temp, dist);
        // 计算实际落点
        float t, y = 0.0;
        t = dist / (ballet_speed * cos(a));
        y_actual = ballet_speed * sin(a) * t - GRAVITY * t * t / 2;
        dy = tvec_y - y_actual;
        y_temp = y_temp + dy;
        // 当枪口抬升角度与实际落点误差较小时退出
        if (fabsf(dy) < 0.01) {
            break;
        }
    }
    return a;
}

void SolveAngle::getAngle_Ambition(vector<Point2f> &image_point, float ballet_speed, float &angle_x, float &angle_y, float &dist)
{
    Point3f offset_(0,0,0);
    float offset_yaw_ = 0.0;
    float offset_pitch_ = 0.0;
    solvePnP(objectPoints,image_point,cameraMatrix,distCoeffs,rVec,tVec);
    Point3f position = static_cast<Point3f>(tVec);
    angle_x =(float)(atan2(position.x + offset_.x,position.z + offset_.z))*180.0f/CV_PI+(float)(offset_yaw_);
    angle_y =(float)(atan2(position.y + offset_.y,position.z + offset_.z))*180.0f/CV_PI+(float)(offset_pitch_);
    dist = position.z;
}

void SolveAngle::compensateOffset(float &angle_x, float &angle_y, float &dist)//ptz
{
    const auto offset_z = 115.0; //mm
    const auto& d = dist;
    const auto theta_y = angle_x / 180 * CV_PI;
    const auto theta_p = angle_y / 180 * CV_PI;
    const auto theta_y_prime = atan2((d*sin(theta_y)) , (d*cos(theta_y) + offset_z));
    const auto theta_p_prime = atan2((d*sin(theta_p)) , (d*cos(theta_p) + offset_z));
    const auto d_prime = sqrt(pow(offset_z + d * cos(theta_y), 2) + pow(d*sin(theta_y), 2));
    angle_x = theta_y_prime / CV_PI * 180;
    angle_y = theta_p_prime / CV_PI * 180;

}

void SolveAngle::compensateGravity(float bullet_speed, float &angle_x, float &angle_y, float &dist)
{
    const auto& theta_p_prime = angle_y / 180 * CV_PI;
    const auto& d_prime = dist;
    const auto& v = bullet_speed;
    const auto& t = d_prime/(v*cos(theta_p_prime));
    const auto theta_p_prime2 = atan2(d_prime*tan(theta_p_prime)-(0.5*9.8*d_prime)/(v*v*cos(theta_p_prime)*cos(theta_p_prime)),d_prime);
    angle_y = theta_p_prime2 / CV_PI * 180;
}

void SolveAngle::Generate3DPoints(uint mode, Point2f offset_point)
{
    objectPoints.clear();
    float x, y, z, width = 0.0f, height = 0.0f;
    switch(mode){
    case 1:   //small_armor
        width  = 130*1.1;
        height = 60*1.1;
        break;
    case 0:   //big_armor
        width  = 230*1.02;
        height = 55*1.02;
        break;
    case 2:   //buff_big_armor
        width  = 230*1.15;
        height = 60*1.15;
        break;
    }

    x = -width/2; y= -height/2; z=0;
    objectPoints.push_back(cv::Point3f(x,y,z)+cv::Point3f(offset_point.x,offset_point.y,0));
    x = width/2; y= -height/2; z=0;
    objectPoints.push_back(cv::Point3f(x,y,z)+cv::Point3f(offset_point.x,offset_point.y,0));
    x = width/2; y= height/2; z=0;
    objectPoints.push_back(cv::Point3f(x,y,z)+cv::Point3f(offset_point.x,offset_point.y,0));
    x = -width/2; y= height/2; z=0;
    objectPoints.push_back(cv::Point3f(x,y,z)+cv::Point3f(offset_point.x,offset_point.y,0));
}




