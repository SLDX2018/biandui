#include "buff_detect.h"
#include<iostream>

float small_angle;
float big_angle;

double Point_distance(Point2f p1,Point2f p2)
{
    double Dis=pow(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2),0.5);
    return Dis;
}

int GetRectIntensity(const Mat &img, Rect rect){
    if(rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y < 1
            || rect.width + rect.x > img.cols || rect.height + rect.y > img.rows)
        return 255;
    Mat roi = img(Range(rect.y, rect.y + rect.height), Range(rect.x, rect.x + rect.width) );
            imshow("roi ", roi);
    int average_intensity = static_cast<int>(mean(roi).val[0]);
    return average_intensity;
}

//拟合圆函数
int CircleFitting(vector<cv::Point2f>& m_Points, cv::Point2f& Centroid, double& dRadius)//拟合圆使用
{
    if (!m_Points.empty())
    {
        int iNum = (int)m_Points.size();
        if (iNum < 3)	return 1;

        double X1 = 0.0;
        double Y1 = 0.0;
        double X2 = 0.0;
        double Y2 = 0.0;
        double X3 = 0.0;
        double Y3 = 0.0;
        double X1Y1 = 0.0;
        double X1Y2 = 0.0;
        double X2Y1 = 0.0;
        vector<cv::Point2f>::iterator iter;
        vector<cv::Point2f>::iterator end = m_Points.end();
        for (iter = m_Points.begin(); iter != end; ++iter)
        {
            X1 = X1 + (*iter).x;
            Y1 = Y1 + (*iter).y;
            X2 = X2 + (*iter).x * (*iter).x;
            Y2 = Y2 + (*iter).y * (*iter).y;
            X3 = X3 + (*iter).x * (*iter).x * (*iter).x;
            Y3 = Y3 + (*iter).y * (*iter).y * (*iter).y;
            X1Y1 = X1Y1 + (*iter).x * (*iter).y;
            X1Y2 = X1Y2 + (*iter).x * (*iter).y * (*iter).y;
            X2Y1 = X2Y1 + (*iter).x * (*iter).x * (*iter).y;
        }
        double C = 0.0;
        double D = 0.0;
        double E = 0.0;
        double G = 0.0;
        double H = 0.0;
        double a = 0.0;
        double b = 0.0;
        double c = 0.0;
        C = iNum * X2 - X1 * X1;
        D = iNum * X1Y1 - X1 * Y1;
        E = iNum * X3 + iNum * X1Y2 - (X2 + Y2) * X1;
        G = iNum * Y2 - Y1 * Y1;
        H = iNum * X2Y1 + iNum * Y3 - (X2 + Y2) * Y1;
        a = (H * D - E * G) / (C * G - D * D);
        b = (H * C - E * D) / (D * D - G * C);
        c = -(a * X1 + b * Y1 + X2 + Y2) / iNum;
        double A = 0.0;
        double B = 0.0;
        double R = 0.0;
        A = a / (-2);
        B = b / (-2);
        R = double(sqrt(a * a + b * b - 4 * c) / 2);
        Centroid.x = A;
        Centroid.y = B;
        dRadius = R;
        return 0;
    }
    else
        return 1;
    return 0;
}

bool BuffDetector::DetectBuff(Mat& img, OtherParam other_param)
{

    other_param.mode=1;
    GaussianBlur(img, img, Size(7,7),0); //3*3
    // **预处理** -图像进行相应颜色的二值化
    points_2d.clear();
    vector<cv::Mat> bgr;
    split(img, bgr);
    Mat result_img;
    if(color_ != 0)
    {
        subtract(bgr[2], bgr[0], result_img);
    }else
    {
        subtract(bgr[0], bgr[2], result_img);
    }
    Mat binary_color_img;
#ifdef TEST_OTSU
    double th = threshold(result_img, binary_color_img, 50, 255, CV_THRESH_BINARY|CV_THRESH_OTSU);
    if(th-10>0)
        threshold(result_img, binary_color_img, th-10, 255, CV_THRESH_BINARY);
#endif

#ifndef TEST_OTSU
    threshold(result_img, binary_color_img, color_th_, 255, CV_THRESH_BINARY);

#endif
            Mat element = getStructuringElement(MORPH_RECT, Size(5,5));
            Mat element2=getStructuringElement(MORPH_RECT, Size(10,10));
            morphologyEx(binary_color_img,binary_color_img,MORPH_CLOSE,element);
            floodFill(binary_color_img,Point(0,0),Scalar(0));
             morphologyEx(binary_color_img,binary_color_img,MORPH_CLOSE,element2);
            //dilate(binary_color_img, binary_color_img, element2);
#ifdef DEBUG_BUFF_DETECT
    imshow("mask", binary_color_img);
#endif

#ifdef TEST_OTSU
    if(th < 20)
        return 0;
#endif
    // **寻找击打矩形目标** -通过几何关系
    // 寻找识别物体并分类到object
    vector<Object> vec_target;
    vector<Rect> vec_color_rect;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(binary_color_img,contours,hierarchy,CV_RETR_CCOMP,CHAIN_APPROX_NONE);
   // all轮廓的调试
    for(int i=0;i<contours.size();i++){
        std::cout<<contours.size()<<std::endl;
        //if(hierarchy[i][3]!=-1)//have father
        drawContours(img,contours,i,Scalar(255,0,255),1); //father
    }
    imshow("contours",img);
    //waitKey(0);
    for(size_t i=0; i < contours.size();i++)
    {

        // 用于寻找小轮廓，没有父轮廓的跳过, 以及不满足6点拟合椭圆
        if(hierarchy[i][3]<0 || contours[i].size() < 6 || contours[static_cast<uint>(hierarchy[i][3])].size() < 6)
            continue;

        // 小轮廓面积条件
        double small_rect_area = contourArea(contours[i]);
        double small_rect_length = arcLength(contours[i],true);
        if(small_rect_length < 10)
            continue;
        if(small_rect_area < 200)
            continue;
        // 用于超预测时比例扩展时矩形的判断
        Rect rect = boundingRect(contours[static_cast<uint>(hierarchy[i][3])]);
        vec_color_rect.push_back(rect);


        // 大轮廓面积条件
        double big_rect_area = contourArea(contours[static_cast<uint>(hierarchy[i][3])]);
        double big_rect_length = arcLength(contours[static_cast<uint>(hierarchy[i][3])],true);
        if(big_rect_area < 300)
            continue;
        if(big_rect_length < 50)
            continue;
        // 能量机关扇叶进行拟合
        Object object;

        //面积参数根据视角调节
//        cout<<"area"<<small_rect_area<<"  "<< big_rect_area<<endl;
        if(big_rect_area>9000)
            drawContours(img,contours,static_cast<uint>(hierarchy[i][3]),Scalar(0,255,0),2);
        else if(big_rect_area>3000)
            drawContours(img,contours,static_cast<uint>(hierarchy[i][3]),Scalar(255,255,255),2);

//        waitKey(0);

#ifdef FUSION_MINAREA_ELLIPASE

        object.small_rect_=fitEllipse(contours[i]);//当前小轮廓
        object.big_rect_ = fitEllipse(contours[static_cast<uint>(hierarchy[i][3])]);//父轮廓

#else
        object.small_rect_=minAreaRect(contours[i]);
        object.big_rect_ = minAreaRect(contours[static_cast<uint>(hierarchy[i][3])]);
#endif
#ifdef DEBUG_DRAW_CONTOURS
        Point2f small_point_tmp[4];//左上开始
        object.small_rect_.points(small_point_tmp);
        circle(img, small_point_tmp[0], 5, Scalar(255, 255, 255), 5);//调试点的顺序
        Point2f big_point_tmp[4];
        object.big_rect_.points(big_point_tmp);
        for(int k=0;k<4;k++)
        {
            line(img, small_point_tmp[k],small_point_tmp[(k+1)%4], Scalar(0, 255, 255), 2);
            line(img, big_point_tmp[k],big_point_tmp[(k+1)%4], Scalar(0, 0, 255), 2);
        }
        small_angle=object.small_rect_.angle;
        big_angle=object.big_rect_.angle;

        //调试查看宽高比,角度

//        putText(img,to_string(small_angle),Point2f(20,20)+object.small_rect_.center,FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));//小面积,白色
//        putText(img,to_string(big_angle),Point2f(20,40)+object.big_rect_.center,FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,255));//大面积,品红
//        float h_w=object.small_rect_.size.height/object.small_rect_.size.width ;
//        putText(img,to_string(object.small_rect_.size.height),Point2f(20,20),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
//        putText(img,to_string(object.small_rect_.size.width),Point2f(20,40),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
//        putText(img,to_string(h_w),Point2f(20,60),FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
//        //

// imshow("contours",img);
#endif
#ifdef FUSION_MINAREA_ELLIPASE
        object.diff_angle=fabsf(object.big_rect_.angle-object.small_rect_.angle);//理想状态为90’


        if(object.small_rect_.size.height/object.small_rect_.size.width < 3)//理想状态为2 nums ,height长轴
        {
            if(object.diff_angle<100 && object.diff_angle>80)
            {
#endif
#ifndef  FUSION_MINAREA_ELLIPASE
                float small_rect_size_ratio;
                if(object.small_rect_.size.width > object.small_rect_.size.height)
                {
                    small_rect_size_ratio = object.small_rect_.size.width/object.small_rect_.size.height;
                }else {
                    small_rect_size_ratio = object.small_rect_.size.height/object.small_rect_.size.width;
                }
#endif
#ifdef FUSION_MINAREA_ELLIPASE
                float small_rect_size_ratio;
                small_rect_size_ratio = object.small_rect_.size.height/object.small_rect_.size.width;
#endif
                // 根据轮廓面积进行判断扇叶类型
                //未激活大小轮廓面积比为2.8=3909/1372
                //已激活大小轮廓面积比为6.7=9115/1359

                double area_ratio = area_ratio_/100;//5

//                putText(img,to_string(small_rect_area),Point2f(20,20)+object.small_rect_.center,FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
//                putText(img,to_string(big_rect_area),Point2f(20,40)+object.big_rect_.center,FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,0,255));

//                //

//               imshow("contours",img);
                if(small_rect_area * 12 >big_rect_area && small_rect_area* area_ratio<big_rect_area
                        && small_rect_size_ratio > 1 && small_rect_size_ratio < 3.0f)
                {
                    object.type_ = ACTION;  // 已经激活类型
                }else if(small_rect_area * area_ratio>=big_rect_area && small_rect_area *2 < big_rect_area
                         && small_rect_size_ratio > 1 && small_rect_size_ratio < 3.0f)
                {
                    //更新世界坐标系顺序
                    object.type_ = INACTION;    // 未激活类型
//                    cout<<"before: "<<object.type_<<endl;

                    circle(img, object.big_rect_.center, 5, Scalar(255, 255, 255), 5);
                    object.UpdateOrder();
                    object.KnowYourself(binary_color_img);
                    cout<<"after: "<<object.type_<<endl;
                    vec_target.push_back(object);

                }else
                {
                    object.type_ = UNKOWN;    // 未激活类型
                }
#ifdef DEBUG_DRAW_TARGET
        object.DrawTarget(img);//画出装甲板
#endif
       float  delta_x=abs(last_point.x-object.small_rect_.center.x);
        float delta_y=abs(last_point.y-object.small_rect_.center.y);
       if(delta_x>2&&delta_y>2)
           {
        points.push_back(object.small_rect_.center);
        last_point=object.small_rect_.center;
       }




#ifdef AREA_LENGTH_ANGLE
                switch (AREA_LENGTH_ANGLE)
                {
                case 1:
                {
                    double multiple_area=fabs(big_rect_area/small_rect_area);
                    putText(img, to_string(multiple_area), Point2f(50,50)+ object.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));
                }break;
                case 2:
                {
//                    drawContours(img,contours,i,Scalar(0,255,255),5);
                    double multiple_length=fabs(big_rect_length/small_rect_length);
                    putText(img, to_string(multiple_length), Point2f(50,50)+ object.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));
                }break;
                case 3:
                {
                    putText(img, to_string(object.diff_angle), Point2f(-20,-20)+ object.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));
                }break;
                }
#endif
#ifdef FUSION_MINAREA_ELLIPASE
            }
        }
#endif
    }
     CircleFitting(points,cente,radium);
     circle(img,cente,radium,Scalar(0,0,255),2);
     circle(img,cente,radium,Scalar(255,0,0),-1);



    // 遍历所有结果并处理\选择需要击打的目标
    Object final_target; //击打目标的临时变量
    bool find_flag = false;
    float diff_angle = 1e8;
    // 你需要击打的能量机关类型 1(true)击打未激活 0(false)击打激活
    for(size_t i=0; i < vec_target.size(); i++)
    {
        Object object_tmp = vec_target.at(i);
        // 普通模式击打未激活机关
        if(object_tmp.type_ == INACTION){

            find_flag = true;
            float ang = fabs(vec_target[i].diff_angle-90.0f);
            if(ang < diff_angle)
            {
                final_target = vec_target.at(i);
                diff_angle = ang;
            }
//            putText(img, "final_target", Point2f(10,-50)+ final_target.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));

            Point2f buff_offset = Point2f(buff_offset_x_ - 100, 100 - buff_offset_y_);
            vector<Point2f> vec_points_2d_tmp;
            for(size_t k=0; k < 4; k++)
            {
                vec_points_2d_tmp.push_back(final_target.points_2d_.at(k) + buff_offset);
            }
            points_2d = vec_points_2d_tmp;
            buff_angle_ = final_target.angle_;


            putText(img, to_string(buff_angle_), Point2f(10,50)+ final_target.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));

#ifdef DEBUG_PUT_TEST_ANGLE
            for(size_t j = 0; j < 4; j++)
            {
//                putText(img, to_string(j), Point2f(5,5)+ final_target.points_2d_[j], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));//未经过偏移补偿的
                putText(img, to_string(j), Point2f(5,5)+ points_2d[j], FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0,255,255));//经过偏移补偿的

            }
#endif
        }
    }
    if(find_flag){


#ifdef DEBUG_PUT_TEST_TARGET
        putText(img, "<<---attack here"/*to_string(object_tmp.angle_)*/, Point2f(5,5)+ final_target.small_rect_.center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255,255,255));
#endif
#ifdef DEBUG_DRAW_TARGET
//        final_target.DrawTarget(img);//画出装甲板
        float world_offset_x = 750 - 500;
        float world_offset_y = 800 - pow((640000 - pow(world_offset_x, 2)), 0.5);
        Point2f world_offset = Point2f(world_offset_x, world_offset_y);
        float pre_angle = atan(world_offset_x/(800-world_offset_y));
        cout<<"pre_angle"<<pre_angle<<endl;
        circle(img, Point2f(final_target.small_rect_.center.x+cosf(pre_angle),final_target.small_rect_.center.y+sinf(pre_angle)), 3, Scalar(0, 0, 255), -1);
#endif

    }
    return find_flag;
}
//
int BuffDetector::getSimpleDirection(float angle)
{
    diff_angle_ = angle - last_angle_;
    last_angle_ = angle;

    if(fabs(diff_angle_) < 10 && fabs(diff_angle_) > 1e-6)
    {
        d_angle_ = (1 - r) * d_angle_ + r * diff_angle_;
        cout<<"d_angle_"<<d_angle_<<endl;
    }

    if(d_angle_ > 4)
        return 1;
    else if(d_angle_ < -4)
        return -1;
    else
        return 0;
}

void Object::KnowYourself(Mat &img)
{
    Point2f vector_height = points_2d_.at(0) - points_2d_.at(3);
    //装甲板宽度中心点
    //    vector_height = Point2f(vector_height.x * 0.5 , vector_height.y * 0.5);
    Point left_center = points_2d_.at(3) - vector_height;
    Point right_center = points_2d_.at(2) - vector_height;
            circle(img, left_center, 3, Scalar(255), -1);
            circle(img, right_center, 3, Scalar(255), 1);

    int width = 10;
    int height = 10;

    Point left1 = Point(left_center.x - width, left_center.y - height);
    Point left2 = Point(left_center.x + width, left_center.y + height);

    Point right1 = Point(right_center.x - width, right_center.y - height);
    Point right2 = Point(right_center.x + width, right_center.y + height);

    Rect left_rect(left1, left2);
    Rect right_rect(right1, right2);

        rectangle(img, left_rect, Scalar(255), 1);
        rectangle(img, right_rect, Scalar(255), 1);

    int left_intensity = GetRectIntensity(img, left_rect);
    int right_intensity = GetRectIntensity(img, right_rect);
    if(left_intensity > 100 && right_intensity > 100)
    {
        type_ = ACTION;
    }else{
        type_ = INACTION;
    }
    putText(img, to_string(left_intensity), left_center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255));
    putText(img, to_string(right_intensity), right_center, FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255));
#ifdef IMSHOW_2_ROI
    //namedWindow("roi_intensity");
    //imshow("roi_intensity", img);
#endif
}

void Object::UpdateOrder()
{
    points_2d_.clear();
#ifdef FUSION_MINAREA_ELLIPASE
    Point2f points[4];
    small_rect_.points(points);
    Point2f point_up_center = (points[0] + points[1])/2;
    Point2f point_down_center = (points[2] + points[3])/2;
    double up_distance = Point_distance(point_up_center, big_rect_.center);
    double down_distance = Point_distance(point_down_center, big_rect_.center);
    if(up_distance > down_distance)
    {
        angle_ = small_rect_.angle;
        points_2d_.push_back(points[0]);points_2d_.push_back(points[1]);
        points_2d_.push_back(points[2]);points_2d_.push_back(points[3]);
    }else
    {
        angle_ = small_rect_.angle + 180;
        points_2d_.push_back(points[2]);points_2d_.push_back(points[3]);
        points_2d_.push_back(points[0]);points_2d_.push_back(points[1]);
    }

#endif

}

int BuffDetector::Get_Pre_Rect(Mat &img, RotatedRect small_rect,Point2f rect_center){
    RotatedRect pre_rect;
    if(direction_tmp==1)
        pre_rect = RotatedRect(rect_center,small_rect.size,small_rect.angle+30);
    else if(direction_tmp==-1)
        pre_rect = RotatedRect(rect_center,small_rect.size,small_rect.angle-30);
    else if(direction_tmp==0)
        pre_rect = RotatedRect(small_rect.center,small_rect.size,small_rect.angle);
    if(direction_tmp!=0)
    {Point2f vertex[4];
    pre_rect.points(vertex);

    for (int i = 0; i < 4; i++)
    {
        cv::line(img, vertex[i], vertex[(i + 1) % 4], cv::Scalar(255, 100, 200),2,CV_AA);
    }
    pre_points_2d.clear();

    Point2f points[4];
    pre_rect.points(points);
    Point2f point_up_center = (points[0] + points[1])/2;
    Point2f point_down_center = (points[2] + points[3])/2;
    double up_distance = Point_distance(point_up_center, cente);
    double down_distance = Point_distance(point_down_center,cente);
    if(up_distance > down_distance)
    {

        pre_points_2d.push_back(points[0]);pre_points_2d.push_back(points[1]);
        pre_points_2d.push_back(points[2]);pre_points_2d.push_back(points[3]);
    }else
    {
        pre_points_2d.push_back(points[2]);pre_points_2d.push_back(points[3]);
        pre_points_2d.push_back(points[0]);pre_points_2d.push_back(points[1]);
    }
    }
    else
        //target.points_2d_.points(pre_points_2d);
       pre_points_2d=target.points_2d_;
}


int BuffDetector::PreTask(Mat& img){
        //大符预测角度 0.785*sin(1.844(t+time))+1.305*time积分
       // double distance=7000;//毫米
        double velocity=15;//米每秒
        double time;//
        float t;
        time=distance_/velocity;
        float pre_ange;
        float mood=0;
        float pre_rad;
        Point2d pre_point;
        if(direction_tmp==1){
            pre_point= Point2d(cente.x-radium*cos(target.small_rect_.angle/180*M_PI-M_PI/6),cente.y-radium*sin(target.small_rect_.angle/180*M_PI+M_PI/6));
            circle(img,pre_point, 3, Scalar(0, 255, 255), 5);}
        if(direction_tmp==-1){
            pre_point=Point2d(cente.x-radium*cos(target.small_rect_.angle/180*M_PI-M_PI/6),cente.y-radium*sin(target.small_rect_.angle/180*M_PI-M_PI/6));
            circle(img,pre_point, 3, Scalar(0, 255, 255), 5);}
        if(direction_tmp==0){
            pre_point=Point2d(target.small_rect_.center.x,target.small_rect_.center.y);
            circle(img,pre_point, 3, Scalar(0, 255, 255), 5); }

        Get_Pre_Rect(img,target.small_rect_,pre_point);


        if(mood==1)// 大符
              { pre_rad=(-0.875/1.844)*cos(1.844*(t+time))+1.305*time;
                pre_ange=180*pre_rad/CV_PI;
                if(direction_tmp == 1) { // shun
                    Point2d pre_point=Point2d(cente.x-radium*cos(target.small_rect_.angle/180*CV_PI+pre_rad),cente.y-radium*sin(target.small_rect_.angle/180*CV_PI+pre_rad));
                    circle(img,pre_point, 3, Scalar(0, 255, 255), 5);
                    }
                else if(direction_tmp == -1){// ni
                    Point2d pre_point=Point2d(cente.x-radium*cos(target.small_rect_.angle/180*CV_PI-pre_rad),cente.y-radium*sin(target.small_rect_.angle/180*CV_PI-pre_rad));
                    circle(img,pre_point, 3, Scalar(0, 255, 255), 5);
                    }
                else if(direction_tmp==0){
                    pre_point=Point2d(target.small_rect_.center.x,target.small_rect_.center.y);
                    circle(img,pre_point, 3, Scalar(0, 255, 255), 5); }
        }
        else if (mood==2)//小符
              { pre_ange=60;
                if(direction_tmp == 1) { // shun
                    Point2d pre_point=Point2d(cente.x-radium*cos(target.small_rect_.angle/180*CV_PI+CV_PI/6),cente.y-radium*sin(target.small_rect_.angle/180*CV_PI+CV_PI/6));
                    circle(img,pre_point, 3, Scalar(0, 255, 255), 5);
                    }
                else if(direction_tmp == -1){// ni
                    Point2d pre_point=Point2d(cente.x-radium*cos(target.small_rect_.angle/180*CV_PI-CV_PI/6),cente.y-radium*sin(target.small_rect_.angle/180*CV_PI-CV_PI/6));
                    circle(img,pre_point, 3, Scalar(0, 255, 255), 5);
                    }
                else  if(direction_tmp==0){
                    pre_point=Point2d(target.small_rect_.center.x,target.small_rect_.center.y);
                    circle(img,pre_point, 3, Scalar(0, 255, 255), 5); }
        }

}


int BuffDetector::BuffDetectTask(Mat& img, OtherParam other_param)
{
    color_ = other_param.color;

    bool find_flag = DetectBuff(img,other_param);
    int command = 0;
    if(find_flag)
    {

        if(find_cnt % 3==0)
        {
//            direction_tmp = getDirection(buff_angle_);
              direction_tmp = getSimpleDirection(buff_angle_);
//            waitKey(0);
        }
        PreTask(img);
        find_cnt ++;
        Point2f world_offset;


#ifdef DIRECTION_FILTER
        float world_offset_x = 750 - 500;
        float world_offset_y = 800 - pow((640000 - pow(world_offset_x, 2)),0.5);
        float pre_angle;
        if(direction_tmp == 1)  // shun
        {
            world_offset = Point2f(world_offset_x, world_offset_y);
            pre_angle = atan(world_offset_x/(800-world_offset_y));
        }
        else if(direction_tmp == -1)//ni
        {
            world_offset = Point2f(-world_offset_x, world_offset_y);
            pre_angle = -atan(world_offset_x/(800-world_offset_y));
        }
        else
        {
            world_offset = Point2f(0, 0);
            pre_angle = 0;
        }
                cout << "direction " << direction_tmp << endl;
                cout << "pre_angle " << pre_angle << endl;
#else
        world_offset = Point2f(world_offset_x_ - 500, world_offset_y_  - 500);
#endif
        solve_angle_long_.Generate3DPoints(2,world_offset);
        solve_angle_long_.getBuffAngle(1,pre_points_2d, BULLET_SPEED
                                       , buff_angle_, 0, gimbal
                                       , angle_x_, angle_y_, distance_);
        command=1;
    }
    return command;
}



