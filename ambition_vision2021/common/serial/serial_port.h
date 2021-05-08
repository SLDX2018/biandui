#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H
#include <iostream>
#include "../../base/base.h"

typedef union{
    float f;
    unsigned char c[4];
}float2uchar;

typedef union{
    int d;
    unsigned char c[2];
}int16uchar;

typedef struct{
    unsigned char mode; // 0 auto 2 buff 1
//    int16uchar yaw_angle;
//    int16uchar pitch_angle;
    float2uchar yaw_angle;//32b
    float2uchar pitch_angle;//32
    float2uchar distance;//32
    unsigned char command;//1*8
    //command explain
    //mode 0: 0 no enemy       1 find enemy
    //mode 1: 0 no target      1 follow buff   2  Bullet launch   3 reset
}VisionData;

//typedef  struct{
//    unsigned char mode;
//    unsigned int  doodd;
//}Getdate;


struct serial_transmit_data
{
    unsigned char raw_data[10];
    int size;
    unsigned char head = 0xA5;
    unsigned char end = 0xEE;
    void get_xy_data(int16_t x, int16_t y, int found);
};


class SerialPort
{
public:
    char buff[20];
    char deal_buff[128];
    int flag;
    int rear;
    int front;
    #define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)
    //"/dev/ttyTHS0"
    SerialPort();
    /**
     * @brief SerialPort
     * @param filename 串口名字
     * @param buadrate 串口波特率,用于stm32通信是0-B115200,用于stm32通信是1-B921600
     */
    SerialPort(int &fd,const char* filename, int buadrate);

    void set_speed(int fd, int speed);
    int set_Parity(int fd,int databits,int stopbits,int parity);
    int set_disp_mode(int fd,int option);
    void restart_serial(void);  // 尝试重连的函数
    void send_data(int &fd,const VisionData &data);
    int deal_date(OtherParam otherparam);//接受数据处理
//    bool read_data(const struct serial_receive_data *data, bool &mode, bool &my_car_color
//                   , int &buff_offset_x, int &buff_offset_y);
    bool read_data(void);
    void getVisionData(unsigned char (&data)[18],VisionData& enemy_position);
    bool read_gimbal(const struct serial_gimbal_data* data, float &gimbal_yaw);
    int local_fd;
    int last_fd;
    float2uchar nowtime;
    float2uchar bullet_speed;

    bool success_;
    unsigned char Tdata[30];
    char mood= '0';
private:


    const char* file_name_;
    int buadrate_;
    float last_bullet_speed;
};




#endif // SERIAL_PORT_H


