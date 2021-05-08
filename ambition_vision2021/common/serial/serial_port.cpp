#include "serial_port.h"

#include <iostream>
#include <fcntl.h>      // File Control Definitions
#include <termios.h>    // PPSIX Terminal Control Definitions
#include <unistd.h>

SerialPort::SerialPort(){}
SerialPort::SerialPort(int &fd,const char* filename, int buadrate)
{
    file_name_ = filename;
    buadrate_ = buadrate;
    success_ = false;
//    serial_mode = NO_INIT;
    //O_NONBLOCK,非阻塞模式
    fd = open(file_name_, O_RDWR | O_NONBLOCK | O_NOCTTY);// Read/Write access to serial port                                           // No terminal will control the process
    local_fd = fd;
    last_fd = fd;
    if(fd ==-1 )
    {
        perror(file_name_);
        printf("open_port wait to open %s .\n", file_name_);
        return;
    }
//    else
//    {
//        fcntl(fd, F_SETFL, 0);
//    }
    printf("Open...\n");
    set_speed(fd,buadrate_);
    if (set_Parity(fd,8,1,'N') == 0)  {
        printf("Set Parity Error\n");
        exit (0);
    }
    set_disp_mode(fd,0);
    printf("Open successed\n");

}

int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
    B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,
    115200, 38400, 19200, 9600, 4800, 2400, 1200,  300, };
void SerialPort::set_speed(int fd, int speed){
    int   i;
    int   status;
    struct termios   Opt;// structure to store the port settings in
    /*tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，该串口是否可用等。
         * 若调用成功，函数返回值为0，若调用失败，函数返回值为1. */
    tcgetattr(fd, &Opt);
    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) {
        if  (speed == name_arr[i]) {
            tcflush(fd, TCIOFLUSH);//刷新输入输出缓冲区。
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
//            options.c_cflag |= (CLOCAL | CREAD);//本地模式(CLOCAL)和串行数据接收(CREAD)设置为有效
                                                    //这是立刻把bote rates设置真正写到串口中去
            status = tcsetattr(fd, TCSANOW, &Opt);//不等数据传输完毕就立即改变属性
            if  (status != 0) {
                perror("tcsetattr fd1");
                return;
            }
            tcflush(fd,TCIOFLUSH);
        }
    }
}

/**
*@brief   设置串口数据位,停止位和效验位
*@param  fd     类型  int  打开的串口文件句柄
*@param  databits 类型  int 数据位   取值 为 7 或者8
*@param  stopbits 类型  int 停止位   取值为 1 或者2
*@param  parity  类型  int  效验类型 取值为N,E,O,,S
*/
int SerialPort::set_Parity(int fd, int databits, int stopbits, int parity)
{
    struct termios options;
    if  ( tcgetattr( fd,&options)  !=  0) {
        perror("SetupSerial 1");
        return 0;
    }
    options.c_cflag &= ~CSIZE;
    switch (databits) /*设置数据位数*/
    {
        case 7:
            options.c_cflag |= CS7;
            break;
        case 8:
            options.c_cflag |= CS8;
            break;
        default:
            fprintf(stderr,"Unsupported data size\n"); return 0;
    }
    switch (parity)
    {
        case 'n':
        case 'N':
            options.c_cflag &= ~PARENB;   /* Clear parity enable */
            options.c_iflag &= ~INPCK;     /* Enable parity checking */
            break;
        case 'o':
        case 'O':
            options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/
            options.c_iflag |= INPCK;             /* Disnable parity checking */
            break;
        case 'e':
        case 'E':
            options.c_cflag |= PARENB;     /* Enable parity */
            options.c_cflag &= ~PARODD;   /* 转换为偶效验*/
            options.c_iflag |= INPCK;       /* Disnable parity checking */
            break;
        case 'S':
        case 's':  /*as no parity*/
            options.c_cflag &= ~PARENB;
            options.c_cflag &= ~CSTOPB;break;
        default:
            fprintf(stderr,"Unsupported parity\n");
            return 0;
    }
    /* 设置停止位*/
    switch (stopbits)
    {
        case 1:
            options.c_cflag &= ~CSTOPB;
            break;
        case 2:
            options.c_cflag |= CSTOPB;
            break;
        default:
            fprintf(stderr,"Unsupported stop bits\n");
            return 0;
    }
    /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;
    tcflush(fd,TCIFLUSH);
    options.c_cc[VTIME] = 150; /* 设置超时15 seconds*/
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("SetupSerial 3");
        return 0;
    }
    //BRKINT Send a SIGINT when a break condition is detected当在输入行中检测到一个终止状态时，产生一个中断
    //ISTRIP 去除字符的第8个比特,移除奇偶位
    //IXON 允许输入时对XON/XOFF流进行控制
    //ICRNL 将输入的NL（换行）转换成CR（回车）
    //IGNCR 忽略输入的回车
    //IXON 对输出启用软件流控
    options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*///忽略回车
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
//    options.c_lflag &= ~(ICANON);
//    options.c_lflag &= ~(ICANON | ISIG);
    options.c_iflag &= ~(ICRNL | IGNCR);
//    options.c_lflag &= ~(ECHO | ECHOE);
    options.c_oflag  &= ~OPOST;   /*Output*/
    if (tcsetattr(fd,TCSAFLUSH,&options) != 0)
    {
        perror("SetupSerial 3");
        return 0;
    }
    return 1;
}

int SerialPort::set_disp_mode(int fd,int option)
{
   int err;
   struct termios term;
   if(tcgetattr(fd,&term)==-1){
     perror("Cannot get the attribution of the terminal");
     return 1;
   }
   if(option)
        term.c_lflag|=ECHOFLAGS;//终端回显
   else
        term.c_lflag &=~ECHOFLAGS;//
   err=tcsetattr(fd,TCSAFLUSH,&term);//err用于保存函数调用后的结果
   if(err==-1 && err==EINTR){
        perror("Cannot set the attribution of the terminal");
        return 1;
   }
   return 0;
}

// pc -> stm32
void SerialPort::send_data(int &fd,const VisionData &data)
{
    Tdata[0] = 0xA5;
    Tdata[1] = data.mode;

    Tdata[2] = data.yaw_angle.c[0];
    Tdata[3] = data.yaw_angle.c[1];
    Tdata[4] = data.yaw_angle.c[2];
    Tdata[5] = data.yaw_angle.c[3];

    Tdata[6] = data.pitch_angle.c[0];
    Tdata[7] = data.pitch_angle.c[1];
    Tdata[8] = data.pitch_angle.c[2];
    Tdata[9] = data.pitch_angle.c[3];

    Tdata[10] =data.distance.c[0];
    Tdata[11] =data.distance.c[1];
    Tdata[12] =data.distance.c[2];
    Tdata[13] =data.distance.c[3];

    Tdata[14] = data.command;
    Tdata[15]=0XEE;
    printf("send x:%f y:%f z:%f\r\n",data.yaw_angle.f,data.pitch_angle.f,data.distance.f);

    if(write(fd, Tdata,18) !=18)
    {
        printf("发送字节错误\n");
        {
//            extern VisionData  tx_data;
//            tx_data.mode='6';
//            tx_data.pitch_angle.d=6;//测试接受的
//            tx_data.yaw_angle.d = 6;
//            tx_data.distance.f = 6;
//            tx_data.command = 6;

        }
       restart_serial();
    }
}

//bool SerialPort::read_data(const struct serial_receive_data *data, bool &mode, bool &my_car_color
//                           , int &buff_offset_x, int &buff_offset_y)
bool SerialPort::read_data(void)
{

    int nread;
    nread = read(local_fd, buff, 20);

        for(int i=0;i<20;i++)
        {
          rear=(rear+1)%128;
          deal_buff[rear]=buff[i];
          flag++;
        }
//    int nread;
//    char buff[18];
//    bool check_recive;
//    nread = read(local_fd, buff, sizeof(buff));
//    if(buff[0]==0xA5 && buff[2]==0XEE){
//       buff[1]=mood;
//       check_recive=true;}
//    else{
//       mood='o';
//       check_recive=false;}
//    if(check_recive){
//    printf("read %d bytes already.\n", nread);
//    printf("收到的数字为 %c\n",mood);
//    return 1;}
//    else {
//        printf("无接受数据");
//        return 0;
//    }
}
int SerialPort::deal_date(OtherParam otherparam)
{

    if(flag<20)
        return 0;
    int x=0;
    front=(front+1)%128;
     //flag--;
    while((unsigned char) deal_buff[front]!=0xA5||(unsigned char)deal_buff[front+19]!=0xEE)
    {
        front=(front+1)%128;
         x++;
       // flag--;
        if(x>19)
            return 0;
    }
//    for(int j=front;j<front+10;j++)
//       printf("%x\n",(unsigned char) deal_buff[j]);
//    if((unsigned char) deal_buff[front+1]==0x66)
//        printf("%c\n",(unsigned char) deal_buff[front+1]);
    if((unsigned char) deal_buff[front+1]==0x66)
        {printf("%c\n",(unsigned char) deal_buff[front+1]);
       mood='f';
       otherparam.mode=1;}

    if((unsigned char) deal_buff[front+1]==0x6d)
       { printf("%c\n",(unsigned char) deal_buff[front+1]);
       mood='m';
       otherparam.mode=0;}

       nowtime.c[0]=deal_buff[front+2];
       nowtime.c[1]=deal_buff[front+3];
       nowtime.c[2]=deal_buff[front+4];
       nowtime.c[3]=deal_buff[front+5];
       otherparam.time=nowtime.f;

       bullet_speed.c[0]=deal_buff[front+6];
       bullet_speed.c[1]=deal_buff[front+7];
       bullet_speed.c[2]=deal_buff[front+8];
       bullet_speed.c[3]=deal_buff[front+9];
       otherparam.bullet_speed=bullet_speed.f;




    front=(front+20)%128;
     printf("rx_read\n");

}



void SerialPort::getVisionData(unsigned char (&data)[18],VisionData& enemy_position)//*data data[] data[18]
{
    //if(data[0]==0xA5)
    if(data[0]==0xA5&&data[11]==0xEE)
    {
       enemy_position.mode=data[1];
       enemy_position.yaw_angle.c[0]=data[2];
       enemy_position.yaw_angle.c[1]=data[3];

       enemy_position.pitch_angle.c[0]=data[4];
       enemy_position.pitch_angle.c[1]=data[5];

       enemy_position.distance.c[0]=data[6];
       enemy_position.distance.c[1]=data[7];
       enemy_position.distance.c[2]=data[8];
       enemy_position.distance.c[3]=data[9];

       enemy_position.command=data[10];
    }
}

void SerialPort::restart_serial(void)
{
    close(local_fd);
    local_fd = open(file_name_, O_RDWR | O_NONBLOCK | O_NOCTTY);
    if(local_fd == -1 )
        return;
    last_fd = local_fd;
    printf("Again Open...\n");
    set_speed(local_fd,buadrate_);
    if (set_Parity(local_fd,8,1,'N') == 0)  {
        printf("Set Parity Error\n");
        exit (0);
    }
    set_disp_mode(local_fd,0);
    printf("Again Open successed\n");

}
