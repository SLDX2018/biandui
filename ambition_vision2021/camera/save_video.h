#ifndef SAVE_VIDEO_H
#define SAVE_VIDEO_H

#include <opencv2/opencv.hpp>

using namespace cv;
/**
 * @brief 保存视频类
 */
class SaveVideo
{
public:
    SaveVideo();
    ~SaveVideo();
    void updateImage(Mat img);
    bool getState(){
        return state_;
    }
private:
    VideoWriter video_writer_;
    bool state_ = false;
};


#endif // SAVE_VIDEO_H
