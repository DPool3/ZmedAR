#ifndef VIDEORECORD_H
#define VIDEORECORD_H

#include <opencv2/opencv.hpp>

class VideoRecord
{
public:
    VideoRecord();
    cv::VideoCapture createVideoCapture(int );
    void checkVideoCaptureOpened(cv::VideoCapture);

};

#endif // VIDEORECORD_H
