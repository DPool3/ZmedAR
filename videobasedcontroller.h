#ifndef VIDEOBASEDCONTROLLER_H
#define VIDEOBASEDCONTROLLER_H

#include <iostream>
#include <QImage>
#include <opencv2/opencv.hpp>

#include "imageprocessor.h"
#include "videomanager.h"
#include "dialogmanager.h"

class VideoBasedController
{
public:
    VideoBasedController();
    bool getProcessedImages(QImage&, QImage&);
    void reinitVideoBasedController();
    int getfps();

    bool useCameraTracking = false;
    std::string videoPathRight = "";
    std::string videoPathLeft = "";

private:
    ImageProcessor imageProcessor;
    VideoManager videoManager;
    DialogManager dialogManager;

    cv::Mat imageLeft;
    cv::Mat imageRight;
    cv::VideoCapture captureLeft;
    cv::VideoCapture captureRight;

    int playFps = 0;

    void getVideoFrames();
    void stop();
};

#endif // VIDEOBASEDCONTROLLER_H
