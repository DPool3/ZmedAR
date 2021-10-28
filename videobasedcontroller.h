#ifndef VIDEOBASEDCONTROLLER_H
#define VIDEOBASEDCONTROLLER_H

#include <iostream>
#include <QImage>
#include <opencv4/opencv2/opencv.hpp>

#include "imageprocessor.h"
#include "videomanager.h"
#include "dialogmanager.h"

/**
 * @brief Die VideoBasedController class ermöglicht die Verarbeitung von zuvor
 * aufgezeichneten Stereovideosequenzen. Auf diese Weise kann das Kameratracking,
 * ohne berücksichtigung von Echtzeitfähigkeit Bild für Bild durchgeführt werden.
 */
class VideoBasedController
{
public:
    VideoBasedController();
    bool getProcessedImages(QImage&, QImage&);
    void reinitVideoBasedController(int, int, int);
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

    void stop();
};

#endif // VIDEOBASEDCONTROLLER_H
