#ifndef ANAGLYPH3DCONTROLLER_H
#define ANAGLYPH3DCONTROLLER_H

#include <QString>

#include "dialogmanager.h"
#include "videomanager.h"
#include "imageprocessor.h"

class Anaglyph3DController
{
public:
    //public methods
    Anaglyph3DController();
    QString searchVideoPath();
    bool getAnaglyphImage(QImage&);
    void reinitAnaglyph3DController();
    int getPlayFps();

    //public variables
    std::string leftVideoPath, rightVideoPath;

private:
    //private methdos
    void stopAnaglyph3DController();

    //private variables
    DialogManager dialogManager;
    VideoManager videoManager;
    ImageProcessor imagePorcessor;

    cv::VideoCapture captureLeft, captureRight;
    cv::Mat imageLeft, imageRight;

    int playFps;
};

#endif // ANAGLYPH3DCONTROLLER_H
