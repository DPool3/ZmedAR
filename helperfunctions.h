#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include <opencv2/opencv.hpp>
#include <sys/types.h>
#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <ctime>

#include "mainsettings.h"
#include "errordialog.h"
#include "filesystem.h"

class HelperFunctions
{
public:
    HelperFunctions();

    //Create Methods
    void createDirectory(std::string);
    void createStereoDirectory(std::string);
    int createVideoWriter(cv::VideoWriter&, cv::VideoWriter&);

    //Get Methods
    std::string getVideoSavePath();
    void getCompleteVideoSavePath(std::string&, std::string&);
    std::string getCurrentSelectedImageSetPath();
    std::string getCurrentDateAsString();
    int getFpsFromMainSettings();
    int getSaveFpsFromMainSettings();

    //Set Methods
    void setCurrentSelectedImageSetPath(std::string);

    //Check Methods
    void checkVideoCaptureOpened(cv::VideoCapture);
    void checkFrameEmpty(cv::Mat);

    //Camera Mathods
    bool testCamera(int);
    void setVideoCapture(cv::VideoCapture&, cv::VideoCapture&);
    void switchCameras(cv::VideoCapture&, cv::VideoCapture&);

    //Call Dialog
    void callErrorDialog(std::string);
    QString getPathFromFileSystem();

private:

};

#endif // HELPERFUNCTIONS_H
