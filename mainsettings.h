#pragma once
#ifndef MAINSETTINGS_H
#define MAINSETTINGS_H

#include <iostream>
#include <opencv2/opencv.hpp>

class MainSettings
{
public:
    //------------------------------------------------------------------
    //Methods
    //------------------------------------------------------------------
    MainSettings();

    //generel
    std::string getImageSetSelectionPath();
    std::string getImageSetsPath();
    std::string getVideosPath();
    std::string getRootPath();

    int setImageSetSelectionPath(std::string);
    int setImageSetsPath(std::string);
    int setVideosPath(std::string);
    int setRootPath(std::string);

    //frames per second
    void setFps(int);
    int getFps();
    void setSaveFps(int);
    int getSaveFps();

    //File name get & set
    std::string getVideoFileName();
    bool setVideoFileName(std::string);

    //get file type
    std::string getVideoFileType();
    std::string getImageFileType();

    //Framesize
    cv::Size getFrameSize();

private:
    //------------------------------------------------------------------
    //Variables
    //------------------------------------------------------------------

    //generell
    std::string imageSetSelectionPath = "";
    std::string imageSetsPath = "/home/daniel/ZAR/ImageSets";
    std::string videosPath = "/home/daniel/ZAR/Videos";
    std::string rootPath = "/home/daniel/ZAR";
    cv::Size framesize = cv::Size(1920,1200);
    int fps = 75;
    int saveFps = 30;

    std::string videoFileName = "video";
    std::string videoFileType = ".mp4";
    std::string imageFileType = ".png";
};

#endif // MAINSETTINGS_H
