#pragma once
#ifndef MAINSETTINGS_H
#define MAINSETTINGS_H

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>


/**
 * @brief Die MainSettings class enthält alle wichtigen Pfade, Dateitypen, frame Zahlen und
 * Bildformate, die für die Durchführung wichtig sind. Alle Werte können nur durch Getter und
 * Setter abgefragt und geändert werden.
 */
class MainSettings
{
public:
    //------------------------------------------------------------------
    //Methods
    //------------------------------------------------------------------
    MainSettings();

    //getter
    std::string getImageSetSelectionPath();
    std::string getImageSetsPath();
    std::string getTrackingFilePath();
    std::string getVideosPath();
    std::string getRootPath();
    std::string getVideoFileType();
    std::string getImageFileType();
    std::string getVideoFileName();
    cv::Size getFrameSize();
    int getFps();
    int getSaveFps();

    //setter
    int setImageSetSelectionPath(std::string);
    int setImageSetsPath(std::string);
    int setTrackingFilesPath(std::string);
    int setVideosPath(std::string);
    int setRootPath(std::string);
    bool setVideoFileName(std::string);
    void setFps(int);
    void setSaveFps(int);

private:
    //------------------------------------------------------------------
    //Variables
    //------------------------------------------------------------------

    //generell
    std::string imageSetSelectionPath = "";
    std::string imageSetsPath = "/home/daniel/ZAR/ImageSets";
    std::string videosPath = "/home/daniel/ZAR/Videos";
    std::string trackingPath = "/home/daniel/ZAR/trackingFiles";
    std::string rootPath = "/home/daniel/ZAR";
    cv::Size framesize = cv::Size(1920,1200);
    int fps = 60;
    int saveFps = 30;

    std::string videoFileName = "video";
    std::string videoFileType = ".mp4";
    std::string imageFileType = ".png";
};

#endif // MAINSETTINGS_H
