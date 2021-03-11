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

    //File name get & set
    bool setFileName(std::string);
    std::string getFileName();

    //get file type
    std::string getFileType();

    //Record videos get & set
    bool setRecordVideos(bool);
    bool getRecordVideos();

    //Display videos get & set
    bool setDisplayVideos(bool);
    bool getDisplayVideos();

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
    int fps = 30;

    std::string fileName = "video";
    std::string fileType = ".mp4";
    bool recordVideos = false;
    bool displayVideos = false;
};

#endif // MAINSETTINGS_H
