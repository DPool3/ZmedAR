#ifndef VIDEOMANAGER_H
#define VIDEOMANAGER_H

#include <opencv4/opencv2/opencv.hpp>
#include <thread>
#include <future>

#include "mainsettings.h"
#include "directorymanager.h"

/**
 * @brief Die VideoManager class ermöglicht bei der kamerabasierten Verarbeitung,
 * das Speichern der Videos mit 30fps oder auch das abspielen von Videos.
 */
class VideoManager
{
public:
    VideoManager();

    //VideoWirter
    void createVideoWriterPair(cv::VideoWriter & writerLeft, cv::VideoWriter & writerRight, std::string directoryNameAddition, int fps);
    void releaseVideoWriterPair(cv::VideoWriter & writerLeft, cv::VideoWriter & writerRight);

    //VideoCapture
    void createVideoCapturePair(cv::VideoCapture & captureLeft, cv::VideoCapture & captureRight, std::string pathLeft, std::string pathRight);
    void releaseVideoCapturePair(cv::VideoCapture & captureLeft, cv::VideoCapture & captureRight);

    void saveImages(cv::Mat imgLeft, cv::Mat imgRight, cv::VideoWriter videoWriterLeft, cv::VideoWriter videoWriterRight);
    void saveImage(cv::Mat img, cv::VideoWriter writer);

    //video file type
    std::string fileType;
    //video file name
    std::string fileName;
    //save path for videos
    std::string videoSavePath;
    //fps of saved videos
    int saveFps;
    //fps of video
    int fps;
    //frame size
    cv::Size frameSize;

private:

    void createVideoWriter(cv::VideoWriter & writer, std::string path, int fps);
    void releaseVideoWriter(cv::VideoWriter & writer);

    void releaseVideoCapture(cv::VideoCapture & capture);

};

#endif // VIDEOMANAGER_H
