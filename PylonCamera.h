#ifndef IMAGEGRABBER_H
#define IMAGEGRABBER_H

#include <QElapsedTimer>
#include <opencv4/opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>
#include <thread>

using namespace Pylon;
using namespace std;

/**
 * @brief In der pylonCamera class werden die Kameras initialisiert,
 * alle Kameraeinstellungen angepasst, Bilder ausgelesen und in cv::Mat Objekte konvertiert.
 */
class pylonCamera
{
public:
    pylonCamera();

    void initCameras();
    void startGrabbing();
    void stopGrabbing();
    bool grabImages(cv::Mat&, cv::Mat&);

//    void setCreateVideo(bool);
//    int initVideoWriter();
//    void closeVideoWriter();

    double getAverageExecutionTime();
    double getAverageFormatTime();

    void setExposure(float);
    void setBrightness(float);
    void setSaturation(float);
    void setContrast(float);

    //camera settings

private:
    //image format
    void imageFormater(cv::Mat&, cv::Mat&);
    cv::Mat imageFormaterLeft();
    cv::Mat imageFormaterRight();

    //video
//    void writeImages();

    //Pylon global variables
    CInstantCameraArray cameras;

    CImageFormatConverter formatConverter;

    CPylonImage pylonImageLeft;
    CPylonImage pylonImageRight;

    CGrabResultPtr pylonResultLeft;
    CGrabResultPtr pylonResultRight;

    size_t c_maxCamerasToUse = 2;

    CVideoWriter videoWriterLeft;
    CVideoWriter videoWriterRight;

    bool createVideo = false;

    //calculation variables
    int completeGrabAndFormat = 0;
    int executionCounter = 0;
    int formatTime = 0;
    int formatCounter = 0;
    int saveCounter = 0;
};

#endif // IMAGEGRABBER_H
