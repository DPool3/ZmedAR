#ifndef CAMERABASEDCONTROLLER_H
#define CAMERABASEDCONTROLLER_H

#include <iostream>
#include <QImage>
#include <QTimer>
#include <QObject>
#include <opencv2/opencv.hpp>

#include <PylonCamera.h>
#include <videomanager.h>
#include <dialogmanager.h>
#include <imageprocessor.h>

class CameraBasedController : public QObject
{
    Q_OBJECT

public:
    CameraBasedController();
    ~CameraBasedController();

    void startStopCameraBasedProcess();
    void startStopRecording();
    void getProcessedImages(QImage &, QImage &);

    void setExposure(int);
    void setBrightness(double);
    void setSaturation(double);

    bool isRunning();
    bool isRecording();

    void stopController();
    void startController();

private:

    //maybe
    //void reinitCameras();

    pylonCamera cameras;
    VideoManager videoManager;
    ImageProcessor imageProcessor;

    cv::VideoWriter writerLeft, writerRight;
    cv::Mat imageLeft, imageRight;

    QTimer * saveTimer;
    QTimer * cameraImageTimer;

    int processingMethod;

private slots:
    void saveImages();
    void getCameraImages();
};

#endif // CAMERABASEDCONTROLLER_H
