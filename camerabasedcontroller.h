#ifndef CAMERABASEDCONTROLLER_H
#define CAMERABASEDCONTROLLER_H

#include <iostream>
#include <QImage>
#include <QElapsedTimer>
#include <QTimer>
#include <QObject>
#include <opencv4/opencv2/opencv.hpp>

#include "PylonCamera.h"
#include "videomanager.h"
#include "dialogmanager.h"
#include "imageprocessor.h"
#include "vivetracking.h"

class CameraBasedController : public QObject
{
    Q_OBJECT

public:
    CameraBasedController();
    ~CameraBasedController();

    void startStopCameraBasedProcess();
    void startStopRecording();
    void startRecording();
    void stopRecording();
    bool getProcessedImages(QImage &, QImage &);
    bool getCameraImages();

    void setExposure(int);
    void setBrightness(double);
    void setSaturation(double);
    void setContrast(double);

    bool isRunning();

    void stopController();
    void startController();

    void initTracker();
    void trackTrackers();

    bool useViveTracking = false;
    bool useCameraTracking = false;
    bool useVideoSaving = false;

private:

    //maybe
    //void reinitCameras();

    pylonCamera cameras;
    VideoManager videoManager;
    ImageProcessor imageProcessor;
    ViveTracking* viveTracking;

    cv::VideoWriter writerLeft, writerRight;
    cv::Mat origLeft, origRight;
    cv::Mat imageLeft, imageRight;
    cv::Mat imageLeftSave, imageRightSave;

    QTimer * saveTimer;
    QTimer * cameraImageTimer;
    QTimer * cameraTrackingTimer;

    QElapsedTimer timer;
    int iterationCounter = 0;

private slots:
    void saveImages();
    void getCameraImagesAndTracking();
    void trackCameras();
};

#endif // CAMERABASEDCONTROLLER_H
