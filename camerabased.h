#ifndef CAMERABASED_H
#define CAMERABASED_H

#include <QDialog>
#include <QTimer>
#include <QElapsedTimer>
#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>
#include <thread>
#include <future>

#include "helperfunctions.h"

using namespace Pylon;

namespace Ui {
class CameraBased;
}

class CameraBased : public QDialog
{
    Q_OBJECT

public:
    explicit CameraBased(QWidget *parent = 0);
    ~CameraBased();

private slots:
    void on_startStopRecording_clicked();
    void on_showVideosCheckBox_toggled(bool checked);
    void on_saveVideoCheckBox_toggled(bool checked);

    void retrieveImages();
    void displayImages(cv::Mat, cv::Mat);
    void saveImages(cv::Mat, cv::Mat);

    void initPylon();

    //parallel
    void saveLeft(cv::Mat);
    void saveRight(cv::Mat);
    void displayImageLeft(cv::Mat);
    void displayImageRight(cv::Mat);
    void formatImages(cv::Mat&, cv::Mat&);
    cv::Mat formatLeft();
    cv::Mat formatRight();

private:
    //UI
    Ui::CameraBased *ui;

    //Pylon
    CInstantCameraArray cameras;

    CImageFormatConverter formatConverter;

    CPylonImage pylonImageLeft;
    CPylonImage pylonImageRight;

    CGrabResultPtr pylonResultLeft;
    CGrabResultPtr pylonResultRight;

    std::size_t c_maxCamerasToUse = 2;

    //Opencv
    QTimer *retrieveImagesTimer;

    cv::VideoCapture captureLeft;
    cv::VideoCapture captureRight;

    cv::VideoWriter writerLeft;
    cv::VideoWriter writerRight;

    cv::Mat imageLeft;
    cv::Mat imageRight;

    //helper
    bool showVideo = false;
    bool saveVideo = false;

    QElapsedTimer overallTimer;
    double executionCounter = 0;
    int convertTimeOverall = 0;
    int displayTimeOverall = 0;
    int saveTimeOverall = 0;
    int completeTimeOverall = 0;

signals:
    void displayNext();
    void saveNext();
};

#endif // CAMERABASED_H
