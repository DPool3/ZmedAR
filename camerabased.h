#ifndef CAMERABASED_H
#define CAMERABASED_H

#include <QDialog>
#include <QTimer>
#include <QElapsedTimer>
#include <opencv2/opencv.hpp>
#include <thread>
#include <future>

#include "helperfunctions.h"
#include "PylonCamera.h"

using namespace std;

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
    void saveImagesWithPylon();

    //parallel
    void saveImagesThread();
    void saveLeft(cv::Mat);
    void saveRight(cv::Mat);

private:
    //UI
    Ui::CameraBased *ui;

    //Pylon
    pylonCamera cameras;

    //Opencv
    QTimer *retrieveImagesTimer;
    QTimer *saveImageTimer;

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
    double saveExecutionCounter = 0;
    int displayTimeOverall = 0;
    int saveTimeOverall = 0;
    int completeTimeOverall = 0;

signals:
    void displayNext();
    void saveNext();
};

#endif // CAMERABASED_H
