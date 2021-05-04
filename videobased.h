#ifndef VIDEOBASED_H
#define VIDEOBASED_H

#include <QDialog>
#include <QTimer>
#include <opencv2/opencv.hpp>

#include "dialogmanager.h"
#include "videomanager.h"

namespace Ui {
class VideoBased;
}

class VideoBased : public QDialog
{
    Q_OBJECT

public:
    explicit VideoBased(QWidget *parent = 0);
    ~VideoBased();

private slots:
    void on_startVideo_button_clicked();
    void on_showVideosCheckBox_toggled(bool checked);

    void on_searchFile_button_2_clicked();
    void on_searchFile_button_clicked();

    void saveEnteredPaths();
    void displayImages(cv::Mat, cv::Mat);

    void update();

    void start();
    void stop();

    void releaseUi();
    void lockUi();

private:
    VideoManager videoManager;

    Ui::VideoBased *ui;

    QTimer *updateTimer;

    std::string videoPathLeft = "";
    std::string videoPathRight = "";

    cv::VideoCapture captureLeft;
    cv::VideoCapture captureRight;

    cv::Mat imageLeft;
    cv::Mat imageRight;

    FileSystem fsd;

    bool showVideo = false;
};

#endif // VIDEOBASED_H
