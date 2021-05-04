#ifndef IMAGESETDIALOG_H
#define IMAGESETDIALOG_H

#include <QDialog>
#include <QTimer>
#include <opencv2/opencv.hpp>

#include "PylonCamera.h"
#include "dialogmanager.h"
#include "videomanager.h"
#include "imageset.h"

using namespace std;

namespace Ui {
class ImageSetDialog;
}

class ImageSetDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ImageSetDialog(QWidget *parent = 0);
    ~ImageSetDialog();

private slots:
    void on_showVideosCheckBox_toggled(bool checked);
    void on_setImageSet_checkBox_toggled(bool checked);

    void on_imageSetRecord_button_clicked();

    void saveImagesInImageSet(cv::Mat, cv::Mat);
    void saveInputInImageSet();
    void displayImages(cv::Mat, cv::Mat);
    std::string getPatternType();
    ImageSet createImageSet();

    void lockUi();
    void releaseUi();

    void update();
    void counter();

private:
    Ui::ImageSetDialog *ui;

    //Pylon
    pylonCamera cameras;

    //Image Set
    ImageSet imageSet;

    QTimer *updateTimer;
    QTimer *counterTimer;

    bool showVideo = false;
    bool setImageSetDefault = false;

    cv::Mat imageLeft;
    cv::Mat imageRight;
};

#endif // IMAGESETDIALOG_H
