#ifndef IMAGESETDIALOG_H
#define IMAGESETDIALOG_H

#include <QDialog>
#include <QTimer>
#include <opencv2/opencv.hpp>
#include <pylon/PylonIncludes.h>

#include "helperfunctions.h"
#include "mainsettings.h"
#include "imageset.h"

using namespace Pylon;

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

    void enableUI(bool value);

    void update();
    void counter();

    void initPylon();

private:
    Ui::ImageSetDialog *ui;

    //Pylon
    CInstantCameraArray cameras;

    CImageFormatConverter formatConverter;

    CPylonImage pylonImageLeft;
    CPylonImage pylonImageRight;

    CGrabResultPtr pylonResultLeft;
    CGrabResultPtr pylonResultRight;

    std::size_t c_maxCamerasToUse = 2;

    //Image Set
    ImageSet imageSet;

    QTimer *updateTimer;
    QTimer *counterTimer;

    bool showVideo = false;
    bool setImageSetDefault = false;

    cv::VideoCapture captureLeft;
    cv::VideoCapture captureRight;

    cv::Mat imageLeft;
    cv::Mat imageRight;
};

#endif // IMAGESETDIALOG_H
