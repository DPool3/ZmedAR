#pragma once
#ifndef STEREOCALIBRATION_H
#define STEREOCALIBRATION_H

#include <QDialog>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <QTextStream>
#include <string>
#include <thread>

#include "dialogmanager.h"
#include "imageset.h"

using namespace std;

namespace Ui {
class StereoCalibration;
}

class StereoCalibration : public QDialog
{
    Q_OBJECT

public:
    explicit StereoCalibration(QWidget *parent = 0);
    ~StereoCalibration();

private slots:

    void on_startCalibration_button_clicked();
    void on_searchFile_button_clicked();
    void on_displayImages_checkbox_toggled(bool);
    void on_resizeFactorSpinBox_valueChanged(double);

private:
    Ui::StereoCalibration *ui;

    ImageSet imageSet;

    //Variables to store images while finding corners in pattern images
    cv::Mat imgL, imgR, grayL, grayR;

    //check for displaying images in ui
    bool showImages = true;
    double resizeFactor = 1;

    //methods
    void loadImageSet(std::string);
    void setImageSetDataInUi();
    void displayImages(cv::Mat, cv::Mat);
    std::string splitFileName(const std::string&);

    void performSteroCalibration();
    void loadImgPoints(int, int, int,
                       float,
                       std::string, std::string, std::string,
                       std::vector< std::vector< cv::Point3f > >&,
                       std::vector< std::vector< cv::Point2f > >&, std::vector< std::vector< cv::Point2f > >&);

    void findChessBoardCorners(cv::Mat, cv::Size, std::vector< cv::Point2f>&, bool&);
    void findCircleGridSym(cv::Mat, cv::Size, std::vector< cv::Point2f>&, bool&);
    void findCircleGridAsym(cv::Mat, cv::Size, std::vector< cv::Point2f>&, bool&);

    void lockUi();
    void releaseUi();

};

#endif // STEREOCALIBRATION_H
