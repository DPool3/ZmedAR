#ifndef STEREOCALIBRATION_H
#define STEREOCALIBRATION_H

#include <QDialog>
#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <string>
#include "helperfunctions.h"
#include "imageset.h"

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
    void displayImages(cv::Mat, cv::Mat);
    void displayImageRight(cv::Mat);
    void displayImageLeft(cv::Mat);

    void on_displayImages_checkbox_toggled(bool checked);

private:
    Ui::StereoCalibration *ui;

    ImageSet imageSet;

    //Actual 3D coordinate of those checkerboard points
    std::vector< std::vector< cv::Point3f > > object_points;
    //Checkerboard corner coordinates in the image
    std::vector< std::vector< cv::Point2f > > imagePointsL, imagePointsR;
    std::vector< cv::Point2f > cornersL, cornersR;

    cv::Mat imgL, imgR, grayL, grayR;

    bool showImages = true;

    //methods
    void loadImageSet();
    void setImageSetDataInUi();
    std::string splitFileName(const std::string&);

    void performSteroCalibration();
    void loadImgPoints(int, int, int, float, std::string, std::string, std::string);
};

#endif // STEREOCALIBRATION_H
