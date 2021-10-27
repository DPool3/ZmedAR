#ifndef STEREOCALIBRATIONCONTROLLER_H
#define STEREOCALIBRATIONCONTROLLER_H

#include <iostream>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <thread>
#include "dialogmanager.h"
#include "imageset.h"

/**
 * @brief Die StereoCalibrationController class führt den kompletten Prozess
 * der Stereokamerakalibrierung, entzerrung und des Remappings durch.
 * Neu errechnete Werte werden in dem verwendeten Image Set gespeichert.
 * Gleichzeitig gibt die Klasse Bilder der Corner Detection
 * sowie des Remappings an stereocalibration view zurück.
 */
class StereoCalibrationController
{
public:
    StereoCalibrationController();
    bool loadImageSet();
    bool startStereoCalibration();
    void getCalibrationInfo(        int& board_width,
                                    int& board_height,
                                    int& num_imgs,
                                    float& square_size,
                                    std::string& patternType,
                                    double& stereoReprojectionError);

    void getImagesForDisplay(QImage& imageLeft, QImage& imageRight);
    QString getImageSetPath();
    bool checkNewImageForDisplay();
    bool checkCalibrationRunning();

private:
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

    cv::Mat imgL, imgR, grayL, grayR;

    cv::Mat displayImageLeft, displayImageRight;

    QString imageSetPath;
    ImageSet imageSet;

    double stereoReprojectionError;

    bool newImagesForDisplay = false;
    bool calibrationRunning = false;
};

#endif // STEREOCALIBRATIONCONTROLLER_H
