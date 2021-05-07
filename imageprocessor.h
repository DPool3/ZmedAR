#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <QImage>

#include "imageset.h"

class ImageProcessor
{
public:
    ImageProcessor();

    QImage prepImageForDisplay(cv::Mat&);
    void cannyEdgeOnImagePair(cv::Mat&, cv::Mat&);
    void stereoVisualOdometry(cv::Mat&, cv::Mat&);

    ImageSet imageSet;
};

#endif // IMAGEPROCESSOR_H
