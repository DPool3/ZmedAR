#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <opencv2/opencv.hpp>
#include <QImage>

class ImageProcessor
{
public:
    ImageProcessor();

    QImage prepImageForDisplay(cv::Mat&);
    void cannyEdgeOnImagePair(cv::Mat&, cv::Mat&);
    void slam(cv::Mat, cv::Mat);
};

#endif // IMAGEPROCESSOR_H
