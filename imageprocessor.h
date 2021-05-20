#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <QElapsedTimer>
#include <iostream>
#include <QImage>
#include <thread>

#include "imageset.h"

class ImageProcessor
{
public:
    ImageProcessor();

    QImage prepImageForDisplay(cv::Mat&, std::string);
    void cannyEdgeOnImagePair(cv::Mat&, cv::Mat&);
    void stereoVisualOdometry(cv::Mat&, cv::Mat&);

private:
    std::vector<cv::DMatch> bruteForceMatches(cv::Mat, cv::Mat);
    std::vector<cv::DMatch> flann(cv::Mat, cv::Mat);

    std::vector<cv::KeyPoint> featureDetectionMethod(cv::Mat, int);
    cv::Mat featureDescriptionMethod(cv::Mat, std::vector<cv::KeyPoint>, int);
    std::vector<cv::DMatch> featureMatchingMethod(cv::Mat, cv::Mat, int);

    ImageSet imageSet;

    cv::Mat prevImageLeft, prevImageRight;
    std::vector<cv::KeyPoint> prevKeypointsLeft, prevKeypointsRight;
    cv::Mat prevDescriptorLeft, prevDescriptorRight;

    //select methods
    std::string selectedDetector = "";
    std::string selectedDescriptor = "";
    std::string selectedMatcher = "";

    //mean time calculation
    int iterations = 0;
    double detectionTimeAcc = 0;
    double descriptionTimeAcc = 0;
    double matchingTimeAcc = 0;
    double completeTimeAcc = 0;

};

#endif // IMAGEPROCESSOR_H
