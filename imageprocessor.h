#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/xfeatures2d.hpp>
#include <QElapsedTimer>
#include <iostream>
#include <QImage>
#include <thread>

#include "imageset.h"
#include "directorymanager.h"

class ImageProcessor
{
public:
    ImageProcessor();
    ImageProcessor(int detector, int descriptor, int matcher);

    void closeTrackingFile();
    void createNewTrackingFile();

    QImage prepImageForDisplay(cv::Mat&);
    void cannyEdgeOnImagePair(cv::Mat&, cv::Mat&);
    void stereoVisualOdometry(cv::Mat, cv::Mat);

private:

    void detectFeatures(
            cv::Mat remappedL,
            cv::Mat remappedR,
            std::vector<cv::KeyPoint>& keyPointVectorLeft,
            std::vector<cv::KeyPoint>& keyPointVectorRight,
            int selectedFeatureDetector);

    void describeFeatures(
            cv::Mat remappedL,
            cv::Mat remappedR,
            cv::Mat& descriptorLeft,
            cv::Mat& descriptorRight,
            std::vector<cv::KeyPoint> keyPointVectorLeft,
            std::vector<cv::KeyPoint> keyPointVectorRight,
            int selectedFeatureDescription);

    void featureMatching(
            cv::Mat descriptorLeft,
            cv::Mat descriptorRight,
            std::vector<cv::KeyPoint> keyPointVectorLeft,
            std::vector<cv::KeyPoint> keyPointVectorRight,
            int selectedFeatureDescriptor,
            int selectedFeatureMatching,
            std::vector<cv::DMatch>& matchesLeftRight,
            std::vector<cv::KeyPoint>& matchedKeyPointsLeft_tMinus1,
            std::vector<cv::KeyPoint>& matchedKeyPointsLeft_t,
            std::vector<cv::KeyPoint>& matchedKeyPointsRight_tMinus1,
            std::vector<cv::KeyPoint>& matchedKeyPointsRight_t);

    void calc3DPointsOfInliers( std::vector<cv::KeyPoint> previousMatches,
                                std::vector<cv::KeyPoint> currentMatches,
                                cv::Mat camMat,
                                std::vector<cv::Point2f>& prevMatchesIn,
                                std::vector<cv::Point2f>& currMatchesIn,
                                std::vector<cv::Point3f>& points3DInliers);

    void drawInliers(cv::Mat prevImage,
                     cv::Mat currImage,
                     std::vector<cv::Point2f> prevMatchesIn,
                     std::vector<cv::Point2f> currMatchesIn,
                     std::vector<cv::Point3f> points3DIn,
                     cv::Mat r,
                     cv::Mat t,
                     cv::Mat camMat,
                     cv::Mat distCoef);

    cv::Mat calcWorldCoords(cv::Mat r, cv::Mat t);

    void addLineToFile(cv::Mat leftCameraWorldCoords, cv::Mat rightCameraWorldCoord);

    std::vector<cv::KeyPoint> featureDetectionMethod(cv::Mat, int);
    cv::Mat featureDescriptionMethod(cv::Mat, std::vector<cv::KeyPoint>, int);
    std::vector<cv::DMatch> featureMatchingMethod(cv::Mat, cv::Mat, int, int);

    ImageSet imageSet;

    std::ofstream trackingFile;

    //1=ORB, 2=FAST, 3=BRISK, 4=SIFT, 5=SURF
    int selectedFeatureDetector = 1;

    //1=ORB, 2=BRIEF, 3=BRISK, 4=SIFT, 5=SURF
    int selectedFeatureDescriptor = 1;

    //1=Brute Force, 2=FLANN (Fast Library for Approximate Nearest Neighbors)
    int selectedFeatureMatching = 1;

    cv::Mat prevImageLeft, prevImageRight;
    std::vector<cv::KeyPoint> prevKeypointsLeft, prevKeypointsRight;
    cv::Mat prevDescriptorLeft, prevDescriptorRight;
    std::vector<cv::DMatch> prevMatchesLeftRight;

    //select methods
    std::string selectedDetector = "";
    std::string selectedDescriptor = "";
    std::string selectedMatcher = "";

    //world coordinates of cameras
    cv::Mat leftCameraWorldCoords;
    cv::Mat rightCameraWorldCoords;

    //mean time calculation
    int iterations = 0;
    int numberOfMatchesAcc = 0;
    int numberOfKeyPointsAcc = 0;
    double detectionTimeAcc = 0;
    double descriptionTimeAcc = 0;
    double matchingTimeAcc = 0;
    double completeTimeAcc = 0;

};

#endif // IMAGEPROCESSOR_H
