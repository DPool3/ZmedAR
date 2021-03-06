#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/viz.hpp>
#include <opencv4/opencv2/calib3d/calib3d.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>
#include <opencv4/opencv2/xfeatures2d.hpp>
#include <QElapsedTimer>
#include <iostream>
#include <QImage>
#include <thread>
#include <math.h>

#include "imageset.h"
#include "directorymanager.h"

/**
 * @brief Die ImageProcessor class führt fast alle bildverarbeitenden Methoden durch.
 * Zu diesen gehören unteranderem die stereovisuelle Odometrie, die Generierung
 * der Anaglyph 3D Bilder, sowie die Vorverarbeitung der Bilder für die
 * Darstellung in der GUI.
 */
class ImageProcessor
{
public:
    ImageProcessor();
    ImageProcessor(int detector, int descriptor, int matcher);

    void closeTrackingFile();
    void createNewTrackingFile();

    QImage prepImageForDisplay(cv::Mat&);
    QImage generateAnaglyphImage(cv::Mat, cv::Mat);
    void cannyEdgeOnImagePair(cv::Mat&, cv::Mat&);
    void stereoVisualOdometry(cv::Mat, cv::Mat);

private:

    void detectFeaturesParallel(
            cv::Mat remappedL,
            cv::Mat remappedR,
            std::vector<cv::KeyPoint>& keyPointVectorLeft,
            std::vector<cv::KeyPoint>& keyPointVectorRight,
            int selectedFeatureDetector);

    void describeFeaturesParallel(
            cv::Mat remappedL,
            cv::Mat remappedR,
            cv::Mat& descriptorLeft,
            cv::Mat& descriptorRight,
            std::vector<cv::KeyPoint> keyPointVectorLeft,
            std::vector<cv::KeyPoint> keyPointVectorRight,
            int selectedFeatureDescription);

    void calc3DPointsOfInliers( std::vector<cv::KeyPoint> previousMatches,
                                std::vector<cv::KeyPoint> currentMatches,
                                cv::Mat camMat,
                                std::vector<cv::Point2f>& prevMatchesIn,
                                std::vector<cv::Point2f>& currMatchesIn,
                                std::vector<cv::Point3f>& points3DInliers);

    void calcWorldCoords(cv::Mat r, cv::Mat& t);

    void addLineToFile(cv::Mat leftCameraWorldTranslation, cv::Mat rightCameraWorldTranslation);

    std::vector<cv::KeyPoint> featureDetectionMethod(cv::Mat, int);
    cv::Mat featureDescriptionMethod(cv::Mat, std::vector<cv::KeyPoint>, int);
    std::vector<cv::DMatch> featureMatchingMethod(cv::Mat, cv::Mat, int, int);

    ImageSet imageSet;

    //stereo maps for remapping
    cv::Mat leftStereoMap1;
    cv::Mat leftStereoMap2;
    cv::Mat rightStereoMap1;
    cv::Mat rightStereoMap2;

    //"/home/daniel/ZAR/ImageSets/22-04-2021-08-15-15"
    //"/home/daniel/ZAR/ImageSets/22-07-2021-08-44-39"
    std::string selectedImageSet = "/home/daniel/ZAR/ImageSets/22-07-2021-08-44-39";

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
    cv::Mat leftCameraWorldTranslation, rightCameraWorldTranslation;

    //mean time calculation
    int iterations = 0;
    int ransacCounterLeft = 0;
    int ransacCounterRight = 0;
    int numberOfMatchesAcc = 0;
    int numberOfTemporalMatchesAcc = 0;
    int numberOfKeyPointsAcc = 0;
    float detectionTimeAcc = 0;
    float descriptionTimeAcc = 0;
    float matchingTimeAcc = 0;
    float temporalMatchingTimeAcc = 0;
    float completeTimeAcc = 0;

    double numberOfInliersAcc = 0;
    double numberOfOutliersAcc = 0;

};

#endif // IMAGEPROCESSOR_H
