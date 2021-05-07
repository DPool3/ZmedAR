#include "imageprocessor.h"

ImageProcessor::ImageProcessor()
{
    imageSet = ImageSet("/home/daniel/ZAR/ImageSets/22-04-2021-08-15-15");
}

QImage ImageProcessor::prepImageForDisplay(cv::Mat & image){
    //Resize Images
    cv::resize(image, image, cv::Size(480, 320), 0, 0);

    //Change to RGB format & save it in global Mat
    cv::cvtColor(image, image, CV_BGR2RGB);

    //Convert to QImage
    QImage qimg((const unsigned char*) image.data, image.cols, image.rows, QImage::Format_RGB888);

    return qimg;
}

void ImageProcessor::cannyEdgeOnImagePair(cv::Mat & imageLeft, cv::Mat &imageRight){
    cv::Mat imgLeftGray, imgRightGray, detectedEdgesLeft, detectedEdgesRight;
    int lowThreshhold = 10;
    const int ratio = 3;
    const int kernel_size = 3;

    cv::cvtColor(imageLeft, imgLeftGray, CV_BGR2GRAY);
    cv::cvtColor(imageRight, imgRightGray, CV_BGR2GRAY);

    cv::blur(imgLeftGray, detectedEdgesLeft, cv::Size(3,3));
    cv::blur(imgRightGray, detectedEdgesRight, cv::Size(3,3));

    cv::Canny(detectedEdgesLeft, detectedEdgesLeft, lowThreshhold, lowThreshhold*ratio, kernel_size);
    cv::Canny(detectedEdgesRight, detectedEdgesRight, lowThreshhold, lowThreshhold*ratio, kernel_size);

    //copy edges in cv mat
    cv::Mat destLeft, destRight;
    imageLeft.copyTo(destLeft, detectedEdgesLeft);
    imageRight.copyTo(destRight, detectedEdgesRight);

    imageLeft = destLeft;
    imageRight = destRight;
}

void ImageProcessor::stereoVisualOdometry(cv::Mat & imageLeft, cv::Mat & imageRight){
    //read in used image set to use variables for rectification and remapping
    bool useSift = false;

//    cv::Mat new_CamL = imageSet.getNewCamL();
//    cv::Mat new_CamR = imageSet.getNewCamR();
//    cv::Mat DistCoefL = imageSet.getDistCoefL();
//    cv::Mat DistCoefR = imageSet.getDistCoefR();
//    cv::Mat rect_l = imageSet.getRectL();
//    cv::Mat rect_r = imageSet.getRectR();
//    cv::Mat proj_mat_l = imageSet.getProjMatL();
//    cv::Mat proj_mat_r = imageSet.getProjMatR();
    cv::Mat Left_Stereo_Map1 = imageSet.getLeftStereoMap1();
    cv::Mat Left_Stereo_Map2 = imageSet.getLeftStereoMap2();
    cv::Mat Right_Stereo_Map1 = imageSet.getRightStereoMap1();
    cv::Mat Right_Stereo_Map2 = imageSet.getRightStereoMap2();

    cv::Mat grayL, grayR, remappedL, remappedR;

//    cv::cvtColor(imageLeft, grayL, CV_RGB2GRAY);
//    cv::cvtColor(imageRight, grayR, CV_RGB2GRAY);

    cv::remap(imageLeft, remappedL, Left_Stereo_Map1, Left_Stereo_Map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    cv::remap(imageRight, remappedR, Right_Stereo_Map1, Right_Stereo_Map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

    //1. Feature detection for both stereo pair images
    if(useSift){
        //SIFT
    }
    else{
        //FAST
        cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create(20, true, cv::FastFeatureDetector::TYPE_9_16);

        std::vector<cv::KeyPoint> keyPointVectorLeft;
        std::vector<cv::KeyPoint> keyPointVectorRight;

        fast->detect(remappedL, keyPointVectorLeft, cv::Mat());
        fast->detect(remappedR, keyPointVectorRight, cv::Mat());

        cv::drawKeypoints(remappedL, keyPointVectorLeft, remappedL);
        cv::drawKeypoints(remappedR, keyPointVectorRight, remappedR);

        imageLeft = remappedL;
        imageRight = remappedR;
    }


    //2. Feature matching between the left images of each stereo pair
    //3. Estimating the relative pose between the left images and finding the inlier matches
    //      -> RANSAC
    //4. Determining the motion type and direction of the camera
    //      -> Kalman filter
    //5. Finding the exact translation motion of the camera
}
