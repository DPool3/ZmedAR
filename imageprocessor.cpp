#include "imageprocessor.h"

ImageProcessor::ImageProcessor()
{

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
