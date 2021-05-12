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
    QElapsedTimer totalImageMatchingTimer;
    totalImageMatchingTimer.start();

    std::string selectedDetector = "";
    std::string selectedDescriptor = "";
    std::string selectedMatcher = "";
    iterations++;

    //read in used image set to use variables for rectification and remapping
    int featureDetectorMethod = 1;
    int featureDescriptorMethod = 1;
    int matchingMethod = 1;

    cv::Mat Left_Stereo_Map1 = imageSet.getLeftStereoMap1();
    cv::Mat Left_Stereo_Map2 = imageSet.getLeftStereoMap2();
    cv::Mat Right_Stereo_Map1 = imageSet.getRightStereoMap1();
    cv::Mat Right_Stereo_Map2 = imageSet.getRightStereoMap2();

    cv::Mat grayL, grayR, remappedL, remappedR;

    cv::Mat descriptorLeft, descriptorRight;
    std::vector<cv::DMatch> matchesLeft, matchesRight;
    std::vector<cv::KeyPoint> keyPointVectorLeft, keyPointVectorRight;

//    cv::cvtColor(imageLeft, grayL, CV_RGB2GRAY);
//    cv::cvtColor(imageRight, grayR, CV_RGB2GRAY);

    cv::remap(imageLeft, remappedL, Left_Stereo_Map1, Left_Stereo_Map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    cv::remap(imageRight, remappedR, Right_Stereo_Map1, Right_Stereo_Map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

    QElapsedTimer detectorTimer;
    detectorTimer.start();

    //1. Feature detection for both stereo pair images
    //1=ORB, 2=FAST, 3=BRISK, 4=AKAZE, 5=AGAST, 6=SIFT, 7=SURF
    switch (featureDetectorMethod) {
    case 1:{
        //ORB - Oriented FAST and Rotated BRIEF
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        orb->detect(remappedL, keyPointVectorLeft, cv::Mat());
        orb->detect(remappedR, keyPointVectorRight, cv::Mat());

        cv::drawKeypoints(remappedL, keyPointVectorLeft, remappedL);
        cv::drawKeypoints(remappedR, keyPointVectorRight, remappedR);

        imageLeft = remappedL;
        imageRight = remappedR;

        selectedDetector = "ORB";

        break;
    }
    case 2:{
        //FAST is only a feature detector and has no descriptors to compute.
        cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create();
        fast->detect(remappedL, keyPointVectorLeft, cv::Mat());
        fast->detect(remappedR, keyPointVectorRight, cv::Mat());

        cv::drawKeypoints(remappedL, keyPointVectorLeft, remappedL);
        cv::drawKeypoints(remappedR, keyPointVectorRight, remappedR);

        imageLeft = remappedL;
        imageRight = remappedR;

        selectedDetector = "FAST";

        break;
    }
    case 3:{
        //BRISK
        cv::Ptr<cv::BRISK> brisk = cv::BRISK::create();
        brisk->detect(remappedL, keyPointVectorLeft, cv::Mat());
        brisk->detect(remappedR, keyPointVectorRight, cv::Mat());

        cv::drawKeypoints(remappedL, keyPointVectorLeft, remappedL);
        cv::drawKeypoints(remappedR, keyPointVectorRight, remappedR);

        imageLeft = remappedL;
        imageRight = remappedR;

        selectedDetector = "BRISK";

        break;
    }
    case 4:{
        //AKAZE
        cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
        akaze->detect(remappedL, keyPointVectorLeft, cv::Mat());
        akaze->detect(remappedR, keyPointVectorRight, cv::Mat());

        cv::drawKeypoints(remappedL, keyPointVectorLeft, remappedL);
        cv::drawKeypoints(remappedR, keyPointVectorRight, remappedR);

        imageLeft = remappedL;
        imageRight = remappedR;

        selectedDetector = "AKAZE";

        break;
    }
    case 5:{
        //AGAST
        cv::Ptr<cv::AgastFeatureDetector> agast = cv::AgastFeatureDetector::create();
        agast->detect(remappedL, keyPointVectorLeft, cv::Mat());
        agast->detect(remappedR, keyPointVectorRight, cv::Mat());

        cv::drawKeypoints(remappedL, keyPointVectorLeft, remappedL);
        cv::drawKeypoints(remappedR, keyPointVectorRight, remappedR);

        imageLeft = remappedL;
        imageRight = remappedR;

        selectedDetector = "AGAST";

        break;
    }
    case 6:{
        cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create();
        sift->detect(remappedL, keyPointVectorLeft, cv::Mat());
        sift->detect(remappedR, keyPointVectorRight, cv::Mat());

        cv::drawKeypoints(remappedL, keyPointVectorLeft, remappedL);
        cv::drawKeypoints(remappedR, keyPointVectorRight, remappedR);

        imageLeft = remappedL;
        imageRight = remappedR;

        selectedDetector = "SIFT";

        break;
    }
    case 7:{
        cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create();
        surf->detect(remappedL, keyPointVectorLeft, cv::Mat());
        surf->detect(remappedR, keyPointVectorRight, cv::Mat());

        cv::drawKeypoints(remappedL, keyPointVectorLeft, remappedL);
        cv::drawKeypoints(remappedR, keyPointVectorRight, remappedR);

        imageLeft = remappedL;
        imageRight = remappedR;

        selectedDetector = "SURF";

        break;
    }
    default:
        return;
    }

    double detectionTime = detectorTimer.elapsed();
    detectionTimeAcc = detectionTimeAcc + detectionTime;

    QElapsedTimer descriptorTimer;
    descriptorTimer.start();

    //1.2 Feature description for both images
    //1=ORB, 2=BRIEF, 3=SIFT, 4=SURF, 5=BRISK, 6=AKAZE
    switch (featureDescriptorMethod) {
    case 1:{
        //ORB - Oriented FAST and Rotated BRIEF
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        orb->compute(remappedL, keyPointVectorLeft, descriptorLeft);
        orb->compute(remappedR, keyPointVectorRight, descriptorRight);

        selectedDescriptor = "ORB";

        break;
    }
    case 2:{
        //BRIEF (Binary Robust Independent Elementary) -> non free
        cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> brief = cv::xfeatures2d::BriefDescriptorExtractor::create();
        brief->compute(remappedL, keyPointVectorLeft, descriptorLeft);
        brief->compute(remappedR, keyPointVectorRight, descriptorRight);

        selectedDescriptor = "BRIEF";

        break;
    }
    case 3:{
        //SIFT (Scale-invariant feature transform)
        cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create();
        sift->compute(remappedL, keyPointVectorLeft, descriptorLeft);
        sift->compute(remappedR, keyPointVectorRight, descriptorRight);

        selectedDescriptor = "SIFT";

        break;
    }
    case 4:{
        //SURF (Speeded Up Robust Features)
        cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create();
        surf->compute(remappedL, keyPointVectorLeft, descriptorLeft);
        surf->compute(remappedR, keyPointVectorRight, descriptorRight);

        selectedDescriptor = "SURF";

        break;
    }
    case 5:{
        //BRISK (Binary Robust Invariant Scalable Keypoints)
        cv::Ptr<cv::BRISK> brisk = cv::BRISK::create();
        brisk->compute(remappedL, keyPointVectorLeft, descriptorLeft);
        brisk->compute(remappedR, keyPointVectorRight, descriptorRight);

        selectedDescriptor = "BRISK";

        break;
    }
    case 6:{
        //Can Only be used with KAZE or AKAZE Keypoints
        cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
        akaze->compute(remappedL, keyPointVectorLeft, descriptorLeft);
        akaze->compute(remappedR, keyPointVectorRight, descriptorRight);

        selectedDescriptor = "AKAZE";

        break;
    }
    default:
        break;
    }

    double descriptionTime = descriptorTimer.elapsed();
    descriptionTimeAcc = descriptionTimeAcc + descriptionTime;

    QElapsedTimer matchingTimer;
    matchingTimer.start();

    //2. Feature matching between the left images of each stereo pair
    //Brute force (Basics of Brute-Force Matcher ) oder FLANN (Fast Library for Approximate Nearest Neighbors)
    if(!prevImageLeft.empty() && !prevImageRight.empty()){
        switch (matchingMethod) {
        case 1:{
            //Brute-Force Matcher
            matchesLeft = bruteForceMatches(descriptorLeft, prevDescriptorLeft);
            matchesRight = bruteForceMatches(descriptorRight, prevDescriptorRight);

            selectedMatcher = "Brute-Force";

            break;
        }
        case 2:{
            //FLANN (Fast Library for Approximate Nearest Neighbors) Matcher
            matchesLeft = flann(descriptorLeft, prevDescriptorLeft);
            matchesRight = flann(descriptorRight, prevDescriptorRight);

            selectedMatcher = "FLANN";

            break;
        }
        default:
            break;
        }

        //Display Matches
//        cv::Mat imgMatchesLeft, imgMatchesRight;
//        cv::drawMatches(prevImageLeft, prevKeypointsLeft, remappedL, keyPointVectorLeft, matchesLeft, imgMatchesLeft);
//        cv::drawMatches(prevImageRight, prevKeypointsRight, remappedR, keyPointVectorRight, matchesRight, imgMatchesRight);
//        cv::imshow("matches left", imgMatchesLeft);
//        cv::imshow("matches right", imgMatchesRight);
//        cv::waitKey(5000);
//        cv::destroyAllWindows();
    }

    double matchingTime = matchingTimer.elapsed();
    matchingTimeAcc = matchingTimeAcc + matchingTime;

    //3. Estimating the relative pose between the left images and finding the inlier matches
    //      -> RANSAC
    //4. Determining the motion type and direction of the camera
    //      -> Kalman filter
    //5. Finding the exact translation motion of the camera

    double completeTime = totalImageMatchingTimer.elapsed();
    completeTimeAcc = completeTimeAcc + completeTime;

    //Set the current values as previous
    prevImageLeft = remappedL;
    prevImageRight = remappedR;
    prevKeypointsLeft = keyPointVectorLeft;
    prevKeypointsRight = keyPointVectorRight;
    prevDescriptorLeft = descriptorLeft;
    prevDescriptorRight = descriptorRight;

    std::cout << "Overall mean time complete: " << completeTimeAcc/iterations << "ms." << std::endl;
    std::cout << "Overall mean time for total feature detection: " << detectionTimeAcc/iterations << "ms." << std::endl;
    std::cout << "Overall mean time for total feature description: " << descriptionTimeAcc/iterations << "ms." << std::endl;
    std::cout << "Overall mean time for total image matching: " << matchingTimeAcc/iterations << "ms." << std::endl;
}

std::vector<cv::DMatch>  ImageProcessor::bruteForceMatches(cv::Mat currentDescriptor, cv::Mat prevDescriptor){
    std::vector<cv::DMatch> matches;
    cv::Ptr<cv::BFMatcher> bfMatcher = cv::BFMatcher::create();
    bfMatcher->match(prevDescriptor, currentDescriptor, matches);
    return matches;
}

std::vector<cv::DMatch> ImageProcessor::flann(cv::Mat currentDescriptor, cv::Mat prevDescriptor){
    std::vector<cv::DMatch> matches;
    cv::Ptr<cv::FlannBasedMatcher> flann = cv::FlannBasedMatcher::create();
    flann->match(prevDescriptor, currentDescriptor, matches);
    return matches;
}
