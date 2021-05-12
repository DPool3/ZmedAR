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

    //increase iterations for time calculation
    iterations++;

    //choose methods for detection, description and matching
    int selectedFeatureDetector = 1;
    int selectedFeatureDescriptor = 1;
    int selectedFeatureMatching = 1;

    //read in used image set to use variables for rectification and remapping
    cv::Mat Left_Stereo_Map1 = imageSet.getLeftStereoMap1();
    cv::Mat Left_Stereo_Map2 = imageSet.getLeftStereoMap2();
    cv::Mat Right_Stereo_Map1 = imageSet.getRightStereoMap1();
    cv::Mat Right_Stereo_Map2 = imageSet.getRightStereoMap2();

    //variables for results
//    cv::Mat grayL, grayR;
    cv::Mat remappedL, remappedR;
    cv::Mat descriptorLeft, descriptorRight;
    std::vector<cv::DMatch> matchesLeft, matchesRight;
    std::vector<cv::KeyPoint> keyPointVectorLeft, keyPointVectorRight;

    //color conversion rgb to gray for faster detection
//    cv::cvtColor(imageLeft, grayL, CV_RGB2GRAY);
//    cv::cvtColor(imageRight, grayR, CV_RGB2GRAY);

    //remap images to remove distortion and to rectify
    cv::remap(imageLeft, remappedL, Left_Stereo_Map1, Left_Stereo_Map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    cv::remap(imageRight, remappedR, Right_Stereo_Map1, Right_Stereo_Map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

    //1. Feature Detection
    QElapsedTimer detectorTimer;
    detectorTimer.start();

//    std::thread t1detect([&] {keyPointVectorLeft = featureDetectionMethod(remappedL, selectedFeatureDetector);});
//    std::thread t2detect([&] {keyPointVectorRight = featureDetectionMethod(remappedR, selectedFeatureDetector);});
//    t1detect.join();
//    t2detect.join();

    keyPointVectorLeft = featureDetectionMethod(remappedL, selectedFeatureDetector);
    keyPointVectorRight = featureDetectionMethod(remappedR, selectedFeatureDetector);

    double detectionTime = detectorTimer.elapsed();
    detectionTimeAcc = detectionTimeAcc + detectionTime;

    //1.1 (Optional) draw matches
    cv::drawKeypoints(remappedL, keyPointVectorLeft, remappedL);
    cv::drawKeypoints(remappedR, keyPointVectorRight, remappedR);
    imageLeft = remappedL;
    imageRight = remappedR;

    //1.2 Feature Description
    QElapsedTimer descriptorTimer;
    descriptorTimer.start();

//    std::thread t1descript([&] {descriptorLeft = featureDescriptionMethod(remappedL, keyPointVectorLeft, selectedFeatureDescriptor);});
//    std::thread t2descript([&] {descriptorRight = featureDescriptionMethod(remappedR, keyPointVectorRight, selectedFeatureDescriptor);});
//    t1descript.join();
//    t2descript.join();

    descriptorLeft = featureDescriptionMethod(remappedL, keyPointVectorLeft, selectedFeatureDescriptor);
    descriptorRight = featureDescriptionMethod(remappedR, keyPointVectorRight, selectedFeatureDescriptor);

    double descriptionTime = descriptorTimer.elapsed();
    descriptionTimeAcc = descriptionTimeAcc + descriptionTime;

    //2. Feature Matching
    QElapsedTimer matchingTimer;
    matchingTimer.start();

    //Only execute if there is a previous image and there are more than 0 matches.
    if((!prevImageLeft.empty() && !prevImageRight.empty()) &&
        !descriptorLeft.empty() && !descriptorRight.empty()){

//        std::thread t1match([&] {matchesLeft = featureMatchingMethod(descriptorLeft, selectedFeatureMatching);});
//        std::thread t2match([&] {matchesRight = featureMatchingMethod(descriptorRight, selectedFeatureMatching);});
//        t1match.join();
//        t2match.join();

        matchesLeft = featureMatchingMethod(descriptorLeft, selectedFeatureMatching);
        matchesRight = featureMatchingMethod(descriptorRight, selectedFeatureMatching);

        std::cout << matchesLeft.size() << " matches in left image" << std::endl;
        std::cout << matchesRight.size() << " matches in right image" << std::endl;
    }
    else{
        std::cerr << "No Features could be detected and therefore no matches could be found." << std::endl;
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

    //Outputs
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

std::vector<cv::KeyPoint> ImageProcessor::featureDetectionMethod(cv::Mat remapped, int selectedFeatureDetection){
    std::vector<cv::KeyPoint> keyPointVector;

    //1. Feature detection for both stereo pair images
    //1=ORB, 2=FAST, 3=BRISK, 4=AKAZE, 5=AGAST, 6=SIFT, 7=SURF
    switch (selectedFeatureDetection) {
    case 1:{
        //ORB - Oriented FAST and Rotated BRIEF
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        orb->detect(remapped, keyPointVector, cv::Mat());

        selectedDetector = "ORB";

        break;
    }
    case 2:{
        //FAST is only a feature detector and has no descriptors to compute.
        cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create();
        fast->detect(remapped, keyPointVector, cv::Mat());

        selectedDetector = "FAST";

        break;
    }
    case 3:{
        //BRISK
        cv::Ptr<cv::BRISK> brisk = cv::BRISK::create();
        brisk->detect(remapped, keyPointVector, cv::Mat());

        selectedDetector = "BRISK";

        break;
    }
    case 4:{
        //AKAZE
        cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
        akaze->detect(remapped, keyPointVector, cv::Mat());

        selectedDetector = "AKAZE";

        break;
    }
    case 5:{
        //AGAST
        cv::Ptr<cv::AgastFeatureDetector> agast = cv::AgastFeatureDetector::create();
        agast->detect(remapped, keyPointVector, cv::Mat());

        selectedDetector = "AGAST";

        break;
    }
    case 6:{
        cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create();
        sift->detect(remapped, keyPointVector, cv::Mat());

        selectedDetector = "SIFT";

        break;
    }
    case 7:{
        cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create();
        surf->detect(remapped, keyPointVector, cv::Mat());

        selectedDetector = "SURF";

        break;
    }
    default:
        break;
    }

    return keyPointVector;
}

cv::Mat ImageProcessor::featureDescriptionMethod(cv::Mat remapped, std::vector<cv::KeyPoint> keyPointVector, int selectedFeatureDescription){
    //1.2 Feature description for both images
    //1=ORB, 2=BRIEF, 3=SIFT, 4=SURF, 5=BRISK, 6=AKAZE
    cv::Mat descriptor;

    switch (selectedFeatureDescription) {
    case 1:{
        //ORB - Oriented FAST and Rotated BRIEF
        cv::Ptr<cv::ORB> orb = cv::ORB::create();
        orb->compute(remapped, keyPointVector, descriptor);

        selectedDescriptor = "ORB";

        break;
    }
    case 2:{
        //BRIEF (Binary Robust Independent Elementary) -> non free
        cv::Ptr<cv::xfeatures2d::BriefDescriptorExtractor> brief = cv::xfeatures2d::BriefDescriptorExtractor::create();
        brief->compute(remapped, keyPointVector, descriptor);

        selectedDescriptor = "BRIEF";

        break;
    }
    case 3:{
        //SIFT (Scale-invariant feature transform)
        cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create();
        sift->compute(remapped, keyPointVector, descriptor);

        selectedDescriptor = "SIFT";

        break;
    }
    case 4:{
        //SURF (Speeded Up Robust Features)
        cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create();
        surf->compute(remapped, keyPointVector, descriptor);

        selectedDescriptor = "SURF";

        break;
    }
    case 5:{
        //BRISK (Binary Robust Invariant Scalable Keypoints)
        cv::Ptr<cv::BRISK> brisk = cv::BRISK::create();
        brisk->compute(remapped, keyPointVector, descriptor);

        selectedDescriptor = "BRISK";

        break;
    }
    case 6:{
        //Can Only be used with KAZE or AKAZE Keypoints
        cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
        akaze->compute(remapped, keyPointVector, descriptor);

        selectedDescriptor = "AKAZE";

        break;
    }
    default:
        break;
    }

    return descriptor;
}

std::vector<cv::DMatch> ImageProcessor::featureMatchingMethod(cv::Mat descriptor, int SelectedFeatureMatching){
    std::vector<cv::DMatch> matches;

    switch (SelectedFeatureMatching) {
    case 1:{
        //Brute-Force Matcher
        matches = bruteForceMatches(descriptor, prevDescriptorLeft);

        selectedMatcher = "Brute-Force";

        break;
    }
    case 2:{
        //FLANN (Fast Library for Approximate Nearest Neighbors) Matcher
        matches = flann(descriptor, prevDescriptorLeft);

        selectedMatcher = "FLANN";

        break;
    }
    default:
        break;
    }

    return matches;
}
