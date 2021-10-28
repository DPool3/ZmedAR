#include "imageprocessor.h"

/**
 * @brief ImageProcessor::ImageProcessor lädt das ImageSet und weitere Matrizen,
 * die recht lange brauchen direkt zu Beginn.
 */
ImageProcessor::ImageProcessor()
{
    imageSet = ImageSet(selectedImageSet);
    leftStereoMap1 = imageSet.getLeftStereoMap1();
    leftStereoMap2 = imageSet.getLeftStereoMap2();
    rightStereoMap1 = imageSet.getRightStereoMap1();
    rightStereoMap2 = imageSet.getRightStereoMap2();
}

/**
 * @brief ImageProcessor::ImageProcessor lädt das ImageSet , weitere Matrizen
 * und speichert die Methoden für das Kameratracking direkt zu Beginn.
 * @param detector
 * @param descriptor
 * @param matcher
 */
ImageProcessor::ImageProcessor(int detector, int descriptor, int matcher)
{
    this->selectedFeatureDetector = detector;
    this->selectedFeatureDescriptor = descriptor;
    this->selectedFeatureMatching = matcher;
    imageSet = ImageSet(selectedImageSet);
    leftStereoMap1 = imageSet.getLeftStereoMap1();
    leftStereoMap2 = imageSet.getLeftStereoMap2();
    rightStereoMap1 = imageSet.getRightStereoMap1();
    rightStereoMap2 = imageSet.getRightStereoMap2();
}

/**
 * @brief ImageProcessor::prepImageForDisplay verarbeitet die cv::Mat
 * Bilder zu QImages, die später in Qt dargestellt werden können.
 * @param image
 * @return das verarbeitete Bild.
 */
QImage ImageProcessor::prepImageForDisplay(cv::Mat& image)
{
    QImage returnImage;

    //Resize Images
    cv::resize(image, image, cv::Size(480, 320), 0, 0);

    //Change to RGB format & save it in global Mat
    cv::cvtColor(image, image, cv::COLOR_BGR2RGB);

    //Convert to QImage
    QImage qimg((const unsigned char*) image.data, image.cols, image.rows, QImage::Format_RGB888);
    returnImage = qimg;

    return returnImage;
}

/**
 * @brief ImageProcessor::generateAnaglyphImage generiert das Anaglyph 3D Bild,
 * indem es die Channels der Beiden Bilder spaltet und zu einem Bild zusammen
 * setzt. Dieses Bild wird zu einem QImage verarbeitet.
 * @param imageLeft
 * @param imageRight
 * @return Anaglyph 3D Bild als QImage.
 */
QImage ImageProcessor::generateAnaglyphImage(cv::Mat imageLeft, cv::Mat imageRight)
{
    //remap images
    cv::Mat remappedLeft, remappedRight;
    cv::remap(imageLeft, remappedLeft, leftStereoMap1, leftStereoMap2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, cv::Scalar());
    cv::remap(imageRight, remappedRight, rightStereoMap1, rightStereoMap2, cv::INTER_LANCZOS4, cv::BORDER_CONSTANT, cv::Scalar());

    //resize
    cv::resize(remappedLeft, remappedLeft, cv::Size(480,320), 0, 0);
    cv::resize(remappedRight, remappedRight, cv::Size(480,320), 0, 0);

    //generate anaglyph 3D image
    cv::Mat remappedSplitLeft[3], remappedSplitRight[3], anaglyph3DImage;
    std::vector<cv::Mat> anaglyph_channels;

    cv::split(remappedLeft, remappedSplitLeft);
    cv::split(remappedRight, remappedSplitRight);

    anaglyph_channels.push_back(remappedSplitLeft[0]);
    anaglyph_channels.push_back(remappedSplitRight[1]);
    anaglyph_channels.push_back(remappedSplitRight[2]);

    cv::merge(anaglyph_channels, anaglyph3DImage);

    QImage anaglyph3DQImage((const unsigned char*) anaglyph3DImage.data, anaglyph3DImage.cols, anaglyph3DImage.rows, QImage::Format_RGB888);

    return anaglyph3DQImage;
}

/**
 * @brief ImageProcessor::cannyEdgeOnImagePair erzeugt ein CannyEdgeBildPaar.
 * @param imageLeft
 * @param imageRight
 */
void ImageProcessor::cannyEdgeOnImagePair(cv::Mat & imageLeft, cv::Mat &imageRight)
{
    cv::Mat imgLeftGray, imgRightGray;
    cv::Mat imgBlurLeft, imgBlurRight;
    cv::Mat detectedEdgesLeft, detectedEdgesRight;
    int lowThreshhold = 50;
    int highThreshold = 110;
    const int kernel_size = 3;

    cv::Mat tempLeft = imageLeft;
    cv::Mat tempRight = imageRight;

    cv::cvtColor(tempLeft, imgLeftGray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(tempRight, imgRightGray, cv::COLOR_BGR2GRAY);

    cv::GaussianBlur(imgLeftGray,   // input image
          imgBlurLeft,              // output image
          cv::Size(3, 3),           // smoothing window width and height in pixels
          2.5);                     // sigma value, determines how much the image will be blurred

    cv::GaussianBlur(imgRightGray, imgBlurRight, cv::Size(3,3), 2.5);

    cv::Canny(imgBlurLeft, detectedEdgesLeft, lowThreshhold, highThreshold, kernel_size);
    cv::Canny(imgBlurRight, detectedEdgesRight, lowThreshhold, highThreshold, kernel_size);

    imageLeft = detectedEdgesLeft;
    imageRight = detectedEdgesRight;
}

/**
 * @brief ImageProcessor::stereoVisualOdometry führt die stereovisuelle Odometrie
 * für ein Bildpaar durch.
 * @param currImageLeft
 * @param currImageRight
 */
void ImageProcessor::stereoVisualOdometry(cv::Mat currImageLeft, cv::Mat currImageRight)
{
    QElapsedTimer totalImageMatchingTimer;
    totalImageMatchingTimer.start();

    //increase iterations for time calculation
    iterations++;

    //choose methods for detection, description and matching
    //1=ORB, 2=FAST, 3=BRISK, 4=SIFT, 5=SURF
    int selectedFeatureDetector = this->selectedFeatureDetector;
    //1=ORB, 2=BRIEF, 3=BRISK, 4=SIFT, 5=SURF
    int selectedFeatureDescriptor = this->selectedFeatureDescriptor;
    //1=Brute Force, 2=FLANN (Fast Library for Approximate Nearest Neighbors)
    int selectedFeatureMatching = this->selectedFeatureMatching;

    //set variables for coordinates to zero
    cv::Mat rVec_Left, tVec_Left, rVec_Right, tVec_Right;
    rVec_Left = cv::Mat(1, 3, CV_64F, double(0));
    tVec_Left = cv::Mat(1, 3, CV_64F, double(0));
    rVec_Right = cv::Mat(1, 3, CV_64F, double(0));
    tVec_Right = cv::Mat(1, 3, CV_64F, double(0));
    leftCameraWorldTranslation = cv::Mat(1, 3, CV_64F, double(0));
    rightCameraWorldTranslation = cv::Mat(1, 3, CV_64F, double(0));

    //read camera matrix and dist coef to find essential matrix later
    cv::Mat camMatLeft = imageSet.getCamL();
    cv::Mat camMatRight = imageSet.getCamR();

    //color conversion rgb to gray for faster detection
    cv::Mat grayL, grayR;
    cv::cvtColor(currImageLeft, grayL, cv::COLOR_RGB2GRAY);
    cv::cvtColor(currImageRight, grayR, cv::COLOR_RGB2GRAY);

    //----------------------------------------------------
    //1. remap images to remove distortion and to rectify.
    //----------------------------------------------------
    cv::Mat remappedL, remappedR;
    cv::remap(grayL, remappedL, leftStereoMap1, leftStereoMap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    cv::remap(grayR, remappedR, rightStereoMap1, rightStereoMap2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

    //-----------------------------------
    //2. Feature detection & description.
    //-----------------------------------
    std::vector<cv::KeyPoint> keyPointVectorLeft, keyPointVectorRight;
    cv::Mat descriptorLeft, descriptorRight;

    detectFeaturesParallel(remappedL, remappedR, keyPointVectorLeft, keyPointVectorRight, selectedFeatureDetector);
    describeFeaturesParallel(remappedL, remappedR, descriptorLeft, descriptorRight, keyPointVectorLeft, keyPointVectorRight, selectedFeatureDescriptor);

    //--------------------------------------------------------------------------------------
    //3. Feature matching for left and right image. If possible also for consecutive images.
    //--------------------------------------------------------------------------------------
    std::vector<cv::DMatch> matchesLeftRight;

    //------------------------------------
    //Step One: Match Left and Right Image
    //------------------------------------
    QElapsedTimer matchingTimer;
    matchingTimer.start();

    matchesLeftRight = featureMatchingMethod(descriptorLeft, descriptorRight, selectedFeatureDescriptor, selectedFeatureMatching);
    std::cout << matchesLeftRight.size() << " matches between left and right" << std::endl;

    double matchingTime = matchingTimer.elapsed();
    matchingTimeAcc = matchingTimeAcc + matchingTime;
    numberOfMatchesAcc = numberOfMatchesAcc + matchesLeftRight.size();

    //-----------------------------------------------------------------------------------------------------
    //Step Two: Select key points and descriptors from matches of left and right images
    //key points are used for triangulation and descriptors are used for temporal matching (t matching t-1)
    //-----------------------------------------------------------------------------------------------------
    std::vector<cv::KeyPoint> matchedKeyPointsLeft, matchedKeyPointsRight;
    cv::Mat matchedDescriptorsLeft, matchedDescriptorsRight;

    for(std::vector<cv::DMatch>::size_type i = 0; i < matchesLeftRight.size(); i++){
        //keypoints
        matchedKeyPointsLeft.push_back(keyPointVectorLeft[matchesLeftRight[i].queryIdx]);
        matchedKeyPointsRight.push_back(keyPointVectorRight[matchesLeftRight[i].trainIdx]);
        //descriptors
        matchedDescriptorsLeft.push_back(descriptorLeft.row(matchesLeftRight[i].queryIdx));
        matchedDescriptorsRight.push_back(descriptorRight.row(matchesLeftRight[i].trainIdx));
    }

    try{
        //For the following step there have to be previous images
        if(iterations == 1)
            throw std::invalid_argument("First image.");

        //---------------------------------------------------------------------------------------------
        //Step Three: Temporal matching. Match Left image at t-1 and t. Match Right image at t-1 and t.
        //than select key points of all images.
        //---------------------------------------------------------------------------------------------
        std::vector<cv::KeyPoint> matchedKeyPointsLeft_tMinus1, matchedKeyPointsLeft_t, matchedKeyPointsRight_tMinus1, matchedKeyPointsRight_t;

        if(prevDescriptorLeft.rows <= 5 || prevDescriptorRight.rows <= 5 || matchedDescriptorsLeft.rows <= 5 || matchedDescriptorsRight.rows <= 5){
            throw std::invalid_argument("Not enough matches after matching of current left and current right image");
        }

        QElapsedTimer temporalMatchingTimer;
        temporalMatchingTimer.start();

        std::vector<cv::DMatch> matchesLeftConsecutive, matchesRightConsecutive;
        matchesLeftConsecutive = featureMatchingMethod(prevDescriptorLeft, matchedDescriptorsLeft, selectedFeatureDescriptor, selectedFeatureMatching);
        matchesRightConsecutive = featureMatchingMethod(prevDescriptorRight, matchedDescriptorsRight, selectedFeatureDescriptor, selectedFeatureMatching);

        float temporalMatchingTime = temporalMatchingTimer.elapsed();
        temporalMatchingTimeAcc = temporalMatchingTimeAcc + temporalMatchingTime;
        numberOfTemporalMatchesAcc = numberOfTemporalMatchesAcc + ((matchesLeftConsecutive.size() + matchesRightConsecutive.size())/2);

        std::cout << matchesLeftConsecutive.size() << " matches in left image in t & t-1" << std::endl;
        std::cout << matchesRightConsecutive.size() << " matches in right image in t & t-1" << std::endl;

        if(matchesLeftConsecutive.size() <= 5 || matchesRightConsecutive.size() <= 5){
            throw std::invalid_argument("Not enough matches after temporal matching.");
        }

        for(std::vector<cv::DMatch>::size_type i = 0; i < matchesLeftConsecutive.size(); i++){
            //select key points for left images t-1 and t
            matchedKeyPointsLeft_tMinus1.push_back(prevKeypointsLeft[matchesLeftConsecutive[i].queryIdx]);
            matchedKeyPointsLeft_t.push_back(matchedKeyPointsLeft[matchesLeftConsecutive[i].trainIdx]);
        }

        for(std::vector<cv::DMatch>::size_type i = 0; i < matchesRightConsecutive.size(); i++){
            //select key points for right images t-1 and t
            matchedKeyPointsRight_tMinus1.push_back(prevKeypointsRight[matchesRightConsecutive[i].queryIdx]);
            matchedKeyPointsRight_t.push_back(matchedKeyPointsRight[matchesRightConsecutive[i].trainIdx]);
        }

        //-----------------------------------------------------------------------------------------------------------
        //4. find essential matrix, recover pose and get triangulated points as cartesian coordinates for left images
        //-----------------------------------------------------------------------------------------------------------
        int inliers = 0;
        int outliers = 0;
        std::vector<cv::Point3f> points3DIn_Left;
        std::vector<cv::Point2f> prevMatchesIn_Left, currMatchesIn_Left;
        calc3DPointsOfInliers(matchedKeyPointsLeft_tMinus1, matchedKeyPointsLeft_t, camMatLeft, prevMatchesIn_Left, currMatchesIn_Left, points3DIn_Left);

        inliers = inliers + points3DIn_Left.size();
        outliers = outliers + (matchedKeyPointsLeft_t.size() - points3DIn_Left.size());

        //------------------------------------------
        //5. Calculate rotation and translation left
        //------------------------------------------
        if(prevMatchesIn_Left.size() > 5 || currMatchesIn_Left.size() > 5){
            cv::Mat empyDistCoeffL, inliersLeft;
            //solvePnPRansac for left to get rotation vector and translation vector
            cv::solvePnPRansac(points3DIn_Left, currMatchesIn_Left, camMatLeft, empyDistCoeffL, rVec_Left, tVec_Left, false, 100, 8.0, 0.99, inliersLeft);

            //---------------------------------------------------------------------------------------------
            //6. Determining the motion of the camera left in world coordinates (cam_world = -inverse(R)*t)
            //---------------------------------------------------------------------------------------------
            //calculate world coordinates for left camera
            calcWorldCoords(rVec_Left, tVec_Left);
            leftCameraWorldTranslation = tVec_Left;

            ransacCounterLeft++;
            std::cout << "ransacsLeft: " << ransacCounterLeft << std::endl;
        }
        else{
            leftCameraWorldTranslation = cv::Mat(1, 3, CV_64F, double(0));
            std::cout << leftCameraWorldTranslation << std::endl;
        }

        //------------------------------------------------------------------------------------------------------------
        //4. find essential matrix, recover pose and get triangulated points as cartesian coordinates for right images
        //------------------------------------------------------------------------------------------------------------
        std::vector<cv::Point3f> points3DIn_Right;
        std::vector<cv::Point2f> prevMatchesIn_Right, currMatchesIn_Right;
        calc3DPointsOfInliers(matchedKeyPointsRight_tMinus1, matchedKeyPointsRight_t, camMatRight, prevMatchesIn_Right, currMatchesIn_Right, points3DIn_Right);

        inliers = inliers + points3DIn_Right.size();
        outliers = outliers + (matchedKeyPointsRight_t.size() - points3DIn_Right.size());

        //-------------------------------------------
        //5. Calculate rotation and translation right
        //-------------------------------------------
        if(prevMatchesIn_Right.size() > 5 || currMatchesIn_Right.size() > 5){
            cv::Mat emptyDistCoeffR, inliersRight;
            //solvePnPRansac for right to get rotation vector and translation vector
            cv::solvePnPRansac(points3DIn_Right, currMatchesIn_Right, camMatRight, emptyDistCoeffR, rVec_Right, tVec_Right, false, 100, 8.0, 0.99, inliersRight);

            //----------------------------------------------------------------------------------------------
            //6. Determining the motion of the camera right in world coordinates (cam_world = -inverse(R)*t)
            //----------------------------------------------------------------------------------------------
            //calculate world coordinates for right camera
            calcWorldCoords(rVec_Right, tVec_Right);
            rightCameraWorldTranslation = tVec_Right;

            ransacCounterRight++;
            std::cout << "ransacsRight: " << ransacCounterRight << std::endl;
        }
        else{
            rightCameraWorldTranslation = cv::Mat(1, 3, CV_64F, double(0));
            std::cout << rightCameraWorldTranslation << std::endl;
        }

        numberOfInliersAcc = numberOfInliersAcc + (inliers / 2);
        numberOfOutliersAcc = numberOfOutliersAcc + (outliers / 2);

        //---------------------------
        //7. Save coordinates in File
        //---------------------------
        //addLineToFile(leftCameraWorldCoords, rightCameraWorldCoords);
        addLineToFile(leftCameraWorldTranslation, rightCameraWorldTranslation);

    }catch(const std::exception& e){
        std::cout << e.what() << std::endl;
        prevKeypointsLeft = matchedKeyPointsLeft;
        prevKeypointsRight = matchedKeyPointsRight;
        prevDescriptorLeft = matchedDescriptorsRight;
        prevDescriptorRight = matchedDescriptorsRight;
        addLineToFile(leftCameraWorldTranslation, rightCameraWorldTranslation);
    }

    double completeTime = totalImageMatchingTimer.elapsed();
    completeTimeAcc = completeTimeAcc + completeTime;

    //Set the current values as previous
    prevImageLeft = remappedL;
    prevImageRight = remappedR;
    prevMatchesLeftRight = matchesLeftRight;

    //Outputs
    std::cout << "Overall mean time complete: " << completeTimeAcc/iterations << "ms." << std::endl;
    std::cout << "Overall mean time for total feature detection: " << detectionTimeAcc/iterations << "ms." << std::endl;
    std::cout << "Overall mean time for total feature description: " << descriptionTimeAcc/iterations << "ms." << std::endl;
    std::cout << "Overall mean time for image matching left and right: " << matchingTimeAcc/iterations << "ms." << std::endl;
    std::cout << "Overall mean time for temporal matching: " << temporalMatchingTimeAcc/iterations << "ms." << std::endl;
    std::cout << "Overall average number of matches: " << numberOfMatchesAcc/iterations << "." << std::endl;
    std::cout << "Overall average number of found key points: " << numberOfKeyPointsAcc/iterations << "." << std::endl;
    std::cout << "Overall average number of temporal matches: " << numberOfTemporalMatchesAcc/iterations << "." << std::endl;
    std::cout << "Overall average number of outliers: " << numberOfOutliersAcc/iterations << "." << std::endl;
    std::cout << "Overall average number of inliers: " << numberOfInliersAcc/iterations << "." << std::endl;
    std::cout << "Number of calculated wordl coordinates (left): " << ransacCounterLeft << "." << std::endl;
    std::cout << "Number of calculated wordl coordinates (right): " << ransacCounterRight << "." << std::endl;
}

/**
 * @brief ImageProcessor::detectFeaturesParallel ruft eine FeatureDetection Methode
 * in zwei threads auf.
 * @param remappedL
 * @param remappedR
 * @param keyPointVectorLeft
 * @param keyPointVectorRight
 * @param selectedFeatureDetector
 */
void ImageProcessor::detectFeaturesParallel(cv::Mat remappedL, cv::Mat remappedR, std::vector<cv::KeyPoint> &keyPointVectorLeft, std::vector<cv::KeyPoint> &keyPointVectorRight, int selectedFeatureDetector){
    //error handling in general

    QElapsedTimer detectorTimer;
    detectorTimer.start();

    std::thread t1detect([&] {keyPointVectorLeft = featureDetectionMethod(remappedL, selectedFeatureDetector);});
    std::thread t2detect([&] {keyPointVectorRight = featureDetectionMethod(remappedR, selectedFeatureDetector);});
    t1detect.join();
    t2detect.join();

    double detectionTime = detectorTimer.elapsed();
    detectionTimeAcc = detectionTimeAcc + detectionTime;
    numberOfKeyPointsAcc = numberOfKeyPointsAcc + ((keyPointVectorLeft.size() + keyPointVectorRight.size())/2);

    std::cout << "LeftKeyPointVector: " << keyPointVectorLeft.size() << std::endl;
    std::cout << "RightKeyPointVector: " << keyPointVectorRight.size() << std::endl;
}

/**
 * @brief ImageProcessor::describeFeaturesParallel ruft eine FeatureDescription Methode
 * in zwei Threads auf.
 * @param remappedL
 * @param remappedR
 * @param descriptorLeft
 * @param descriptorRight
 * @param keyPointVectorLeft
 * @param keyPointVectorRight
 * @param selectedFeatureDescription
 */
void ImageProcessor::describeFeaturesParallel(cv::Mat remappedL, cv::Mat remappedR, cv::Mat &descriptorLeft, cv::Mat &descriptorRight, std::vector<cv::KeyPoint> keyPointVectorLeft, std::vector<cv::KeyPoint> keyPointVectorRight, int selectedFeatureDescription)
{
    //error handling in general

    QElapsedTimer descriptorTimer;
    descriptorTimer.start();

    std::thread t1descript([&] {descriptorLeft = featureDescriptionMethod(remappedL, keyPointVectorLeft, selectedFeatureDescription);});
    std::thread t2descript([&] {descriptorRight = featureDescriptionMethod(remappedR, keyPointVectorRight, selectedFeatureDescription);});
    t1descript.join();
    t2descript.join();

    double descriptionTime = descriptorTimer.elapsed();
    descriptionTimeAcc = descriptionTimeAcc + descriptionTime;
}

/**
 * @brief ImageProcessor::calc3DPointsOfInliers verwendet die vorherigen und aktuellen
 * Matches um die Essenzielle Matrix mittels findEssentialMat zu berechnen. Diese
 * wird verwendet um im nächsten Schritt recoverPose um die 3D Punkte in homogenen
 * Koordinaten zu erhalten und gleichzeitig die outlier zu filtern. Diese Punkte
 * werden in kartesische Koordinaten überführt, bevor erneut die inlier mittels
 * mask herausgesucht werden.
 * @param previousMatches
 * @param currentMatches
 * @param camMat
 * @param prevMatchesIn
 * @param currMatchesIn
 * @param points3DInliers
 */
void ImageProcessor::calc3DPointsOfInliers(std::vector<cv::KeyPoint> previousMatches,
                                           std::vector<cv::KeyPoint> currentMatches,
                                           cv::Mat camMat,
                                           std::vector<cv::Point2f> &prevMatchesIn,
                                           std::vector<cv::Point2f> &currMatchesIn,
                                           std::vector<cv::Point3f> &points3DInliers)
{
    std::vector<cv::Point2f> keyPointsCurrent, keyPointsPrevious;
    std::vector<cv::Point2f> currMatchesGood, prevMatchesGood;
    cv::KeyPoint::convert(previousMatches, keyPointsPrevious, std::vector<int>());
    cv::KeyPoint::convert(currentMatches, keyPointsCurrent, std::vector<int>());

    //filter only good
    for(int i = 0; i < (int)keyPointsPrevious.size(); i++){
       prevMatchesGood.push_back(keyPointsPrevious[i]);
       currMatchesGood.push_back(keyPointsCurrent[i]);
    }

    cv::Mat E, mask;
    E = cv::findEssentialMat(prevMatchesGood, currMatchesGood, camMat, cv::RANSAC, 0.95, 1.0, mask);
    std::cout << "inliers E: " << sum(mask) << std::endl;

    //recover pose
    cv::Mat R, t, points3D_homogeneous;
    int inliers = cv::recoverPose(E, prevMatchesGood, currMatchesGood, camMat, R, t, 100, mask, points3D_homogeneous);
    std::cout << "inliers after recoverPose: " << inliers << std::endl;

    //convert points to cartesian
    cv::Point3f point;
    std::vector<cv::Point3f> points3D;
    for(int i = 0; i < points3D_homogeneous.cols; i++)
    {
      point.x = points3D_homogeneous.at<double>(0, i) / points3D_homogeneous.at<double>(3, i);
      point.y = points3D_homogeneous.at<double>(1, i) / points3D_homogeneous.at<double>(3, i);
      point.z = points3D_homogeneous.at<double>(2, i) / points3D_homogeneous.at<double>(3, i);
      points3D.push_back(point);
    }

    //filter only inliers
    for(int i = 0; i < int(keyPointsPrevious.size()); i++){
      if(mask.at<bool>(i,0) == 1){
        prevMatchesIn.push_back(keyPointsPrevious[i]);
        currMatchesIn.push_back(keyPointsCurrent[i]);
        points3DInliers.push_back(points3D[i]);
      }
    }

}


/**
 * @brief ImageProcessor::calcWorldCoords berechnet die Weltkoordinate
 * aus dem Rotationsvektor und Translationsvektor.
 * @param rvec
 * @param tvec
 */
void ImageProcessor::calcWorldCoords(cv::Mat rvec, cv::Mat& tvec)
{
    cv::Mat R = cv::Mat(3, 3, CV_64F, double(0));;
    cv::Rodrigues(rvec, R);
    R = R.t();
    tvec = -R * tvec;
}

/**
 * @brief ImageProcessor::featureDetectionMethod enthält ein Switch Case, mit allen
 * Feature Detection Methoden.
 * @param remapped
 * @param selectedFeatureDetection
 * @return detectierte Keypoints in einem Vektor.
 */
std::vector<cv::KeyPoint> ImageProcessor::featureDetectionMethod(cv::Mat remapped, int selectedFeatureDetection)
{
    std::vector<cv::KeyPoint> keyPointVector;

    //1. Feature detection for both stereo pair images
    //1=ORB, 2=FAST, 3=BRISK, 4=SIFT, 5=SURF
    switch (selectedFeatureDetection) {
    case 1:{
        //ORB - Oriented FAST and Rotated BRIEF
        cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);
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
        cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create(1000);
        sift->detect(remapped, keyPointVector, cv::Mat());

        selectedDetector = "SIFT";

        break;
    }
    case 5:{
        cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create(400);
        surf->detect(remapped, keyPointVector, cv::Mat());

        selectedDetector = "SURF";

        break;
    }
    default:
        break;
    }

    return keyPointVector;
}

/**
 * @brief ImageProcessor::featureDescriptionMethod enthält ein Switch Case, mit allen
 * Feature Description Methoden.
 * @param remapped
 * @param keyPointVector
 * @param selectedFeatureDescription
 * @return den Deskriptor für die Keypoints.
 */
cv::Mat ImageProcessor::featureDescriptionMethod(cv::Mat remapped, std::vector<cv::KeyPoint> keyPointVector, int selectedFeatureDescription)
{
    //1.2 Feature description for both images
    //1=ORB, 2=BRIEF, 3=BRISK, 4=SIFT, 5=SURF
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
        //BRISK (Binary Robust Invariant Scalable Keypoints)
        cv::Ptr<cv::BRISK> brisk = cv::BRISK::create();
        brisk->compute(remapped, keyPointVector, descriptor);

        selectedDescriptor = "BRISK";

        break;
    }
    case 4:{
        //SIFT (Scale-invariant feature transform)
        cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create();
        sift->compute(remapped, keyPointVector, descriptor);

        selectedDescriptor = "SIFT";

        break;
    }
    case 5:{
        //SURF (Speeded Up Robust Features)
        cv::Ptr<cv::xfeatures2d::SURF> surf = cv::xfeatures2d::SURF::create();
        surf->compute(remapped, keyPointVector, descriptor);

        selectedDescriptor = "SURF";

        break;
    }
    default:
        break;
    }

    return descriptor;
}

/**
 * @brief ImageProcessor::featureMatchingMethod enthält alle Matching Methoden
 * inklusive Auswahl zwischen CrossCheck, NoCheck und DLowe.
 * @param descriptorOne
 * @param descriptorTwo
 * @param selectedFeatureDescription
 * @param SelectedFeatureMatching
 * @return Vektor von Matches.
 */
std::vector<cv::DMatch> ImageProcessor::featureMatchingMethod(cv::Mat descriptorOne, cv::Mat descriptorTwo, int selectedFeatureDescription ,int SelectedFeatureMatching)
{
    std::vector<std::vector<cv::DMatch>> matches;
    std::vector<cv::DMatch> goodMatches;

    //check if number of features in both vectors is equal or greater than number of nearest neighbors in knn match.
    //else knn match will throw an error
    if((descriptorOne.rows < 6) || (descriptorTwo.rows < 6 || descriptorOne.empty() || descriptorTwo.empty())){
        std::cout << "Less than two features found in at least one descriptor." << std::endl;
    }

    if(SelectedFeatureMatching == 1){
        //Brute-Force Matcher
        selectedMatcher = "Brute-Force";

        //specifies the distance measurement to be used. Default cv::NORM_L2
        //int normType = cv::NORM_L2;

        //If it is true, Matcher returns only those matches with value (i,j)
        //such that i-th descriptor in set A has j-th descriptor in set B as the best match and vice-versa.
        //That is, the two features in both sets should match each other. It provides consistent result,
        //and is a good alternative to ratio test proposed by D.Lowe in SIFT paper.
        //false by default.
        //https://docs.opencv.org/master/dc/dc3/tutorial_py_matcher.html
        int crossCheck = true;

        if(selectedFeatureDescription < 4){
            //https://docs.opencv.org/2.4/modules/features2d/doc/common_interfaces_of_descriptor_matchers.html
            //best values for orb, brief or brisk descriptors
            cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_HAMMING, crossCheck);

            if(crossCheck){
                matcher->knnMatch(descriptorOne, descriptorTwo, matches, 1);

                for(int i = 0; i < (int)matches.size(); i++)
                    if(!(matches[i].empty()))
                        goodMatches.push_back(matches[i][0]);
            }
            else{
                matcher->knnMatch(descriptorOne, descriptorTwo, matches, 2);

                for(size_t i = 0; i < matches.size(); i++)
                    if(!(matches[i].empty()))
                        goodMatches.push_back(matches[i][0]);
            }

        }
        else{
            //use default values and knnMatch for sift or surf
            cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_L2, crossCheck);

            if(crossCheck){
                matcher->knnMatch(descriptorOne, descriptorTwo, matches, 1);

                std::sort(matches.begin(), matches.end());

                for(size_t i = 0; i < matches.size(); i++)
                    if(!(matches[i].empty()))
                        goodMatches.push_back(matches[i][0]);
            }
            else{
                matcher->knnMatch(descriptorOne, descriptorTwo, matches, 2);

                std::sort(matches.begin(), matches.end());

                for(size_t i = 0; i < matches.size(); i++)
                    if(!(matches[i].empty()))
                        goodMatches.push_back(matches[i][0]);
            }

        }

    }
    else{
        //FLANN (Fast Library for Approximate Nearest Neighbors) Matcher
        selectedMatcher = "FLANN";

        if(selectedFeatureDescription < 4){
            //Flann parameter. algorithm = flann indexed lsh, table_number = 12, key_size = 20, multi_probe_level = 1   #
            cv::FlannBasedMatcher matcher (new cv::flann::LshIndexParams(12,20,2));
            matcher.knnMatch(descriptorOne, descriptorTwo, matches, 2);

            //apply ratio test by D.Lowe
            for(size_t i = 0; i < matches.size(); i++)
                if(!(matches[i].empty()) && (matches[i][0].distance < 0.75 * matches[i][1].distance))
                    goodMatches.push_back(matches[i][0]);
        }
        else{
            //Flann parameter. algorithm = kd tree, trees = 4
            cv::FlannBasedMatcher matcher (new cv::flann::KDTreeIndexParams(5));
            matcher.knnMatch(descriptorOne, descriptorTwo, matches, 2);

            //apply ratio test by D.Lowe
            for(size_t i = 0; i < matches.size(); i++)
                if(!(matches[i].empty()) && (matches[i][0].distance < 0.75 * matches[i][1].distance))
                    goodMatches.push_back(matches[i][0]);
        }
    }

    return goodMatches;
}

/**
 * @brief ImageProcessor::createNewTrackingFile erstellt eine neue Trackingdatei
 * unter dem angegebenen Pfad.
 * !WICHTIG! Dieser Pfad muss bei Wechsel des Rechners angepasst werden.
 */
void ImageProcessor::createNewTrackingFile()
{
    std::string fileName = "VO_" + DirectoryManager().getCurrentDateAsString();
    std::string fileType = ".txt";
    this->trackingFile.open("/home/daniel/ZAR/trackingFiles/" + fileName + fileType, std::ios_base::app);
}

/**
 * @brief ImageProcessor::closeTrackingFile schließt die Trackingdatei.
 */
void ImageProcessor::closeTrackingFile()
{
    if(trackingFile.is_open())
        trackingFile.close();
}

/**
 * @brief ImageProcessor::addLineToFile fügt eine Zweile mit Translations- und Rotationsvektor
 * der Trackingdatei hinzu.
 * @param tVecLeft
 * @param tVecRight
 */
void ImageProcessor::addLineToFile(cv::Mat tVecLeft, cv::Mat tVecRight)
{
    this->trackingFile << "TL, " << std::setprecision(3) << tVecLeft.at<double>(0,0) * 0.001 << ", " << std::setprecision(3) << tVecLeft.at<double>(0,1) * -0.001 << ", " << std::setprecision(3) << tVecLeft.at<double>(0,2) * 0.001 <<
                          ", TR, " << std::setprecision(3) << tVecRight.at<double>(0,0) * 0.001 << ", " << std::setprecision(3) << tVecRight.at<double>(0,1) * -0.001 << ", " << std::setprecision(3) << tVecRight.at<double>(0,2) * 0.001 << std::endl;
}
