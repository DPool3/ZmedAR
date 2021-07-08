#include "imageprocessor.h"

ImageProcessor::ImageProcessor()
{
    imageSet = ImageSet("/home/daniel/ZAR/ImageSets/22-04-2021-08-15-15");
}

ImageProcessor::ImageProcessor(int detector, int descriptor, int matcher)
{
    this->selectedFeatureDetector = detector;
    this->selectedFeatureDescriptor = descriptor;
    this->selectedFeatureMatching = matcher;
    imageSet = ImageSet("/home/daniel/ZAR/ImageSets/22-04-2021-08-15-15");
}

QImage ImageProcessor::prepImageForDisplay(cv::Mat& image){
    QImage returnImage;

    //Resize Images
    cv::resize(image, image, cv::Size(480, 320), 0, 0);

    //Change to RGB format & save it in global Mat
    cv::cvtColor(image, image, CV_BGR2RGB);

    //Convert to QImage
    QImage qimg((const unsigned char*) image.data, image.cols, image.rows, QImage::Format_RGB888);
    returnImage = qimg;

    return returnImage;
}

void ImageProcessor::cannyEdgeOnImagePair(cv::Mat & imageLeft, cv::Mat &imageRight){
    cv::Mat imgLeftGray, imgRightGray;
    cv::Mat imgBlurLeft, imgBlurRight;
    cv::Mat detectedEdgesLeft, detectedEdgesRight;
    int lowThreshhold = 50;
    int highThreshold = 110;
    const int kernel_size = 3;

    cv::Mat tempLeft = imageLeft;
    cv::Mat tempRight = imageRight;

    cv::cvtColor(tempLeft, imgLeftGray, CV_BGR2GRAY);
    cv::cvtColor(tempRight, imgRightGray, CV_BGR2GRAY);

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

void ImageProcessor::stereoVisualOdometry(cv::Mat imageLeft, cv::Mat imageRight){
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

    //read used image set to use variables for rectification and remapping
    cv::Mat Left_Stereo_Map1 = imageSet.getLeftStereoMap1();
    cv::Mat Left_Stereo_Map2 = imageSet.getLeftStereoMap2();
    cv::Mat Right_Stereo_Map1 = imageSet.getRightStereoMap1();
    cv::Mat Right_Stereo_Map2 = imageSet.getRightStereoMap2();

    //read camera matix to find essential matrix later
    cv::Mat camMatLeft = imageSet.getCamL();
    cv::Mat camMatRight = imageSet.getCamR();

    //color conversion rgb to gray for faster detection
    cv::Mat grayL, grayR;
    cv::cvtColor(imageLeft, grayL, CV_RGB2GRAY);
    cv::cvtColor(imageRight, grayR, CV_RGB2GRAY);

    //----------------------------------------------------
    //1. remap images to remove distortion and to rectify.
    //----------------------------------------------------
    cv::Mat remappedL, remappedR;
    cv::remap(grayL, remappedL, Left_Stereo_Map1, Left_Stereo_Map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    cv::remap(grayR, remappedR, Right_Stereo_Map1, Right_Stereo_Map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

    //-----------------------------------
    //2. Feature detection & description.
    //-----------------------------------
    std::vector<cv::KeyPoint> keyPointVectorLeft, keyPointVectorRight;
    cv::Mat descriptorLeft, descriptorRight;

    detectFeatures(remappedL, remappedR, keyPointVectorLeft, keyPointVectorRight, selectedFeatureDetector);
    describeFeatures(remappedL, remappedR, descriptorLeft, descriptorRight, keyPointVectorLeft, keyPointVectorRight, selectedFeatureDescriptor);

    //--------------------------------------------------------------------------------------
    //3. Feature matching for left and right image. If possible also for consecutive images.
    //--------------------------------------------------------------------------------------
    std::vector<cv::KeyPoint> currentMatchesLeft, previousMatchesLeft;
    std::vector<cv::KeyPoint> currentMatchesRight, previousMatchesRight;
    std::vector<cv::DMatch> matchesLeftRight;

    //matches are searched between left and right as well as previous to current
    //this way we can find matches between all 4 images. This way we can guarantee a possible pose recovery
    featureMatching(descriptorLeft, descriptorRight,
                    keyPointVectorLeft, keyPointVectorRight,
                    selectedFeatureDescriptor, selectedFeatureMatching,
                    matchesLeftRight,
                    currentMatchesLeft, previousMatchesLeft,
                    currentMatchesRight, previousMatchesRight);

    if(iterations >= 1 && (!currentMatchesLeft.empty() && !previousMatchesLeft.empty() && !currentMatchesRight.empty() && !previousMatchesRight.empty())){

        //----------------------------------------------------------------------------------------
        //find essential matrix, recover pose and get triangulated points as cartesian coordinates
        //----------------------------------------------------------------------------------------

        //find for left
        cv::Mat E_Left, mask_Left;

        std::vector<cv::Point2f> leftKeyPointsCurrent, leftKeyPointsPrevious;
        cv::KeyPoint::convert(previousMatchesLeft, leftKeyPointsPrevious, std::vector<int>());
        cv::KeyPoint::convert(currentMatchesLeft, leftKeyPointsCurrent, std::vector<int>());

        E_Left = cv::findEssentialMat(leftKeyPointsPrevious, leftKeyPointsCurrent, camMatLeft, cv::RANSAC, 0.999, 1.0, mask_Left);

        //recover pose left
        cv::Mat R_Left, t_Left;
        cv::Mat points3D_homogeneous_Left;
        int inlierPose_Left;
        inlierPose_Left = cv::recoverPose(E_Left, leftKeyPointsPrevious, leftKeyPointsCurrent, camMatLeft, R_Left, t_Left, mask_Left);

        //convert points left to cartesian

        //filter only inliers of left images

        //draw left (optional)

        //find for right
        cv::Mat E_Right, mask_Right;

        std::vector<cv::Point2f> rightKeyPointsCurrent, rightKeyPointsPrevious;
        cv::KeyPoint::convert(previousMatchesRight, rightKeyPointsPrevious, std::vector<int>());
        cv::KeyPoint::convert(currentMatchesRight, rightKeyPointsCurrent, std::vector<int>());

        E_Right = cv::findEssentialMat(rightKeyPointsPrevious, rightKeyPointsCurrent, camMatRight, cv::RANSAC, 0.999, 1.0, mask_Right);

        //recover pose right
        cv::Mat R_Right, t_Right;
        cv::Mat points3D_homogeneous_Right;
        int inlierPose_Right;
        inlierPose_Right = cv::recoverPose(E_Right, rightKeyPointsPrevious, rightKeyPointsCurrent, camMatRight, R_Right, t_Right, mask_Right);

        //convert ponts right to cartesian

        //filter only inliers of right images

        //draw right (optional)


        //---------------------------------------------------------------------------------
        //4. Triangulation of matching key points for L(t-1) & L (t) and for R(t-1) & R(t).
        //---------------------------------------------------------------------------------
    //    std::vector<cv::Point3f> d3PointsVectorLeft, d3PointsVectorRight;
    //    triangulate(matchedKeyPointsLeft_tMinus1, matchedKeyPointsLeft_t, d3PointsVectorLeft);
    //    triangulate(matchedKeyPointsRight_tMinus1, matchedKeyPointsRight_t, d3PointsVectorRight);

        //-------------------------------------------------------
        //5. Relative camera pose estimation in world coordinates
        //-------------------------------------------------------

        //solvePnPRansac for left to get rotation vector and translation vector
        cv::Mat rVec_Left, tVec_Left;
        std::vector<cv::Point2f> projectedPoints_Left;

        //solvePnPRansac for right to get rotation vector and translation vector
        cv::Mat rVec_Right, tVec_Right;
        std::vector<cv::Point2f> projectedPoints_Right;

        //6. Determining the motion type and direction of the camera
        //      -> Kalman filter

        //7. Finding the exact translation motion of the camera
    }
    else{
        if(iterations == 1){
            //no execution of pose estimation because process just started and there are not enough images
            //set Koordinates to 0,0,0
            std::cout << "fist image" << std::endl;
        }
        else if(iterations > 1 && (imageLeft.empty() && imageRight.empty())){
            //image is empty
            //keep Koordinates from prevous iteration
            std::cout << "image not found" << std::endl;
        }
        else{
            //not enough key points found. Koordinates are kept from prevous iteration
            std::cout << "not enough key points" << std::endl;
        }
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
    std::cout << "Overall mean time for total image matching: " << matchingTimeAcc/iterations << "ms." << std::endl;
    std::cout << "Overall average number of matches: " << numberOfMatchesAcc/iterations << "." << std::endl;
    std::cout << "Overall average number of found key points: " << numberOfKeyPointsAcc/iterations << "." << std::endl;
}

void ImageProcessor::detectFeatures(cv::Mat remappedL, cv::Mat remappedR, std::vector<cv::KeyPoint> &keyPointVectorLeft, std::vector<cv::KeyPoint> &keyPointVectorRight, int selectedFeatureDetector){
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

void ImageProcessor::describeFeatures(cv::Mat remappedL, cv::Mat remappedR, cv::Mat &descriptorLeft, cv::Mat &descriptorRight, std::vector<cv::KeyPoint> keyPointVectorLeft, std::vector<cv::KeyPoint> keyPointVectorRight, int selectedFeatureDescription){
    //error handling in general

    QElapsedTimer descriptorTimer;
    descriptorTimer.start();

    std::thread t1descript([&] {descriptorLeft = featureDescriptionMethod(remappedL, keyPointVectorLeft, selectedFeatureDescriptor);});
    std::thread t2descript([&] {descriptorRight = featureDescriptionMethod(remappedR, keyPointVectorRight, selectedFeatureDescriptor);});
    t1descript.join();
    t2descript.join();

    double descriptionTime = descriptorTimer.elapsed();
    descriptionTimeAcc = descriptionTimeAcc + descriptionTime;
}

void ImageProcessor::featureMatching(cv::Mat descriptorLeft, cv::Mat descriptorRight, std::vector<cv::KeyPoint> keyPointVectorLeft, std::vector<cv::KeyPoint> keyPointVectorRight, int selectedFeatureDescriptor, int selectedFeatureMatching, std::vector<cv::DMatch> &matchesLeftRight, std::vector<cv::KeyPoint> &matchedKeyPointsLeft_tMinus1, std::vector<cv::KeyPoint> &matchedKeyPointsLeft_t, std::vector<cv::KeyPoint> &matchedKeyPointsRight_tMinus1, std::vector<cv::KeyPoint> &matchedKeyPointsRight_t){
    //error handling in general ToDo

    //------------------------------------
    //Step One: Match Left and Right Image
    //------------------------------------
    QElapsedTimer matchingTimer;
    matchingTimer.start();

    if(descriptorLeft.rows >= 3  && descriptorRight.rows >= 3){
        matchesLeftRight = featureMatchingMethod(descriptorLeft, descriptorRight, selectedFeatureDescriptor, selectedFeatureMatching);
        std::cout << matchesLeftRight.size() << " matches in images in t" << std::endl;
    }
    else{
        std::cerr << "Not enough features could be detected in atleast one image and therefore no matches could be found." << std::endl;
    }

    //-----------------------------------------------------------------------------------------------------
    //Step Two: Select key points and descriptors from matches of left and right images
    //key points are used for triangulation and descriptors are used for temporal matching (t matching t-1)
    //-----------------------------------------------------------------------------------------------------
    std::vector<cv::KeyPoint> matchedKeyPointsLeft, matchedKeyPointsRight;
    cv::Mat matchedDescriptorsLeft, matchedDescriptorsRight;
    for(std::vector<cv::DMatch>::size_type i = 0; i < matchesLeftRight.size(); i++){
        matchedKeyPointsLeft.push_back(keyPointVectorLeft[matchesLeftRight[i].queryIdx]);
        matchedKeyPointsRight.push_back(keyPointVectorRight[matchesLeftRight[i].trainIdx]);
        matchedDescriptorsLeft.push_back(descriptorLeft.row(matchesLeftRight[i].queryIdx));
        matchedDescriptorsRight.push_back(descriptorRight.row(matchesLeftRight[i].trainIdx));
    }

    double matchingTime = matchingTimer.elapsed();
    matchingTimeAcc = matchingTimeAcc + matchingTime;
    numberOfMatchesAcc = numberOfMatchesAcc + matchesLeftRight.size();

    //---------------------------------------------------------------------------------------------
    //Step Three: Temporal matching. Match Left image at t-1 and t. Match Right image at t-1 and t.
    //than select key points of all images.
    //---------------------------------------------------------------------------------------------

    //Not executed for first image
    if(!prevImageLeft.empty() && !prevImageRight.empty()){
        std::vector<cv::DMatch> matchesLeftConsecutive, matchesRightConsecutive;

        //only possible for > 3
        if(prevDescriptorLeft.rows >= 3 && prevImageRight.rows >= 3 && matchedDescriptorsLeft.rows >= 3 && matchedDescriptorsRight.rows >= 3){
            matchesLeftConsecutive = featureMatchingMethod(prevDescriptorLeft, matchedDescriptorsLeft, selectedFeatureDescriptor, selectedFeatureMatching);
            matchesRightConsecutive = featureMatchingMethod(prevDescriptorRight, matchedDescriptorsRight, selectedFeatureDescriptor, selectedFeatureMatching);

            std::cout << matchesLeftConsecutive.size() << " matches in left image in t & t-1" << std::endl;
            std::cout << matchesRightConsecutive.size() << " matches in right image in t & t-1" << std::endl;
        }
        else{
            //error not enough key points for good triangulation
        }

        if(matchesLeftConsecutive.size() > 3 && matchesRightConsecutive.size() > 3){
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
        }
        else{
            //error not enough key points for good triangulation.
        }
    }{
        //error no previous image
    }

    prevKeypointsLeft = matchedKeyPointsLeft;
    prevKeypointsRight = matchedKeyPointsRight;
    prevDescriptorLeft = matchedDescriptorsRight;
    prevDescriptorRight = matchedDescriptorsRight;
}

void ImageProcessor::triangulate(std::vector<cv::KeyPoint> keyPointVectorLeft, std::vector<cv::KeyPoint> keyPointVectorRight, std::vector<cv::Point3f>& d3PointsVector){
    cv::Mat leftProjMat = imageSet.getProjMatL();
    cv::Mat rightProjMat = imageSet.getProjMatR();

    std::vector<cv::Point2f> leftKeyPoints, rightKeyPoints;
    cv::Mat points3DHomogeneous;
    cv::Point3f d3Points;

    if(keyPointVectorLeft.size() >= 3 && keyPointVectorRight.size() >= 3){
        //convert key points
        cv::KeyPoint::convert(keyPointVectorLeft, leftKeyPoints, std::vector<int>());
        cv::KeyPoint::convert(keyPointVectorRight, rightKeyPoints, std::vector<int>());

        //calculate the 3D points in homogenous coordinates
        cv::triangulatePoints(leftProjMat, rightProjMat, leftKeyPoints, rightKeyPoints, points3DHomogeneous);

        //convert from homogenous coordinates to Cartesian 3D coordinates
    //    for(int i = 0; i < points3DHomogeneous.cols; i++){
    //        d3Points.x = points3DHomogeneous[0,i]/points3DHomogeneous(3,i);
    //        d3Points.y = points3DHomogeneous(1,i)/points3DHomogeneous(3,i);
    //        d3Points.z = points3DHomogeneous(2,i)/points3DHomogeneous(3,i);
    //        if(d3Points.x < 10000 && d3Points.y < 10000  && d3Points.z < 10000)
    //            d3PointsVector.push_back(d3Points);
    //    }
    }
    else{
        //throw error because not enough points for triangulation
    }
}

std::vector<cv::KeyPoint> ImageProcessor::featureDetectionMethod(cv::Mat remapped, int selectedFeatureDetection){
    std::vector<cv::KeyPoint> keyPointVector;

    //1. Feature detection for both stereo pair images
    //1=ORB, 2=FAST, 3=BRISK, 4=SIFT, 5=SURF
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
        cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create();
        sift->detect(remapped, keyPointVector, cv::Mat());

        selectedDetector = "SIFT";

        break;
    }
    case 5:{
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

std::vector<cv::DMatch> ImageProcessor::featureMatchingMethod(cv::Mat descriptorOne, cv::Mat descriptorTwo, int selectedFeatureDescription ,int SelectedFeatureMatching){
    std::vector<std::vector<cv::DMatch>> matches;
    std::vector<cv::DMatch> goodMatches;

    //check if number of features in both vectors is equal or greater than number of nearest neighbors in knn match.
    //else knn match will throw an error
    if((descriptorOne.rows >= 2) && (descriptorTwo.rows >= 2)){

        //matching ratio
        const float ratio = 0.7;

        switch (SelectedFeatureMatching) {
        case 1:{
            //Brute-Force Matcher
            selectedMatcher = "Brute-Force";

            //specifies the distance measurement to be used. Default cv::NORM_L2
            int normType = cv::NORM_L2;

            //If it is true, Matcher returns only those matches with value (i,j)
            //such that i-th descriptor in set A has j-th descriptor in set B as the best match and vice-versa.
            //That is, the two features in both sets should match each other. It provides consistent result,
            //and is a good alternative to ratio test proposed by D.Lowe in SIFT paper.
            //false by default.
            //https://docs.opencv.org/master/dc/dc3/tutorial_py_matcher.html
            int crossCheck = false;

            switch (selectedFeatureDescription) {
            case 1:
            case 2:
            case 3:{
                //best values for orb, brief or brisk descriptors
                normType = cv::NORM_HAMMING;
                crossCheck = true;

                //use matcher with normtype and crosscheck
                cv::BFMatcher matcher(normType, crossCheck);
                matcher.match(descriptorOne, descriptorTwo, goodMatches);

                break;
            }
            case 4:
            case 5:{
                //use default values and knnMatch for sift or surf
                cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create();
                matcher->knnMatch(descriptorOne, descriptorTwo, matches, 2);

                //apply ratio test by D.Lowe
                const float ratio_thresh = 0.75f;
                for(size_t i = 0; i < matches.size(); i++)
                {
                    if(matches[i][0].distance < ratio_thresh * matches[i][1].distance)
                    {
                        goodMatches.push_back(matches[i][0]);
                    }
                }

                break;
            }
            default:
                break;
            }

            break;
        }
        case 2:{
            //FLANN (Fast Library for Approximate Nearest Neighbors) Matcher
            selectedMatcher = "FLANN";

            switch (selectedFeatureDescription) {
            case 1:
            case 2:
            case 3: {
                //Flann parameter. algorithm = flann indexed lsh, table_number = 12, key_size = 20, multi_probe_level = 1   #
                cv::FlannBasedMatcher matcher (new cv::flann::LshIndexParams(12,20,2));
                matcher.knnMatch(descriptorOne, descriptorTwo, matches, 2);
                break;
            }
            case 4:
            case 5:{
                //Flann parameter. algorithm = kd tree, trees = 4
                cv::FlannBasedMatcher matcher (new cv::flann::KDTreeIndexParams(5));
                matcher.knnMatch(descriptorOne, descriptorTwo, matches, 2);
                break;
            }
            default:

                break;
            }

            const float ratio_thresh = 0.75f;
            for (size_t i = 0; i < matches.size(); i++)
            {
                if (matches[i].size() == 2 && (matches[i][0].distance < ratio_thresh * matches[i][1].distance))
                {
                    goodMatches.push_back(matches[i][0]);
                }
            }

            break;
        }
        default:
            break;
        }

        //Optimization of matches
        for(size_t i = 0; i < matches.size(); i++){
            if(matches[i].size() < 2)
                continue;
            if(matches[i][0].distance > ratio * matches[i][1].distance)
                continue;
            goodMatches.push_back(matches[i][0]);
        }
    }
    else{
        std::cout << "Less than two features found in at least one descriptor." << std::endl;
    }

    return goodMatches;
}
