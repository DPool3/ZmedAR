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

    //read in used image set to use variables for rectification and remapping
    cv::Mat Left_Stereo_Map1 = imageSet.getLeftStereoMap1();
    cv::Mat Left_Stereo_Map2 = imageSet.getLeftStereoMap2();
    cv::Mat Right_Stereo_Map1 = imageSet.getRightStereoMap1();
    cv::Mat Right_Stereo_Map2 = imageSet.getRightStereoMap2();

    //variables for results
    cv::Mat grayL, grayR;
    cv::Mat remappedL, remappedR;

    cv::Mat descriptorLeft, descriptorRight;
    std::vector<cv::Point2f> leftKeyPoints, rightKeyPoints;
    std::vector<cv::KeyPoint> keyPointVectorLeft, keyPointVectorRight;
    std::vector<cv::DMatch> matchesLeftRight, matchesLeftConsecutive, matchesRightConsecutive;
    std::vector<cv::Point3f> d3PointsVector;

    //color conversion rgb to gray for faster detection
    cv::cvtColor(imageLeft, grayL, CV_RGB2GRAY);
    cv::cvtColor(imageRight, grayR, CV_RGB2GRAY);

    //---------------------------------------------------
    //1. remap images to remove distortion and to rectify
    //---------------------------------------------------
    cv::remap(grayL, remappedL, Left_Stereo_Map1, Left_Stereo_Map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    cv::remap(grayR, remappedR, Right_Stereo_Map1, Right_Stereo_Map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

    //----------------------------------^
    //2. Feature Detection & Description
    //----------------------------------

    //Feature Detection
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

    //Feature Description
    QElapsedTimer descriptorTimer;
    descriptorTimer.start();

    std::thread t1descript([&] {descriptorLeft = featureDescriptionMethod(remappedL, keyPointVectorLeft, selectedFeatureDescriptor);});
    std::thread t2descript([&] {descriptorRight = featureDescriptionMethod(remappedR, keyPointVectorRight, selectedFeatureDescriptor);});
    t1descript.join();
    t2descript.join();

    cv::KeyPoint::convert(keyPointVectorLeft, leftKeyPoints, std::vector<int>());
    cv::KeyPoint::convert(keyPointVectorRight, rightKeyPoints, std::vector<int>());

    double descriptionTime = descriptorTimer.elapsed();
    descriptionTimeAcc = descriptionTimeAcc + descriptionTime;

    //----------------------------------
    //3. Feature Matching left and right
    //----------------------------------
    QElapsedTimer matchingTimer;
    matchingTimer.start();

    if(!descriptorLeft.empty() && !descriptorRight.empty()){
        //match current images in time t
        matchesLeftRight = featureMatchingMethod(descriptorLeft, descriptorRight, selectedFeatureDescriptor, selectedFeatureMatching);
        std::cout << matchesLeftRight.size() << " matches in images in t" << std::endl;
    }
    else{
        std::cerr << "No Features could be detected in atleast one image and therefore no matches could be found." << std::endl;
    }

    double matchingTime = matchingTimer.elapsed();
    matchingTimeAcc = matchingTimeAcc + matchingTime;
    numberOfMatchesAcc = numberOfMatchesAcc + matchesLeftRight.size();

    //----------------------------
    //4. Temporal Feature Matching
    //----------------------------
    //Only execute if there is a previous image
    if((!prevImageLeft.empty() && !prevImageRight.empty()) && (!descriptorLeft.empty() && !descriptorRight.empty())){

        matchesLeftConsecutive = featureMatchingMethod(descriptorLeft, prevDescriptorLeft, selectedFeatureDescriptor, selectedFeatureMatching);
        matchesRightConsecutive = featureMatchingMethod(descriptorRight, prevDescriptorRight, selectedFeatureDescriptor, selectedFeatureMatching);

        std::cout << matchesLeftConsecutive.size() << " matches in left image in t & t-1" << std::endl;
        std::cout << matchesRightConsecutive.size() << " matches in right image in t & t-1" << std::endl;
    }

    //--------------------------------------
    //5. Triangulation of matching Keypoints
    //--------------------------------------
    //d3PointsVector = triangulate(leftKeyPoints, rightKeyPoints);

    //---------------------------
    //6. Relative Pose Estimation
    //---------------------------
    //
    //Estimating the relative pose between the left images and finding the inlier matches
    //      -> RANSAC

    //7. Determining the motion type and direction of the camera
    //      -> Kalman filter

    //8. Finding the exact translation motion of the camera

    double completeTime = totalImageMatchingTimer.elapsed();
    completeTimeAcc = completeTimeAcc + completeTime;

    //Set the current values as previous
    prevImageLeft = remappedL;
    prevImageRight = remappedR;
    prevKeypointsLeft = leftKeyPoints;
    prevKeypointsRight = rightKeyPoints;
    prev3DPointsVector = d3PointsVector;
    prevDescriptorLeft = descriptorLeft;
    prevDescriptorRight = descriptorRight;
    prevMatchesLeftRight = matchesLeftRight;

    //Outputs
    std::cout << "Overall mean time complete: " << completeTimeAcc/iterations << "ms." << std::endl;
    std::cout << "Overall mean time for total feature detection: " << detectionTimeAcc/iterations << "ms." << std::endl;
    std::cout << "Overall mean time for total feature description: " << descriptionTimeAcc/iterations << "ms." << std::endl;
    std::cout << "Overall mean time for total image matching: " << matchingTimeAcc/iterations << "ms." << std::endl;
    std::cout << "Overall average number of matches: " << numberOfMatchesAcc/iterations << "." << std::endl;
    std::cout << "Overall average number of found key points: " << numberOfKeyPointsAcc/iterations << "." << std::endl;
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

std::vector<cv::Point3f> ImageProcessor::triangulate(std::vector<cv::Point2f> leftKeyPoints, std::vector<cv::Point2f> rightKeyPoints){
    cv::Mat leftProjMat = imageSet.getProjMatL();
    cv::Mat rightProjMat = imageSet.getProjMatR();
    std::vector<cv::Point3f> d3PointsVector;
    cv::Point3f d3Points;
    cv::Mat points3DHomogeneous;

    //calculate the 3D points in homogenous coordinates
    cv::triangulatePoints(leftProjMat, rightProjMat, leftKeyPoints, rightKeyPoints, points3DHomogeneous);

    //convert from homogenous coordinates to Cartesian 3D coordinates
//    for(int i = 0; i < d4Points.cols; i++){
//        d3Points.x = d4Points[0,i]/d4Points(3,i);
//        d3Points.y = d4Points(1,i)/d4Points(3,i);
//        d3Points.z = d4Points(2,i)/d4Points(3,i);
//        if(d3Points.x < 10000 && d3Points.y < 10000  && d3Points.z < 10000)
//            d3PointsVector.push_back(d3Points);
//    }

    return d3PointsVector;
}
