#include "stereocalibrationcontroller.h"

/**
 * @brief StereoCalibrationController::StereoCalibrationController ist der Konstruktor.
 */
StereoCalibrationController::StereoCalibrationController()
{

}

/**
 * @brief StereoCalibrationController::loadImageSet lädt das gewählte ImageSet in ein
 * ImageSet Objekt.
 * @return true falls erfolgreich und false falls nicht erfolgreich.
 */
bool StereoCalibrationController::loadImageSet(){
    //Seach yaml file of image set
    imageSetPath = DialogManager().getPathFromFileSystem();
    //delete yaml file from string
    std::string path = splitFileName(imageSetPath.toStdString());

    //load image set
    if(!(path == "")){
        imageSet = ImageSet(path);
    }
    else{
        return false;
    }

    return true;
}

/**
 * @brief StereoCalibrationController::startStereoCalibration startet einen Kalibrierungsprozess,
 * losgelöst von den restlichen Threads.
 * @return true falls erfolgreich gestartet und false, falls nicht.
 */
bool StereoCalibrationController::startStereoCalibration(){
    //load image set
    try{
        //check if an image set has been loaded
        if(imageSet.getPath() != ""){
            //perform the stereo calibration (can not be stopped)
            std::thread t(&StereoCalibrationController::performSteroCalibration, this);
            t.detach();

            calibrationRunning = true;
        }
        else{
            throw std::invalid_argument("Error: Kein Image Set wurde ausgewählt. Bitte wählen Sie ein Image Set aus und starten Sie den Prozess erneut.");
        }
    }catch(const std::exception& e){
        DialogManager().callErrorDialog(e.what());
        return false;
    }

    return true;
}

/**
 * @brief StereoCalibrationController::performSteroCalibration ruft loadImgPoints aus,
 * um alle Corner Punkte aller Bilder zu erhalten. Anschließend werden zwei einfache
 * Kalibrierungen der einzelnen Kameras durchgeführt, bevor stereoCalibrate
 * mit allen zuvor erhaltenen Informationen durchgeführt wird.
 * Auf diese Weise verbessert sich die Kalibrierung.
 * Zum Schluss wird Rektifiziert, wodurch die Projektionsmatrizen berechnet werden
 * und Entzerrt, wodurch weitere Matrizen für das Remapping erstellt werden.
 * Abschließend wird ein Bild Remapped um das Ergebnis zu sehen.
 * Alle berechneten Werte werden in dem ImageSet Objekt gespeichert.
 */
void StereoCalibrationController::performSteroCalibration(){
    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector< std::vector< cv::Point3f > > object_points;
    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector< std::vector< cv::Point2f > > imagePointsL, imagePointsR;
    //read the intrinsics and distortion coefficients for each camera and store them in Mat format.
    //F stores the fundamental matrix.
    //E stores the essential matrix.
    //R stores the rotation from the left to the right camera.
    //T stores the translation from the left to the right camera.
    //Q is required to get depth map from disparity map
    cv::Mat CamL, DistCoefL, R_L, T_L;
    cv::Mat CamR, DistCoefR, R_R, T_R;
    cv::Mat R, T, E, F;

    //Once we know the transformation between the two cameras we can perform stereo rectification
    cv::Mat rect_l, rect_r, proj_mat_l, proj_mat_r, Q;

    //Undistorted rectified stereo image pair
    cv::Mat Left_Stereo_Map1, Left_Stereo_Map2;
    cv::Mat Right_Stereo_Map1, Right_Stereo_Map2;

    cv::Mat remappedL, remappedR;

    //Load images and find all checkerboard corner points
    loadImgPoints(imageSet.getColumns(),
                  imageSet.getRows(),
                  imageSet.getNumberOfImages(),
                  imageSet.getSquareSize(),
                  imageSet.getPath(),
                  imageSet.getFileType(),
                  imageSet.getPatternType(),
                  object_points,
                  imagePointsL,
                  imagePointsR);

    //Calibrate left camera
    double leftReprojectionError = cv::calibrateCamera(object_points, imagePointsL, grayL.size(), CamL, DistCoefL, R_L, T_L);
    std::cout << "Reprojection Error left camera: " << leftReprojectionError << endl;

    //Calibrate Right camera
    double rightReprojectionError = cv::calibrateCamera(object_points, imagePointsR, grayR.size(), CamR, DistCoefR, R_R, T_R);
    std::cout << "Reprojection Error right camera: " << rightReprojectionError << endl;

    //Stereo calibrate cameras
    int flag = 0;
    flag |= cv::CALIB_FIX_INTRINSIC;

    //This step is performed for transformation between the two cameras and alculate Essential and Fundamental matrix
    double stereoReprojectionError = cv::stereoCalibrate(object_points,
                        imagePointsL,
                        imagePointsR,
                        CamL,
                        DistCoefL,
                        CamR,
                        DistCoefR,
                        grayR.size(),
                        R, T, E, F,
                        flag, cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6));

    cv::stereoRectify(CamL, DistCoefL, CamR, DistCoefR, grayR.size(), R, T, rect_l, rect_r, proj_mat_l, proj_mat_r, Q, 1);

    cv::initUndistortRectifyMap(CamL, DistCoefL, rect_l, proj_mat_l, grayL.size(), CV_16SC2, Left_Stereo_Map1, Left_Stereo_Map2);
    cv::initUndistortRectifyMap(CamR, DistCoefR, rect_r, proj_mat_r, grayR.size(), CV_16SC2, Right_Stereo_Map1, Right_Stereo_Map2);

    cv::remap(imgL, remappedL, Left_Stereo_Map1, Left_Stereo_Map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    cv::remap(imgR, remappedR, Right_Stereo_Map1, Right_Stereo_Map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

    this->displayImageLeft = remappedL;
    this->displayImageRight = remappedR;
    this->newImagesForDisplay = true;

    //store all the calibration data in a YAML file.
    imageSet.setReprojectionError(stereoReprojectionError);
    imageSet.setCamL(CamL);
    imageSet.setCamR(CamR);
    imageSet.setDistCoefL(DistCoefL);
    imageSet.setDistCoefR(DistCoefR);
    imageSet.setR(R);
    imageSet.setT(T);
    imageSet.setE(E);
    imageSet.setF(F);
    imageSet.setQ(Q);
    imageSet.setRectL(rect_l);
    imageSet.setRectR(rect_r);
    imageSet.setProjMatL(proj_mat_l);
    imageSet.setProjMatR(proj_mat_r);
    imageSet.setLeftStereoMap1(Left_Stereo_Map1);
    imageSet.setRightStereoMap1(Right_Stereo_Map1);
    imageSet.setLeftStereoMap2(Left_Stereo_Map2);
    imageSet.setRightStereoMap2(Right_Stereo_Map2);

    calibrationRunning = false;

    //src::https://learnopencv.com/making-a-low-cost-stereo-camera-using-opencv/
}

/**
 * @brief StereoCalibrationController::loadImgPoints sucht mittels findChessBoardCorners
 * alle Eckpunkte des Schachbrettmusters und verbessert diese Ergebnisse mittels
 * cornerSubPix. Allen Punkten werden Ihre Koordinaten entsprechend der
 * Square Size zugeordnet. Die verwendeten parameter sind aus dem ImageSet.
 * @param board_width
 * @param board_height
 * @param num_imgs
 * @param square_size
 * @param path
 * @param fileType
 * @param patternType
 * @param object_points
 * @param imagePointsL
 * @param imagePointsR
 */
void StereoCalibrationController::loadImgPoints(
        int board_width,
        int board_height,
        int num_imgs,
        float square_size,
        std::string path,
        std::string fileType,
        std::string patternType,
        std::vector< std::vector< cv::Point3f > >& object_points,
        std::vector< std::vector< cv::Point2f > >& imagePointsL,
        std::vector< std::vector< cv::Point2f > >& imagePointsR)
{
    // vector to store the pixel coordinates of detected checker board corners
    std::vector< cv::Point2f > cornersL, cornersR;

    // bool to tell if all chess board corners have been found in one image
    bool foundL, foundR = false;

    cv::Size board_size = cv::Size(board_width, board_height);

    //Object points are stored. We try to keep the world origin as the top left corner point.
    std::vector< cv::Point3f > obj;
    for (int i = 0; i < board_height; i++) {
        for (int j = 0; j < board_width; j++) {
            obj.push_back(cv::Point3f((float)j * square_size, (float)i * square_size, 0));
        }
    }

    std::string tempLeftPath = path + "/Left/";
    std::string tempRightPath = path + "/Right/";

    //Loop over all Images in the directories
    for (int i = 1; i <= num_imgs; i++) {

        //Load images left and right
        std::string leftPath = tempLeftPath + std::to_string(i) + "left" + fileType;
        std::string rightPath = tempRightPath + std::to_string(i) + "right" + fileType;

        imgL = cv::imread(leftPath, cv::IMREAD_COLOR);
        imgR = cv::imread(rightPath, cv::IMREAD_COLOR);

        cv::cvtColor(imgL, grayL, cv::COLOR_RGB2GRAY);
        cv::cvtColor(imgR, grayR, cv::COLOR_RGB2GRAY);

        if(patternType == "chessboard"){
            std::thread tl(&StereoCalibrationController::findChessBoardCorners, this, grayL, board_size, std::ref(cornersL), std::ref(foundL));
            std::thread tr(&StereoCalibrationController::findChessBoardCorners, this, grayR, board_size, std::ref(cornersR), std::ref(foundR));
            tl.join();
            tr.join();
        }
        else if(patternType == "circle"){
            std::thread tl(&StereoCalibrationController::findCircleGridSym, this, grayL, board_size, std::ref(cornersL), std::ref(foundL));
            std::thread tr(&StereoCalibrationController::findCircleGridSym, this, grayR, board_size, std::ref(cornersR), std::ref(foundR));
            tl.join();
            tr.join();
        }
        else if(patternType == "circle asymmetrical"){
            std::thread tl(&StereoCalibrationController::findCircleGridAsym, this, grayL, board_size, std::ref(cornersL), std::ref(foundL));
            std::thread tr(&StereoCalibrationController::findCircleGridAsym, this, grayR, board_size, std::ref(cornersR), std::ref(foundR));
            tl.join();
            tr.join();
        }
        else{
            throw std::invalid_argument("Error: Undefined pattern.");
        }

        if (foundL && foundR) {
            std::cout << "Corners found for image " << i << endl;

            if(patternType == "chessboard"){

                cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

                cv::cornerSubPix(grayL, cornersL, cv::Size(3, 3), cv::Size(-1, -1), criteria);
                cv::cornerSubPix(grayR, cornersR, cv::Size(3, 3), cv::Size(-1, -1), criteria);

                cv::Mat tempLeft = imgL;
                cv::Mat tempRight = imgR;

                cv::drawChessboardCorners(tempLeft, board_size, cornersL, foundL);
                cv::drawChessboardCorners(tempRight, board_size, cornersR, foundR);

                //prepare new images for display
                this->displayImageLeft = tempLeft;
                this->displayImageRight = tempRight;
                newImagesForDisplay = true;
            }

            imagePointsL.push_back(cornersL);
            imagePointsR.push_back(cornersR);
            object_points.push_back(obj);
        }
        else{
            std::cout << "No corners could be found for image " << i << endl;
        }
    }

    if(object_points.empty()){
        throw std::invalid_argument("Es konnten keine Eckpunkte gefunden werden.");
    }
}

/**
 * @brief StereoCalibrationController::findCircleGridSym ist für das finden von Kreiszentren
 * in symmetrischen Pattern.
 * @param img
 * @param board_size
 * @param corners
 * @param found
 */
void StereoCalibrationController::findCircleGridSym(cv::Mat img, cv::Size board_size, std::vector< cv::Point2f >& corners, bool & found){
    found = cv::findCirclesGrid(img, board_size, corners, cv::CALIB_CB_SYMMETRIC_GRID);
}

/**
 * @brief StereoCalibrationController::findCircleGridAsym ist für das finden von Kreiszentren
 * in asymmetrischen Pattern.
 * @param img
 * @param board_size
 * @param corners
 * @param found
 */
void StereoCalibrationController::findCircleGridAsym(cv::Mat img, cv::Size board_size, std::vector< cv::Point2f >& corners, bool & found){
    found = cv::findCirclesGrid(img, board_size, corners, cv::CALIB_CB_ASYMMETRIC_GRID);
}

/**
 * @brief StereoCalibrationController::findChessBoardCorners ist fpr das finden von
 * Eckpunkten in einem Schachbrettmuster.
 * @param img
 * @param board_size
 * @param corners
 * @param found
 */
void StereoCalibrationController::findChessBoardCorners(cv::Mat img, cv::Size board_size, std::vector<cv::Point2f> & corners, bool & found){
    found = cv::findChessboardCorners(img, board_size, corners);
}

/**
 * @brief StereoCalibrationController::getCalibrationInfo gibt alle Werte an die
 * Benutzeroberfläche zurück, die dargestellt werden sollen.
 * @param board_width
 * @param board_height
 * @param num_imgs
 * @param square_size
 * @param patternType
 * @param stereoReprojectionError
 */
void StereoCalibrationController::getCalibrationInfo(
        int& board_width,
        int& board_height,
        int& num_imgs,
        float& square_size,
        std::string& patternType,
        double& stereoReprojectionError){
    board_width = imageSet.getColumns();
    board_height = imageSet.getRows();
    num_imgs = imageSet.getNumberOfImages();
    square_size = imageSet.getSquareSize();
    patternType = imageSet.getPatternType();
    stereoReprojectionError = imageSet.getReprojectionError();
}

/**
 * @brief StereoCalibrationController::getImagesForDisplay bereitet die Bilder
 * für die Darstellung in der Benutzeroberfläche vor.
 * @param qImageLeft
 * @param qImageRight
 */
void StereoCalibrationController::getImagesForDisplay(QImage& qImageLeft, QImage& qImageRight){
    //access new image for display
    cv::resize(this->displayImageLeft, this->displayImageLeft, cv::Size(480, 320), 0, 0);
    cv::resize(this->displayImageRight, this->displayImageRight, cv::Size(480, 320), 0, 0);

    //Change to RGB format & save it in global Mat
    cv::cvtColor(this->displayImageLeft, this->displayImageLeft, cv::COLOR_BGR2RGB);
    cv::cvtColor(this->displayImageRight, this->displayImageRight, cv::COLOR_BGR2RGB);

    //Convert to QImage
    QImage ql((const unsigned char*) this->displayImageLeft.data, this->displayImageLeft.cols, this->displayImageLeft.rows, QImage::Format_RGB888);
    QImage qr((const unsigned char*) this->displayImageRight.data, this->displayImageRight.cols, this->displayImageRight.rows, QImage::Format_RGB888);

    qImageLeft = ql;
    qImageRight = qr;

    newImagesForDisplay = false;
}

/**
 * @brief StereoCalibrationController::getImageSetPath gibt den Pfad
 * des ImageSets zurück.
 * @return Pfad des ImageSets.
 */
QString StereoCalibrationController::getImageSetPath(){
    return this->imageSetPath;
}

/**
 * @brief StereoCalibrationController::checkNewImageForDisplay prüft ob bereits
 * neue Bilder zum Darstellen berechnet wurden.
 * @return true falls neue Bilder für die Darstellung vorhanden sind, false falls nicht.
 */
bool StereoCalibrationController::checkNewImageForDisplay(){
    return this->newImagesForDisplay;
}

/**
 * @brief StereoCalibrationController::checkCalibrationRunning prüft, ob die
 * Kalibrierung noch läuft.
 * @return true falls die Kalibrierung noch läuft.
 */
bool StereoCalibrationController::checkCalibrationRunning(){
    return this->calibrationRunning;
}

/**
 * @brief StereoCalibrationController::splitFileName teilt einen String
 * an der Stelle des gewählten Symbols.
 * @param str
 * @return der String, aber ohne das Symbol und alles was dahinter ist.
 */
std::string StereoCalibrationController::splitFileName(const std::string& str){
    std::size_t found = str.find_last_of("/\\");
    return str.substr(0,found);
}
