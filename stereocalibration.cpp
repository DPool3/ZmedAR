#include "stereocalibration.h"
#include "ui_stereocalibration.h"

HelperFunctions helper = HelperFunctions();

StereoCalibration::StereoCalibration(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::StereoCalibration)
{
    ui->setupUi(this);

    loadImageSet();
}

StereoCalibration::~StereoCalibration()
{
    delete ui;
}

void StereoCalibration::on_startCalibration_button_clicked()
{
    lockUi();
    loadImageSet();
    try{
        //check if an image set has been loaded
        if(imageSet.getPath() != ""){
            //perform the stereo calibration
            std::thread t(&StereoCalibration::performSteroCalibration, this);
            t.detach();
        }
    }catch(const std::exception& e){
        std::string text(e.what());
        helper.callErrorDialog(text);
    }
}

void StereoCalibration::on_searchFile_button_clicked()
{
    //Seach yaml file of image set
    QString qPath = HelperFunctions().getPathFromFileSystem();
    //Set line edit
    ui->lineEdit->setText(qPath);
    //Set currently selected image set
    helper.setCurrentSelectedImageSetPath(qPath.toStdString());
    //Reload image set
    loadImageSet();
}

void StereoCalibration::on_displayImages_checkbox_toggled(bool checked)
{
    this->showImages = checked;
}

void StereoCalibration::loadImageSet(){
    //Get path of image set selection from main settings.
    std::string pathWithAddition = helper.getCurrentSelectedImageSetPath();

    //delete yaml file from string
    std::string path = splitFileName(pathWithAddition);

    //If path is not empty
    if(!path.empty()){
        //Load imageSet for further use
        imageSet = ImageSet(path);
        //Set image set data in ui
        setImageSetDataInUi();
    }
}

void StereoCalibration::setImageSetDataInUi(){
    ui->numberImages_spinbox->setValue(imageSet.getNumberOfImages());
    ui->numbeRows_spinbox->setValue(imageSet.getRows());
    ui->numberColumns_spinbox->setValue(imageSet.getColumns());
    ui->squareSize_spinbox->setValue(imageSet.getSquareSize());
    ui->patternType_lineEdit->setText(QString::fromStdString(imageSet.getPatternType()));
    ui->reprojectionError_spinbox->setValue(imageSet.getReprojectionError());
}

void StereoCalibration::on_resizeFactorSpinBox_valueChanged(double newResizeFactor)
{
    this->resizeFactor = newResizeFactor;
}

std::string StereoCalibration::splitFileName(const std::string& str){
    std::size_t found = str.find_last_of("/\\");
    return str.substr(0,found);
}

void StereoCalibration::performSteroCalibration()
{
    // Creating vector to store vectors of 3D points for each checkerboard image
    std::vector< std::vector< cv::Point3f > > object_points;
    // Creating vector to store vectors of 2D points for each checkerboard image
    std::vector< std::vector< cv::Point2f > > imagePointsL, imagePointsR;
    //read the intrinsics and distortion coefficients for each camera and store them in Mat format.
    //F stores the fundamental matrix.
    //E stores the essential matrix.
    //R stores the rotation from the left to the right camera.
    //T stores the translation from the left to the right camera.
    cv::Mat CamL, DistCoefL, R_L, T_L;
    cv::Mat CamR, DistCoefR, R_R, T_R;
    cv::Mat R, T, E, F;
    cv::Mat new_CamL, new_CamR;

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
    new_CamL = cv::getOptimalNewCameraMatrix(CamL, DistCoefL, grayL.size(), 1, grayL.size(), 0);
    std::cout << "Reprojection Error left camera: " << leftReprojectionError << endl;

    //Calibrate Right camera
    double rightReprojectionError = cv::calibrateCamera(object_points, imagePointsR, grayR.size(), CamR, DistCoefR, R_R, T_R);
    new_CamR = cv::getOptimalNewCameraMatrix(CamR, DistCoefR, grayR.size(), 1, grayR.size(), 0);
    std::cout << "Reprojection Error right camera: " << rightReprojectionError << endl;

    //Stereo calibrate cameras
    int flag = 0;
    flag |= cv::CALIB_FIX_INTRINSIC;

    //This step is performed for transformation between the two cameras and alculate Essential and Fundamental matrix
    double stereoReprojectionError = cv::stereoCalibrate(object_points,
                        imagePointsL,
                        imagePointsR,
                        new_CamL,
                        DistCoefL,
                        new_CamR,
                        DistCoefR,
                        grayR.size(),
                        R, T, E, F,
                        flag, cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 30, 1e-6));

    ui->reprojectionError_spinbox->setValue(stereoReprojectionError);

    cv::stereoRectify(new_CamL, DistCoefL, new_CamR, DistCoefR, grayR.size(), R, T, rect_l, rect_r, proj_mat_l, proj_mat_r, Q, 1);

    cv::initUndistortRectifyMap(new_CamL, DistCoefL, rect_l, proj_mat_l, grayL.size(), CV_16SC2, Left_Stereo_Map1, Left_Stereo_Map2);
    cv::initUndistortRectifyMap(new_CamR, DistCoefR, rect_r, proj_mat_r, grayR.size(), CV_16SC2, Right_Stereo_Map1, Right_Stereo_Map2);

    cv::remap(imgL, remappedL, Left_Stereo_Map1, Left_Stereo_Map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    cv::remap(imgR, remappedR, Right_Stereo_Map1, Right_Stereo_Map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

    displayImages(remappedL, remappedR);

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
    imageSet.setNewCamL(new_CamL);
    imageSet.setNewCamR(new_CamR);
    imageSet.setRectL(rect_l);
    imageSet.setRectR(rect_r);
    imageSet.setProjMatL(proj_mat_l);
    imageSet.setProjMatR(proj_mat_r);
    imageSet.setLeftStereoMap1(Left_Stereo_Map1);
    imageSet.setRightStereoMap1(Right_Stereo_Map1);
    imageSet.setLeftStereoMap2(Left_Stereo_Map2);
    imageSet.setRightStereoMap2(Right_Stereo_Map2);

    releaseUi();

    //src::https://learnopencv.com/making-a-low-cost-stereo-camera-using-opencv/
}

//loadImgPoints is used to find all the corner points of each image and theri corresponding 3D world point
void StereoCalibration::loadImgPoints(
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

        cv::cvtColor(imgL, grayL, CV_RGB2GRAY);
        cv::cvtColor(imgR, grayR, CV_RGB2GRAY);

        //resized images left and right for faster findChessBoardCorners an circles
        if(resizeFactor != 1.0){
            cv::resize(grayL, grayL, cv::Size(grayL.cols*resizeFactor,grayL.rows*resizeFactor),0,0);
            cv::resize(grayR, grayR, cv::Size(grayR.cols*resizeFactor,grayR.rows*resizeFactor),0,0);

            cv::resize(imgL, imgL, cv::Size(imgL.cols*resizeFactor,imgL.rows*resizeFactor),0,0);
            cv::resize(imgR, imgR, cv::Size(imgR.cols*resizeFactor,imgR.rows*resizeFactor),0,0);
        }

        if(patternType == "chessboard"){
            std::thread tl(&StereoCalibration::findChessBoardCorners, this, grayL, board_size, std::ref(cornersL), std::ref(foundL));
            std::thread tr(&StereoCalibration::findChessBoardCorners, this, grayR, board_size, std::ref(cornersR), std::ref(foundR));
            tl.join();
            tr.join();
        }
        else if(patternType == "circle"){
            std::thread tl(&StereoCalibration::findCircleGridSym, this, grayL, board_size, std::ref(cornersL), std::ref(foundL));
            std::thread tr(&StereoCalibration::findCircleGridSym, this, grayR, board_size, std::ref(cornersR), std::ref(foundR));
            tl.join();
            tr.join();
        }
        else if(patternType == "circle asymmetrical"){
            std::thread tl(&StereoCalibration::findCircleGridAsym, this, grayL, board_size, std::ref(cornersL), std::ref(foundL));
            std::thread tr(&StereoCalibration::findCircleGridAsym, this, grayR, board_size, std::ref(cornersR), std::ref(foundR));
            tl.join();
            tr.join();
        }
        else{
            throw std::invalid_argument("Error: Undefined pattern.");
        }

        if (foundL && foundR) {
            std::cout << "Corners found for image " << i << endl;

            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);

            cv::cornerSubPix(grayL, cornersL, cv::Size(3, 3), cv::Size(-1, -1), criteria);
            cv::cornerSubPix(grayR, cornersR, cv::Size(3, 3), cv::Size(-1, -1), criteria);

            if(this->showImages){
                cv::Mat tempLeft = imgL;
                cv::Mat tempRight = imgR;

                cv::drawChessboardCorners(tempLeft, board_size, cornersL, foundL);
                cv::drawChessboardCorners(tempRight, board_size, cornersR, foundR);

                std::thread td(&StereoCalibration::displayImages, this, tempLeft, tempRight);
                td.detach();
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

void StereoCalibration::displayImages(cv::Mat imageLeft, cv::Mat imageRight){
        //Resize Images
        cv::resize(imageLeft, imageLeft, cv::Size(480, 320), 0, 0);
        cv::resize(imageRight, imageRight, cv::Size(480, 320), 0, 0);

        //Change to RGB format & save it in global Mat
        cv::cvtColor(imageLeft, imageLeft, CV_BGR2RGB);
        cv::cvtColor(imageRight, imageRight, CV_BGR2RGB);

        //Convert to QImage
        QImage qimgLeft((const unsigned char*) imageLeft.data, imageLeft.cols, imageLeft.rows, QImage::Format_RGB888);
        QImage qimgRight((const unsigned char*) imageRight.data, imageRight.cols, imageRight.rows, QImage::Format_RGB888);

        //Display on Input Label
        ui->videoLabelLeft->setPixmap(QPixmap::fromImage(qimgLeft));
        ui->videoLabelRight->setPixmap(QPixmap::fromImage(qimgRight));

        //Resize the label to fit the image
        ui->videoLabelLeft->resize(ui->videoLabelLeft->pixmap()->size());
        ui->videoLabelRight->resize(ui->videoLabelRight->pixmap()->size());
}

void StereoCalibration::findCircleGridSym(cv::Mat img, cv::Size board_size, std::vector< cv::Point2f >& corners, bool & found){
    found = cv::findCirclesGrid(img, board_size, corners, cv::CALIB_CB_SYMMETRIC_GRID);
}

void StereoCalibration::findCircleGridAsym(cv::Mat img, cv::Size board_size, std::vector< cv::Point2f >& corners, bool & found){
    found = cv::findCirclesGrid(img, board_size, corners, cv::CALIB_CB_ASYMMETRIC_GRID);
}

void StereoCalibration::findChessBoardCorners(cv::Mat img, cv::Size board_size, std::vector<cv::Point2f> & corners, bool & found){
    found = cv::findChessboardCorners(img, board_size, corners);
}

void StereoCalibration::lockUi(){
    ui->startCalibration_button->setText(QString::fromStdString("Stereokalibrierung läuft"));
    ui->startCalibration_button->setEnabled(false);
    ui->displayImages_checkbox->setEnabled(false);
    ui->searchFile_button->setEnabled(false);
    ui->resizeFactorSpinBox->setEnabled(false);
}

void StereoCalibration::releaseUi(){
    ui->startCalibration_button->setText(QString::fromStdString("Stereokalibrierung starten"));
    ui->startCalibration_button->setEnabled(true);
    ui->displayImages_checkbox->setEnabled(true);
    ui->searchFile_button->setEnabled(true);
    ui->resizeFactorSpinBox->setEnabled(true);
}
