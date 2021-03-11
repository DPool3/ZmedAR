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
    ui->startCalibration_button->setText(QString::fromStdString("Stereokalibrierung läuft"));
    try{
        //check if an image set has been loaded
        if(imageSet.getPath() != ""){
            //perform the stereo calibration
            performSteroCalibration();
        }
    }catch(const std::exception& e){
        std::string text(e.what());
        helper.callErrorDialog(text);
    }
    ui->startCalibration_button->setText(QString::fromStdString("Stereokalibrierung starten"));
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

std::string StereoCalibration::splitFileName(const std::string& str){
    std::size_t found = str.find_last_of("/\\");
    return str.substr(0,found);
}

void StereoCalibration::performSteroCalibration()
{
    //Load images and find all checkerboard corner points
    loadImgPoints(imageSet.getColumns(),
                  imageSet.getRows(),
                  imageSet.getNumberOfImages(),
                  imageSet.getSquareSize(),
                  imageSet.getPath(),
                  imageSet.getFileType(),
                  imageSet.getPatternType());

    //read the intrinsics and distortion coefficients for each camera and store them in Mat format.
    //F stores the fundamental matrix.
    //E stores the essential matrix.
    //R stores the rotation from the left to the right camera.
    //T stores the translation from the left to the right camera.
    cv::Mat CamL, CamR, DistCoefL, DistCoefR, R, F, E;
    cv::Vec3d T;
    int flag = 0;
    flag |= cv::CALIB_FIX_INTRINSIC;	//tells the stereoCalibrate function to not guess the individual intrinsics for each camera.
    std::cout << "Read intrinsics" << std::endl;

    double reprojectionError = 0.0;
    try{
        double reprojectionError = cv::stereoCalibrate(object_points, imagePointsL, imagePointsR, CamL, DistCoefL, CamR, DistCoefR, imgL.size(), R, T, E, F, cv::CALIB_FIX_INTRINSIC |cv::CALIB_SAME_FOCAL_LENGTH);
        std::cout << "Reprojection Error: " + std::to_string(reprojectionError) + "\n";
        ui->reprojectionError_spinbox->setValue(reprojectionError);
    }catch(const std::exception& e){
        std::string message(e.what());
        throw std::invalid_argument("Es gab ein Problem bei der Anzahl gefundener Eckpunkte des Musters. Aus diesem Grund konnte die Stereokalibrierung nicht durchgeführt werden.\n\n" + message);
    }

    //store all the calibration data in a YAML file.
    imageSet.setReprojectionError(reprojectionError);
    imageSet.setCamL(CamL);
    imageSet.setCamR(CamR);
    imageSet.setDistCoefL(DistCoefL);
    imageSet.setDistCoefR(DistCoefR);
    imageSet.setR(R);
    //imageSet.setT(T); //to do. why is there an error reading vec3d from file?
    imageSet.setE(E);
    imageSet.setF(F);

    printf("Done Calibration\n");

    //Stereo rectification is the task of applying a projective transformation to both image planes such that the resulting epipolar lines become horizontal scan lines.
    printf("Starting Rectification\n");

    //RL is the rectification transform for the left camera.
    //RR for the right camera.
    //PL is projection matrix in the new rectified coordinate system for the left camera.
    //PR for the right camera.
    //Q is known as the disparity-to-depth mapping matrix.
    cv::Mat RL, RR, PL, PR, Q;

    cv::stereoRectify(CamL, DistCoefL, CamR, DistCoefR, imgL.size(), R, T, RL, RR, PL, PR, Q, cv::CALIB_ZERO_DISPARITY);

    cv::Mat rmap[2][2];
    cv::initUndistortRectifyMap(CamL, DistCoefL, RL, PL, imgL.size(), CV_16SC2, rmap[0][0], rmap[0][1]);
    cv::initUndistortRectifyMap(CamR, DistCoefR, RR, PR, imgR.size(), CV_16SC2, rmap[1][0], rmap[1][1]);

    cv::Mat remappedL, remappedR;
    cv::remap(imgL, remappedL, rmap[0][0], rmap[0][1], cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());
    cv::remap(imgR, remappedR, rmap[1][0], rmap[1][1], cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar());

    displayImages(remappedL, remappedR);

    printf("Done Rectification\n");
}

//loadImgPoints is used to find all the corner points of each image and theri corresponding 3D world point
void StereoCalibration::loadImgPoints(
    int board_width,
    int board_height,
    int num_imgs,
    float square_size,
    std::string path,
    std::string fileType,
    std::string patternType)
{
    cv::Size board_size = cv::Size(board_width, board_height);

    cv::Mat Zero = cv::Mat(480, 320, CV_64F, 0.0);

    for (int i = 1; i <= num_imgs; i++) {
        //Load images left and right
        std::string tempLeftPath = path + "/Left";
        std::string tempRightPath = path + "/Right";
        tempLeftPath += "/" + std::to_string(i) + "left" + fileType;
        tempRightPath += "/" + std::to_string(i) + "right" + fileType;
        imgL = cv::imread(tempLeftPath, cv::IMREAD_COLOR);
        imgR = cv::imread(tempRightPath, cv::IMREAD_COLOR);

        bool foundL, foundR = false;

        //differentiation of pattern types in image sets
        if (patternType == "chessboard") {
            //set flags for findChessboardCorners
            //int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE;
            int chessBoardFlags = cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE;
            //find corner points for left image
            try{
                foundL = cv::findChessboardCorners(imgL, board_size, cornersL, chessBoardFlags);
                //find corner points for right image
                foundR = cv::findChessboardCorners(imgR, board_size, cornersR, chessBoardFlags);
            }catch(const std::exception& e){
                std::string message(e.what());
                throw std::invalid_argument("Die angegebene Anzahl Spalten und Reihen stimmt nicht mit denen des Musters überein.\n\n" + message);
            }

        }
        else if (patternType == "circle") {
            //find circle center for left image
            foundL = cv::findCirclesGrid(imgL, board_size, cornersL);
            //find circle center for right image
            foundR = cv::findCirclesGrid(imgR, board_size, cornersR);
        }
        else {
            //find circle center for left image
            foundL = cv::findCirclesGrid(imgL, board_size, cornersL, cv::CALIB_CB_ASYMMETRIC_GRID);
            //find circle center for right image
            foundR = cv::findCirclesGrid(imgR, board_size, cornersR, cv::CALIB_CB_ASYMMETRIC_GRID);
        }

        //Object points are stored. We try to keep the world origin as the top left corner point.
        std::vector< cv::Point3f > obj;
        for (int i = 0; i < board_height; i++) {
            for (int j = 0; j < board_width; j++) {
                obj.push_back(cv::Point3f((float)j * square_size, (float)i * square_size, 0));
            }
        }

        if (foundL && foundR) {
            cv::cvtColor(imgL, grayL, cv::COLOR_RGB2GRAY);
            //further refinement of the corners with cornerSubPix
            cv::cornerSubPix(grayL, cornersL, cv::Size(5, 5), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            cv::cvtColor(imgR, grayR, cv::COLOR_RGB2GRAY);
            //further refinement of the corners with cornerSubPix
            cv::cornerSubPix(grayR, cornersR, cv::Size(5, 5), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            if(this->showImages){
                cv::Mat tempR = imgR.clone(); //neded because imgR is used later;
                cv::drawChessboardCorners(tempR, board_size, cornersR, foundR);

                cv::Mat tempL = imgL.clone(); //neded because imgR is used later;
                cv::drawChessboardCorners(tempL, board_size, cornersL, foundL);

                displayImages(tempL, tempR);
            }

            std::cout << i << ". Found corners!\n";
            imagePointsL.push_back(cornersL);
            imagePointsR.push_back(cornersR);
            object_points.push_back(obj);
        }
        else if (foundR && !foundL) {
            cv::cvtColor(imgR, grayR, cv::COLOR_RGB2GRAY);
            //further refinement of the corners with cornerSubPix
            cv::cornerSubPix(grayR, cornersR, cv::Size(5, 5), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            if(this->showImages){
                cv::Mat tempR = imgR.clone(); //neded because imgR is used later;
                cv::drawChessboardCorners(tempR, board_size, cornersR, foundR);
                displayImageRight(tempR);
            }
        }
        else if (!foundR && foundL){
            cv::cvtColor(imgL, grayL, cv::COLOR_RGB2GRAY);
            //further refinement of the corners with cornerSubPix
            cv::cornerSubPix(grayL, cornersL, cv::Size(5, 5), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.1));

            if(this->showImages){
                cv::Mat tempL = imgL.clone(); //neded because imgR is used later;
                cv::drawChessboardCorners(tempL, board_size, cornersL, foundL);
                displayImageLeft(tempL);
            }
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

        //wait
        cv::waitKey(2000);
}

void StereoCalibration::displayImageRight(cv::Mat imageRight){
        cv::resize(imageRight, imageRight, cv::Size(480, 320), 0, 0);
        cv::cvtColor(imageRight, imageRight, CV_BGR2RGB);
        QImage qimgRight((const unsigned char*) imageRight.data, imageRight.cols, imageRight.rows, QImage::Format_RGB888);
        ui->videoLabelRight->setPixmap(QPixmap::fromImage(qimgRight));
        ui->videoLabelRight->resize(ui->videoLabelRight->pixmap()->size());
        //wait
        cv::waitKey(2000);
}

void StereoCalibration::displayImageLeft(cv::Mat imageLeft){
        cv::resize(imageLeft, imageLeft, cv::Size(480, 320), 0, 0);
        cv::cvtColor(imageLeft, imageLeft, CV_BGR2RGB);
        QImage qimgLeft((const unsigned char*) imageLeft.data, imageLeft.cols, imageLeft.rows, QImage::Format_RGB888);
        ui->videoLabelLeft->setPixmap(QPixmap::fromImage(qimgLeft));
        ui->videoLabelLeft->resize(ui->videoLabelLeft->pixmap()->size());
        //wait
        cv::waitKey(2000);
}
