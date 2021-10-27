#include "imagesetcontroller.h"

/**
 * @brief ImageSetController::ImageSetController
 */
ImageSetController::ImageSetController()
{
    cameraImageTimer = new QTimer(this);
    QObject::connect(cameraImageTimer, SIGNAL(timeout()), this, SLOT(getCameraImages()));
}

/**
 * @brief ImageSetController::~ImageSetController
 */
ImageSetController::~ImageSetController(){
    stopRecording();
}

/**
 * @brief ImageSetController::setNumberOfImages
 * @param numberImgs
 */
void ImageSetController::setNumberOfImages(int numberImgs){
    this->numberImages = numberImgs;
}

/**
 * @brief ImageSetController::setNumberRows
 * @param numberRows
 */
void ImageSetController::setNumberRows(int numberRows){
    this->numberRows = numberRows;
}

/**
 * @brief ImageSetController::setNumberColumns
 * @param numberColumns
 */
void ImageSetController::setNumberColumns(int numberColumns){
    this->numberColumns = numberColumns;
}

/**
 * @brief ImageSetController::setPatternType
 * @param patternType
 */
void ImageSetController::setPatternType(std::string patternType){
    this->patternType = patternType;
}

/**
 * @brief ImageSetController::setSquareSize
 * @param squareSize
 */
void ImageSetController::setSquareSize(double squareSize){
    this->squareSize = squareSize;
}

/**
 * @brief ImageSetController::isRunning
 * @return
 */
bool ImageSetController::isRunning(){
    return this->running;
}

/**
 * @brief ImageSetController::startRecording
 * @return
 */
bool ImageSetController::startRecording(){
    try{
        cameras.initCameras();
        cameras.startGrabbing();
        this->cameraImageTimer->start();
        this->imageSet = createImageSet();
        saveInputInImageSet();
        this->running = true;
        return true;
    }catch(const std::exception& e){
        // Error: Fehler während Kamerainitialisierung
        DialogManager().callErrorDialog(e.what());
        return false;
    }
}

/**
 * @brief ImageSetController::stopRecording
 */
void ImageSetController::stopRecording(){
    this->running = false;
    this->cameraImageTimer->stop();
    cameras.stopGrabbing();
    this->origLeft.release();
    this->origRight.release();
}

/**
 * @brief ImageSetController::getProcessedImages
 * @param qImageLeft
 * @param qImageRight
 * @return
 */
bool ImageSetController::getProcessedImages(QImage& qImageLeft, QImage& qImageRight){

    this->imageLeft = this->origLeft.clone();
    this->imageRight = this->origRight.clone();

    try{
        if(!this->imageLeft.empty() && !this->imageRight.empty()){


            cv::resize(this->imageLeft, this->imageLeft, cv::Size(480, 320), 0, 0);
            cv::resize(this->imageRight, this->imageRight, cv::Size(480, 320), 0, 0);

            //Change to RGB format & save it in global Mat
            cv::cvtColor(this->imageLeft, this->imageLeft, cv::COLOR_BGR2RGB);
            cv::cvtColor(this->imageRight, this->imageRight, cv::COLOR_BGR2RGB);

            //Convert to QImage
            QImage ql((const unsigned char*) this->imageLeft.data, this->imageLeft.cols, this->imageLeft.rows, QImage::Format_RGB888);
            QImage qr((const unsigned char*) this->imageRight.data, this->imageRight.cols, this->imageRight.rows, QImage::Format_RGB888);

            qImageLeft = ql;
            qImageRight = qr;

            return true;
        }
        else{
            return false;
        }
    }catch(const std::exception& e){
        std::cout << e.what() << std::endl;
        return false;
    }
}

/**
 * @brief ImageSetController::getCameraImages
 */
void ImageSetController::getCameraImages(){
    std::cout << "called" << std::endl;
    try{
        if(!cameras.grabImages(this->origLeft, this->origRight)){
            std::cerr << "No Image could be grabed" << std::endl;
        }
    }catch(std::runtime_error& e){
        this->cameraImageTimer->stop();
        DialogManager().callErrorDialog(e.what());
    }
}

/**
 * @brief ImageSetController::takeImage
 */
void ImageSetController::takeImage(){
    //create save path
    std::string savePathLeft =  imageSet.getPath() +
                                "/Left/" +
                                std::to_string((imageSet.getNumberRecordedImages()+1)) +
                                "left" +
                                imageSet.getFileType();

    std::string savePathRight = imageSet.getPath() +
                                "/Right/" +
                                std::to_string((imageSet.getNumberRecordedImages()+1)) +
                                "right" +
                                imageSet.getFileType();

    //save left and right image
    cv::imwrite(savePathLeft, this->origLeft);
    cv::imwrite(savePathRight, this->origRight);

    this->imageSet.incrementNumberRecordedImages();
}

/**
 * @brief ImageSetController::createImageSet
 * @return
 */
ImageSet ImageSetController::createImageSet(){
    //Create stereo directory for left and right camera where images are stored
    std::string path = DirectoryManager().createImageSetDirectory();

    //Create Image set with path
    return ImageSet(path);
}

/**
 * @brief ImageSetController::saveInputInImageSet
 */
void ImageSetController::saveInputInImageSet(){
    //save ui entries in new image set settings
    this->imageSet.setNumberOfImages(this->numberImages);
    this->imageSet.setRows(this->numberRows);
    this->imageSet.setColumns(this->numberColumns);
    this->imageSet.setSquareSize(this->squareSize);
    this->imageSet.setPatternType(this->patternType);
}
