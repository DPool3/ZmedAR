#include "camerabasedcontroller.h"

CameraBasedController::CameraBasedController()
{
    saveTimer = new QTimer(this);
    saveTimer->setInterval(1000/videoManager.fps);
    QObject::connect(saveTimer, SIGNAL(timeout()), this, SLOT(saveImages()));

    cameraImageTimer = new QTimer(this);
    cameraImageTimer->setInterval(1000/videoManager.fps);
    QObject::connect(cameraImageTimer, SIGNAL(timeout()), this, SLOT(getCameraImages()));

}

CameraBasedController::~CameraBasedController(){
    stopController();
}

void CameraBasedController::startStopCameraBasedProcess(){
    if(cameraImageTimer->isActive()){
        stopController();
    }
    else{
        startController();
    }
}

void CameraBasedController::startController(){
    //Cameras initialisieren
    //Initialize cameras
    try{
        cameras.initCameras();
    }
    catch (const exception &e)
    {
        std::string exceptionMsg = e.what();
        std::string errMsg = "Error: Es gabe einen Fehler während der Inititalisierung der Kameras.\n\"" + exceptionMsg + "\"";
        DialogManager().callErrorDialog(errMsg);
        return;
    }
    cameras.startGrabbing();
    //neuen ImageProcessor erstellen und Bildaquirierung starten
    imageProcessor = ImageProcessor();
    cameraImageTimer->start();
}

void CameraBasedController::stopController(){
    //Aufnahme stoppen falls laufend
    if(saveTimer->isActive()){
        saveTimer->stop();
        videoManager.releaseVideoWriterPair(this->writerLeft, this->writerRight);
    }
    //Bildaquirierung stoppen und Mat zurücksetzen
    if(cameraImageTimer->isActive()){
        cameraImageTimer->stop();
        this->imageLeft.release();
        this->imageRight.release();
        cameras.stopGrabbing();
    }
}

void CameraBasedController::startStopRecording(){
    //Stoppen und dann writer releasen
    if(saveTimer->isActive()){
        saveTimer->stop();
        videoManager.releaseVideoWriterPair(this->writerLeft, this->writerRight);
    }
    //writer erstellen und dann timer starten
    else{
        videoManager.createVideoWriterPair(this->writerLeft, this->writerRight, "", 30);
        saveTimer->start();
    }
}

void CameraBasedController::setExposure(int newValue){
    std::cout << float(newValue) << std::endl;
    cameras.exposureValue = float(newValue);
}

void CameraBasedController::setBrightness(double newValue){
    std::cout << float(newValue) << std::endl;
    cameras.brightnessValue = float(newValue);
}

void CameraBasedController::setSaturation(double newValue){
    std::cout << float(newValue) << std::endl;
    cameras.saturationValue = float(newValue);
}

void CameraBasedController::getProcessedImages(QImage & qImageLeft, QImage & qImageRight){
    try{
        qImageLeft = imageProcessor.prepImageForDisplay(this->imageLeft);
        qImageRight = imageProcessor.prepImageForDisplay(this->imageRight);
    }catch(std::exception& e){
        std::cerr << "Color conversion error: " << e.what() << std::endl;
        cv::imshow("error image left",this->imageLeft);
        cv::imshow("error image right",this->imageRight);
        cv::waitKey(10000);
        cv::destroyAllWindows();
    }
}

void CameraBasedController::getCameraImages(){
    try{
        if(!cameras.grabImages(this->imageLeft, this->imageRight)){
            std::cerr << "No Image could be grabed" << std::endl;
            return;
        }
    }catch(std::invalid_argument& e){
        DialogManager().callErrorDialog(e.what());
        stopController();
        return;
    }catch(std::exception& e){
        std::cerr << "error: " << e.what() << std::endl;
        return;
    }
}

void CameraBasedController::saveImages(){
    if(!(this->imageLeft.empty() && this->imageRight.empty()))
        videoManager.saveImages(this->imageLeft, this->imageRight, this->writerLeft, this->writerRight);
}

bool CameraBasedController::isRecording(){
    return this->saveTimer->isActive();
}

bool CameraBasedController::isRunning(){
    return this->cameraImageTimer->isActive();
}
