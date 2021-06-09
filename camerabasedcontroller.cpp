#include "camerabasedcontroller.h"

CameraBasedController::CameraBasedController()
{
//    saveTimer = new QTimer(this);
//    saveTimer->setInterval(1000/videoManager.saveFps);
//    QObject::connect(saveTimer, SIGNAL(timeout()), this, SLOT(saveImages()));

    cameraImageTimer = new QTimer(this);
    QObject::connect(cameraImageTimer, SIGNAL(timeout()), this, SLOT(getCameraImagesAndTracking()));

    cameraTrackingTimer = new QTimer(this);
    cameraTrackingTimer->setInterval(1000/videoManager.fps);
    QObject::connect(cameraTrackingTimer, SIGNAL(timeout()), this, SLOT(trackCameras()));
}

CameraBasedController::~CameraBasedController(){
    stopController();
}

//Start/Stop
void CameraBasedController::startStopCameraBasedProcess(){
    if(cameraImageTimer->isActive()){
        stopController();
        if(useVideoSaving){
            double time = timer.elapsed();
            std::cout << "save executed " << iterationCounter  << " times in " << time/1000 << "Seconds. Which results in " << iterationCounter/(time/1000) << "fps." << std::endl;
        }
    }
    else{
        if(useVideoSaving){
            iterationCounter = 0;
            timer.restart();
        }
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

    if(useVideoSaving){
          cameraImageTimer->setInterval(1000/videoManager.saveFps);
    }
    else{
        cameraImageTimer->setInterval(1000/videoManager.fps);
    }

    cameraImageTimer->start();

    if(useViveTracking)
        initTracker();

    if(useCameraTracking )
        cameraTrackingTimer->start();

    if(useVideoSaving)
        startRecording();

}

void CameraBasedController::stopController(){
    //Aufnahme stoppen falls laufend
    stopRecording();

    //Kameratracking stoppen
    if(useCameraTracking)
        cameraTrackingTimer->stop();

    //Bildaquirierung stoppen und Mat zurücksetzen
    if(cameraImageTimer->isActive()){
        cameraImageTimer->stop();
        this->imageLeft.release();
        this->imageRight.release();
        cameras.stopGrabbing();
    }

}

void CameraBasedController::startRecording(){
    if(useVideoSaving){
        videoManager.createVideoWriterPair(this->writerLeft, this->writerRight, "", videoManager.saveFps);
    }
}

void CameraBasedController::stopRecording(){
    if(useVideoSaving){
        videoManager.releaseVideoWriterPair(this->writerLeft, this->writerRight);
    }
}

void CameraBasedController::startStopRecording(){
    if(useVideoSaving)
        useVideoSaving = false;
    else
        useVideoSaving = true;
}

//Bildverarbeitung
bool CameraBasedController::getProcessedImages(QImage & qImageLeft, QImage & qImageRight){
    this->imageLeft = this->origLeft.clone();
    this->imageRight = this->origRight.clone();
    if(!(this->imageLeft.empty() && this->imageRight.empty())){
        try{
//            cv::Mat tempLeft = this->imageLeft.clone();
//            cv::Mat tempRight = this->imageRight.clone();

//            cv::resize(tempLeft, tempLeft, cv::Size(480, 320), 0, 0);
//            cv::resize(tempRight, tempRight, cv::Size(480, 320), 0, 0);

//            cv::cvtColor(tempLeft, tempLeft, CV_BGR2RGB);
//            cv::cvtColor(tempRight, tempRight, CV_BGR2RGB);

//            QImage ql((const unsigned char*) tempLeft.data, tempLeft.cols, tempLeft.rows, QImage::Format_RGB888);
//            QImage qr((const unsigned char*) tempRight.data, tempRight.cols, tempRight.rows, QImage::Format_RGB888);

            cv::resize(this->imageLeft, this->imageLeft, cv::Size(480, 320), 0, 0);
            cv::resize(this->imageRight, this->imageRight, cv::Size(480, 320), 0, 0);

            cv::cvtColor(this->imageLeft, this->imageLeft, CV_BGR2RGB);
            cv::cvtColor(this->imageRight, this->imageRight, CV_BGR2RGB);

            QImage ql((const unsigned char*) this->imageLeft.data, this->imageLeft.cols, this->imageLeft.rows, QImage::Format_RGB888);
            QImage qr((const unsigned char*) this->imageRight.data, this->imageRight.cols, this->imageRight.rows, QImage::Format_RGB888);

            qImageLeft = ql;
            qImageRight = qr;

            return true;
        }catch(std::exception& e){
            std::cerr << "Color conversion error: " << e.what() << std::endl;
            return false;
        }
    }
    return false;
}

bool CameraBasedController::getCameraImages(){
    try{
        if(!cameras.grabImages(origLeft, origRight)){
            std::cerr << "No Image could be grabed" << std::endl;
            return false;
        }
        //images for display
        this->imageLeft = origLeft.clone();
        this->imageRight = origRight.clone();
        //images for saving
        this->imageLeftSave = origLeft.clone();
        this->imageRightSave = origRight.clone();
    }catch(std::invalid_argument& e){
        DialogManager().callErrorDialog(e.what());
        stopController();
        return false;
    }catch(std::exception& e){
        std::cerr << "error: " << e.what() << std::endl;
        //stopController();
        return false;
    }
    return true;
}

void CameraBasedController::initTracker(){
    InitFlags flags;
    flags.printCoords = true;
    flags.printRotation = true;
    flags.printTrack = false;
    flags.printAnalog = false;
    flags.printBEvents = false;
    flags.printEvents = false;
    flags.printSetIds = false;
    flags.pipeCoords = false;

    viveTracking = new ViveTracking(flags);
}

void CameraBasedController::trackTrackers(){
    if(useViveTracking)
        viveTracking->RunProcedure();
}

void CameraBasedController::getCameraImagesAndTracking(){
    if(getCameraImages()){
        trackTrackers();
        saveImages();
    }
}

void CameraBasedController::trackCameras(){
    imageProcessor.stereoVisualOdometry(this->imageLeft, this->imageRight);
}

//Bilder speichern
void CameraBasedController::saveImages(){
    iterationCounter++;
    if(!(this->imageLeftSave.empty() && this->imageRightSave.empty()) && useVideoSaving)
        videoManager.saveImages(this->imageLeftSave, this->imageRightSave, this->writerLeft, this->writerRight);
}

//Bilder werden aus Kamera gelesen
bool CameraBasedController::isRunning(){
    return this->cameraImageTimer->isActive();
}

//Kameraeinstellungen
void CameraBasedController::setExposure(int newValue){
    std::cout << float(newValue) << std::endl;
    cameras.setExposure(float(newValue));
}

void CameraBasedController::setBrightness(double newValue){
    std::cout << float(newValue) << std::endl;
    cameras.setBrightness(float(newValue));
}

void CameraBasedController::setSaturation(double newValue){
    std::cout << float(newValue) << std::endl;
    cameras.setSaturation(float(newValue));
}

void CameraBasedController::setContrast(double newValue){
    std::cout << float(newValue) << std::endl;
    cameras.setContrast(float(newValue));
}
