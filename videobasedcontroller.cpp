#include "videobasedcontroller.h"

/**
 * @brief VideoBasedController::VideoBasedController ist der Konstruktor
 */
VideoBasedController::VideoBasedController()
{

}

/**
 * @brief VideoBasedController::reinitVideoBasedController initialisiert und
 * reinitialisiert den videoBasedController bei start. Dafür wird ein
 * ImageProcessor mit den notwendigen Methoden für das Kameratracking erstellt,
 * Eine neue Trackingdatei erstellt, ein VideoCapture Paar erstellt und die
 * FPS Zahl der Videos abgefragt.
 * @param detector
 * @param descriptor
 * @param matcher
 */
void VideoBasedController::reinitVideoBasedController(int detector, int descriptor, int matcher){
    //if combination BRISK/FAST
    if(detector == 4 && descriptor == 1){
        throw std::invalid_argument("SIFT detector kann nicht mit ORB descriptor verwendet werden.");
    }

    imageProcessor = ImageProcessor(detector, descriptor, matcher);

    try{
        imageProcessor.createNewTrackingFile();
        videoManager.createVideoCapturePair(this->captureLeft, this->captureRight, this->videoPathLeft, this->videoPathRight);
        this->playFps = captureLeft.get(cv::CAP_PROP_FPS);
    }catch(const std::invalid_argument& e){
        dialogManager.callErrorDialog(e.what());
        stop();
        return;
    }
}

/**
 * @brief VideoBasedController::stop stoppt den Prozess, indem es das
 * VideoCapture Paar freigibt und die Trackingfile schließt.
 */
void VideoBasedController::stop(){
    videoManager.releaseVideoCapturePair(this->captureLeft, this->captureRight);
    imageProcessor.closeTrackingFile();
}

/**
 * @brief VideoBasedController::getProcessedImages verarbeitet die ausgelesenen Bilder
 * für die Darstellung und führt auch die stereovisuelle Odometrie durch.
 * @param qImageLeft
 * @param qImageRight
 * @return true falls die Bilder nicht leer waren und false, falls mindestens eins leer war.
 */
bool VideoBasedController::getProcessedImages(QImage &qImageLeft, QImage &qImageRight){
    //Bilder von Videos lesen
    this->captureLeft >> this->imageLeft;
    this->captureRight >> this->imageRight;

    if(imageLeft.empty() || imageRight.empty()){
        stop();
        return false;
    }

    //Bilder für darstellung verarbeiten
    try{
        qImageLeft = imageProcessor.prepImageForDisplay(this->imageLeft);
        qImageRight = imageProcessor.prepImageForDisplay(this->imageRight);
    }catch(std::exception& e){
        std::cerr << "Color conversion error: " << e.what() << std::endl;
    }

    //originale für die stereo visuelle odometrie verwenden
    if(useCameraTracking)
        imageProcessor.stereoVisualOdometry(this->imageLeft, this->imageRight);

    return true;
}

/**
 * @brief VideoBasedController::getfps gibt die fps des Videos zurück.
 * @return FPS des Videos.
 */
int VideoBasedController::getfps(){
    return this->playFps;
}
