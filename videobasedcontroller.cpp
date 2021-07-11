#include "videobasedcontroller.h"

VideoBasedController::VideoBasedController()
{

}

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

void VideoBasedController::getVideoFrames(){
    this->captureLeft >> this->imageLeft;
    this->captureRight >> this->imageRight;

    if(imageLeft.empty() || imageRight.empty()){
        stop();
        return;
    }
}

void VideoBasedController::stop(){
    videoManager.releaseVideoCapturePair(this->captureLeft, this->captureRight);
    imageProcessor.closeTrackingFile();
}

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

int VideoBasedController::getfps(){
    return this->playFps;
}
