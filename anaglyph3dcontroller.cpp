#include "anaglyph3dcontroller.h"

Anaglyph3DController::Anaglyph3DController()
{

}

QString Anaglyph3DController::searchVideoPath()
{
    return dialogManager.getPathFromFileSystem();
}

bool Anaglyph3DController::getAnaglyphImage(QImage &anaglyphImage)
{
    this->captureLeft >> imageLeft;
    this->captureRight >> imageRight;

    if(imageLeft.empty() || imageRight.empty()){
        stopAnaglyph3DController();
        return false;
    }

    anaglyphImage = this->imagePorcessor.generateAnaglyphImage(imageLeft, imageRight);

    return true;
}

void Anaglyph3DController::reinitAnaglyph3DController()
{
    imagePorcessor = ImageProcessor();
    try{
        videoManager.createVideoCapturePair(this->captureLeft, this->captureRight, this->leftVideoPath, this->rightVideoPath);
        this->playFps = captureLeft.get(cv::CAP_PROP_FPS);
    }catch(const std::invalid_argument& e){
        dialogManager.callErrorDialog(e.what());
        stopAnaglyph3DController();
        return;
    }
}

int Anaglyph3DController::getPlayFps()
{
    return this->playFps;
}

void Anaglyph3DController::stopAnaglyph3DController()
{
    videoManager.releaseVideoCapturePair(this->captureLeft, this->captureRight);
}
