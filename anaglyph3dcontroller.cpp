#include "anaglyph3dcontroller.h"

/**
 * @brief Anaglyph3DController::Anaglyph3DController Konstrukter der Klasse.
 */
Anaglyph3DController::Anaglyph3DController()
{

}

/**
 * @brief Anaglyph3DController::searchVideoPath ruft den Dateiexplorer auf,
 * um einen Videopfad auszuwählen.
 * @return Pfad als QString.
 */
QString Anaglyph3DController::searchVideoPath()
{
    return dialogManager.getPathFromFileSystem();
}

/**
 * @brief Anaglyph3DController::getAnaglyphImage liest die Bilder der Videos aus,
 * ruft die Generierung eines Anaglyph 3D Bildes auf und speichert dieses in der Variable.
 * @param anaglyphImage
 * @return true falls das Anaglyph 3D Bild erstellt werden konnte. False falls eines der
 * Bilder leer ist.
 */
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

/**
 * @brief Anaglyph3DController::reinitAnaglyph3DController initialisiert und reinitialisiert den Prozess
 * für die Erstellung der Anaglyph 3D Videos. Dafür werden die Videos geladen und die Bildrate des Videos
 * ausgelesen.
 */
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

/**
 * @brief Anaglyph3DController::getPlayFps gibt die zuvor ausgelesene Bildrate
 * des Videos zurück.
 * @return Bildrate des aktuellen Videos.
 */
int Anaglyph3DController::getPlayFps()
{
    return this->playFps;
}

/**
 * @brief Anaglyph3DController::stopAnaglyph3DController stoppt den laufenden Prozess
 * in der anaglyph3dcontroller Klasse. Hier werden die Videos die gerade gelesen werden,
 * mit Hilfe des VideoManagers wieder freigegeben.
 */
void Anaglyph3DController::stopAnaglyph3DController()
{
    videoManager.releaseVideoCapturePair(this->captureLeft, this->captureRight);
}
