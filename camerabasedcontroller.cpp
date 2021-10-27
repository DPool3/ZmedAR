#include "camerabasedcontroller.h"

/**
 * @brief CameraBasedController::CameraBasedController initialisiert den Timer
 * cameraImageTimer, der für das auslesen, Darstellen der Bilder sowie das
 * Sensor Tracking notwendig ist und den Timer, der für die stereovisuelle
 * Odometrie notwendig ist.
 */
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

/**
 * @brief CameraBasedController::~CameraBasedController ist der Destruktor der Klasse,
 * der den Controller zuvor vollständig stoppt.
 */
CameraBasedController::~CameraBasedController(){
    stopController();
}

/**
 * @brief CameraBasedController::startStopCameraBasedProcess startet und stoppt
 * den Controller. Falls ausgeführt, wenn es läuft, wird alles gestoppt.
 * Falls es nicht ausgeführt ist, wird es gestoppt.
 */
void CameraBasedController::startStopCameraBasedProcess(){
    if(cameraImageTimer->isActive()){
        stopController();

        //Dieser Teil ist für die Berechnung der fps Zahl
        if(useVideoSaving){
            double time = timer.elapsed();
            std::cout << "save executed " << iterationCounter  << " times in " << time/1000 << "Seconds. Which results in " << iterationCounter/(time/1000) << "fps." << std::endl;
        }
    }
    else{
        //Dieser Teil ist für die Berechnung der fps Zahl
        if(useVideoSaving){
            iterationCounter = 0;
            timer.restart();
        }

        startController();
    }
}

/**
 * @brief CameraBasedController::startController ist direkt für den Start des Prozesses
 * verantwortlich. Es werden die Kameras initialisiert, die Aufnahme der Bilder gestartet,
 * der ImageProcessor für die Bildverarbeitung erstellt, die Geschwindigkeit des Timers
 * gesetzt, abhängig davon ob gespeichert wird oder nicht (Speichern benötigt mehr Zeit
 * und würde maximal 30 fps zulassen), es wird geprüft ob Vive-Tracking verwendet wird bevor
 * der cameraImageTimer gestartet wird. Erst nach dem starten dieses Timers, werden Bilder
 * gelesen, aus diesem Grund können erst nach dieser Zeile, cameraTrackingTimer und der
 * Speichervorgang gestartet werden (falls gewählt), da diese bereits Bilder benötigen.
 */
void CameraBasedController::startController(){
    //Cameras initialisieren
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

    if(useViveTracking){
        trackingInitialized = initTracker();
        if(!trackingInitialized)
            return;
    }

    cameraImageTimer->start();

    if(useCameraTracking )
        cameraTrackingTimer->start();

    if(useVideoSaving)
        startRecording();
}

/**
 * @brief CameraBasedController::stopController stoppt alle Timer und ruft den
 * Destruktor des Vive-Trackers auf, um diesen korrekt zu stoppen. Die Reihenfolge
 * des stoppens ist wichtig, da das Stoppen des cameraImageTimers vor dem
 * Speichern der Bilder und dem cameraTrackingTimer zu einem Fehler führen würde.
 */
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

    if(useViveTracking && trackingInitialized)
        viveTracking->~ViveTracking();

}

/**
 * @brief CameraBasedController::startRecording ruft eine Methode des
 * Videomanagers auf, und gibt die Bilder mit der Bildrate an diesen weiter,
 * damit die Schreibobjekte für die Videos erstellt werden können.
 */
void CameraBasedController::startRecording(){
    if(useVideoSaving){
        videoManager.createVideoWriterPair(this->writerLeft, this->writerRight, "", videoManager.saveFps);
    }
}

/**
 * @brief CameraBasedController::stopRecording ruft die Methode zur
 * Freigabe der Schreibobjekte im Videomanager auf, um den
 * Schreibeprozess zu beenden, damit die Videos beendet werden.
 */
void CameraBasedController::stopRecording(){
    if(useVideoSaving){
        videoManager.releaseVideoWriterPair(this->writerLeft, this->writerRight);
    }
}

/**
 * @brief CameraBasedController::startStopRecording setzt den bool-Wert für
 * das Speichern der Videos in dieser Klasse.
 */
void CameraBasedController::startStopRecording(){
    if(useVideoSaving)
        useVideoSaving = false;
    else
        useVideoSaving = true;
}

//Bildverarbeitung
/**
 * @brief CameraBasedController::getProcessedImages führt die Vorverarbeitung der Bilder
 * für die Darstellung in der Benutzeroberfläche durch, indem es den ImageProcessor
 * verwendet.
 * @param qImageLeft
 * @param qImageRight
 * @return True falls die Bilder korrekt erstellt wurden und False falls es im Prozess
 * ein Problem gab.
 */
bool CameraBasedController::getProcessedImages(QImage & qImageLeft, QImage & qImageRight){
    this->imageLeft = this->origLeft.clone();
    this->imageRight = this->origRight.clone();

    if(!(this->imageLeft.empty() && this->imageRight.empty())){
        try{
            qImageLeft = imageProcessor.prepImageForDisplay(this->imageLeft);
            qImageRight = imageProcessor.prepImageForDisplay(this->imageRight);

            return true;
        }catch(std::exception& e){
            std::cerr << "Color conversion error: " << e.what() << std::endl;
            return false;
        }
    }
    return false;
}

/**
 * @brief CameraBasedController::getCameraImages holt die aktuellen und
 * Konvertierten Bilder des Pylon Cameras.
 * @return True falls alles erfolgreich war und false falls es Probleme gab.
 */
bool CameraBasedController::getCameraImages(){
    try{
        if(!cameras.grabImages(origRight, origLeft)){
            std::cerr << "No Image could be grabed" << std::endl;
            return false;
        }
        //images for display
//        this->imageLeft = origLeft.clone();
//        this->imageRight = origRight.clone();
        //images for saving
        this->imageLeftSave = origLeft.clone();
        this->imageRightSave = origRight.clone();
//    }catch(std::invalid_argument& e){
//        stopController();
//        DialogManager().callErrorDialog(e.what());
//        return false;
    }catch(std::runtime_error& e){
        stopController();
        DialogManager().callErrorDialog(e.what());
        return false;
    }
    return true;
}

/**
 * @brief CameraBasedController::initTracker initialisiert die ViveTracking
 * Klasse für die Durchführung des Trackings.
 * @return True falls es erfolgreich war und false, falls es ein Problem gab.
 */
bool CameraBasedController::initTracker(){
    InitFlags flags;
    flags.printCoords = true;
    flags.printRotation = true;
    flags.printTrack = false;
    flags.printAnalog = false;
    flags.printBEvents = false;
    flags.printEvents = false;
    flags.printSetIds = false;
    flags.pipeCoords = false;

    try{
        viveTracking = new ViveTracking(flags);
    }catch(std::runtime_error& e){
        stopController();
        DialogManager().callErrorDialog(e.what());
        return false;
    }
    return true;
}

/**
 * @brief CameraBasedController::trackTrackers fragt ein mal alle tracker ab.
 */
void CameraBasedController::trackTrackers(){
    if(useViveTracking){
        try{
            viveTracking->RunProcedure();
        }catch(std::runtime_error& e){
            stopController();
            DialogManager().callErrorDialog(e.what());
        }
    }
}

/**
 * @brief CameraBasedController::getCameraImagesAndTracking führt das Tracking
 * und das Speichern der Bilder aus, falls erfolgreich Bilder von den Kameras
 * ausgelesen werden konnten.
 */
void CameraBasedController::getCameraImagesAndTracking(){
    if(getCameraImages()){
        trackTrackers();
        saveImages();
    }
}

/**
 * @brief CameraBasedController::trackCameras führt, unter Verwendung des
 * ImageProcessors die stereovisuelle Odometrie für ein Bildpaar durch.
 */
void CameraBasedController::trackCameras(){
    imageProcessor.stereoVisualOdometry(this->imageLeft, this->imageRight);
}

//Bilder speichern
/**
 * @brief CameraBasedController::saveImages führt das Speichern der Bilder,
 * mittels VideoManager, durch.
 */
void CameraBasedController::saveImages(){
    iterationCounter++;
    if(!(this->imageLeftSave.empty() && this->imageRightSave.empty()) && useVideoSaving)
        videoManager.saveImages(this->imageLeftSave, this->imageRightSave, this->writerLeft, this->writerRight);
}

//Bilder werden aus Kamera gelesen
/**
 * @brief CameraBasedController::isRunning prüft den Status des cameraImageTimers.
 * @return True falls es läuft und false falls es nicht läuft.
 */
bool CameraBasedController::isRunning(){
    return this->cameraImageTimer->isActive();
}

//Kameraeinstellungen
/**
 * @brief CameraBasedController::setExposure ändert die Belichtungszeit.
 * @param newValue ist die neue Belichtungszeit.
 */
void CameraBasedController::setExposure(int newValue){
    std::cout << float(newValue) << std::endl;
    cameras.setExposure(float(newValue));
}

/**
 * @brief CameraBasedController::setBrightness ändert die Helligkeit.
 * @param newValue ist die neue Helligkeit.
 */
void CameraBasedController::setBrightness(double newValue){
    std::cout << float(newValue) << std::endl;
    cameras.setBrightness(float(newValue));
}

/**
 * @brief CameraBasedController::setSaturation ändert die Sättigung.
 * @param newValue ist die neue Sättigung.
 */
void CameraBasedController::setSaturation(double newValue){
    std::cout << float(newValue) << std::endl;
    cameras.setSaturation(float(newValue));
}

/**
 * @brief CameraBasedController::setContrast ändert den Kontrast.
 * @param newValue ist der neue Kontrast.
 */
void CameraBasedController::setContrast(double newValue){
    std::cout << float(newValue) << std::endl;
    cameras.setContrast(float(newValue));
}
