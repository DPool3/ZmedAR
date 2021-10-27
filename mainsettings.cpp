#include "mainsettings.h"

/**
 * @brief MainSettings::MainSettings ist der Konstruktor der Klasse
 */
MainSettings::MainSettings(){

}

//getter
/**
 * @brief MainSettings::getImageSetSelectionPath ist eine get Methode für den selection path,
 * welcher eine Vorauswahl eines Image Sets für die Kalibrierung referenziert.
 * @return Pfad des vorher ausgewählten Image Sets.
 */
std::string MainSettings::getImageSetSelectionPath()
{
    return this->imageSetSelectionPath;
}

/**
 * @brief MainSettings::getImageSetsPath ist eine get Methode für den Pfad,
 * unter dem Image Sets erstellt werden.
 * @return Pfad unter dem alle Ordner der Image Sets erstellt werden.
 */
std::string MainSettings::getImageSetsPath()
{
    return this->imageSetsPath;
}

/**
 * @brief MainSettings::getVideosPath ist eine get Methode für den Pfad,
 * unter dem alle Video-Ordner erstellt werden.
 * @return Pfad unter dem alle Video-Ordner erstellt werden.
 */
std::string MainSettings::getVideosPath()
{
    return this->videosPath;
}

/**
 * @brief MainSettings::getTrackingFilePath ist eine get Methode für den Pfad,
 * unter dem alle erstellten Tracking-Dateien erstellt werden.
 * @return Pfad unter dem alle Tracking-Dateien erstellt werden.
 */
std::string MainSettings::getTrackingFilePath(){
    return this->trackingPath;
}

/**
 * @brief MainSettings::getRootPath ist eine get Methode für den Pfad,
 * unter dem die Ordner für Image Sets, Tracking-Dateien und Videos
 * liegen.
 * @return Root-Pfad für alle erstellten und verwendeten Dateien der Software.
 */
std::string MainSettings::getRootPath()
{
    return this->rootPath;
}

/**
 * @brief MainSettings::getFps ist eine get Methode für die Bildrate der
 * Darstellung der live Videostreams.
 * @return Anzahl Bilder Pro Sekunde für live Videostreams.
 */
int MainSettings::getFps(){
    return fps;
}

/**
 * @brief MainSettings::getSaveFps ist eine get Methode für die Bildrate mit der
 * Videos gespeichert werden.
 * @return Anzahl Bilder Pro Sekunde mit der Videos gespeichert werden.
 */
int MainSettings::getSaveFps(){
    return saveFps;
}

/**
 * @brief MainSettings::getVideoFileName ist eine get Methode für den Präfix des
 * Dateinamen der Videos, die gespeichert werden.
 * @return Präfix für die Namen der Videodateien.
 */
std::string MainSettings::getVideoFileName(){
    return this->videoFileName;
}

/**
 * @brief MainSettings::getVideoFileType ist eine get Methode für den Dateityp
 * der Videos.
 * @return Dateityp der Videos.
 */
std::string MainSettings::getVideoFileType(){
    return this->videoFileType;
}

/**
 * @brief MainSettings::getImageFileType ist eine get Methode für den Dateityp
 * der Bilder der Image Sets.
 * @return Dateityp der Bilder.
 */
std::string MainSettings::getImageFileType(){
    return this->imageFileType;
}

/**
 * @brief MainSettings::getFrameSize ist eine get Methode für die Bildmaße.
 * @return Maße der aufgenommenen Bilder
 */
cv::Size MainSettings::getFrameSize(){
    return framesize;
}

//setter
/**
 * @brief MainSettings::setImageSetSelectionPath ist eine set Methode für den selection path,
 * welcher eine Vorauswahl eines Image Sets für die Kalibrierung referenziert. Zuvor wird geprüft,
 * ob diese Variable gesetzt ist oder nicht.
 * @param newPathName.
 * @return 0 falls erfolgreich gesetzt. 1 Falls die Variable empty war.
 */
int MainSettings::setImageSetSelectionPath(std::string newPathName)
{
    if(!newPathName.empty()){
        this->imageSetSelectionPath = newPathName;
        return 0;
    }
    return 1;
}

/**
 * @brief MainSettings::setImageSetsPath ist eine set Methode für den Pfad,
 * unter dem Image Sets erstellt werden.
 * @param newPathName.
 * @return 0 falls erfolgreich gesetzt. 1 Falls die Variable empty war.
 */
int MainSettings::setImageSetsPath(std::string newPathName)
{
    if(!newPathName.empty()){
        this->imageSetsPath = rootPath + "/" + newPathName;
        return 0;
    }
    return 1;
}

/**
 * @brief MainSettings::setVideosPath ist eine set Methode für den Pfad,
 * unter dem alle Video-Ordner erstellt werden.
 * @param newPathName.
 * @return 0 falls erfolgreich gesetzt. 1 Falls die Variable empty war.
 */
int MainSettings::setVideosPath(std::string newPathName)
{
    if(!newPathName.empty()){
        this->videosPath = rootPath + "/" + newPathName;
        return 0;
    }
    return 1;
}

/**
 * @brief MainSettings::setTrackingFilesPath ist eine set Methode für den Pfad,
 * unter dem alle erstellten Tracking-Dateien erstellt werden.
 * @param newTrackingFilesPathName.
 * @return 0 falls erfolgreich gesetzt. 1 Falls die Variable empty war.
 */
int MainSettings::setTrackingFilesPath(std::string newTrackingFilesPathName){
    if(newTrackingFilesPathName.empty()){
        this->trackingPath = rootPath + "/" + newTrackingFilesPathName;
        return 0;
    }
    return 1;
}

/**
 * @brief MainSettings::setRootPath ist eine set Methode für den Pfad,
 * unter dem die Ordner für Image Sets, Tracking-Dateien und Videos
 * liegen.
 * @param newPathName
 * @return 0 falls erfolgreich gesetzt. 1 Falls die Variable empty war.
 */
int MainSettings::setRootPath(std::string newPathName)
{
    this->rootPath = newPathName;
    return 0;
}

/**
 * @brief MainSettings::setFps ist eine set Methode für die Bildrate
 * der live Aufnahmen.
 * @param newFps.
 */
void MainSettings::setFps(int newFps){
    this->fps = newFps;
}

/**
 * @brief MainSettings::setSaveFps ist eine set Methode für die Bildrate
 * der zu speichernden Videos.
 * @param newSaveFps.
 */
void MainSettings::setSaveFps(int newSaveFps){
    this->saveFps = newSaveFps;
}

/**
 * @brief MainSettings::setVideoFileName ist eine set Methode für den Präfix
 * der Videodateinamen.
 * @param newFileName.
 * @return True falls erfolgreich gesetzt. False Falls die Variable empty war.
 */
bool MainSettings::setVideoFileName(std::string newFileName){
    if(!newFileName.empty()){
        videoFileName = newFileName;
        return true;
    }
    return false;
}
