#include "mainsettings.h"

MainSettings::MainSettings(){

}

//generel
//getter
std::string MainSettings::getImageSetSelectionPath()
{
    return this->imageSetSelectionPath;
}

std::string MainSettings::getImageSetsPath()
{
    return this->imageSetsPath;
}

std::string MainSettings::getVideosPath()
{
    return this->videosPath;
}

std::string MainSettings::getRootPath()
{
    return this->rootPath;
}

int MainSettings::getFps(){
    return fps;
}

int MainSettings::getSaveFps(){
    return saveFps;
}

//setter
int MainSettings::setImageSetSelectionPath(std::string newPathName)
{
    if(!newPathName.empty()){
        this->imageSetSelectionPath = newPathName;
        return 0;
    }
    return 1;
}

int MainSettings::setImageSetsPath(std::string newPathName)
{
    if(!newPathName.empty()){
        this->imageSetsPath = rootPath + "/" + newPathName;
        return 0;
    }
    return 1;
}

int MainSettings::setVideosPath(std::string newPathName)
{
    if(!newPathName.empty()){
        this->videosPath = rootPath + "/" + newPathName;
        return 0;
    }
    return 1;
}

int MainSettings::setRootPath(std::string newPathName)
{
    this->rootPath = newPathName;
    return 0;
}

void MainSettings::setFps(int newFps){
    this->fps = newFps;
}

void MainSettings::setSaveFps(int newSaveFps){
    this->saveFps = newSaveFps;
}

//Videos tab
bool MainSettings::setDisplayVideos(bool newValue){
    displayVideos = newValue;
    return true;
}

bool MainSettings::getDisplayVideos(){
    return this->displayVideos;
}

bool MainSettings::setRecordVideos(bool newValue){
    recordVideos = newValue;
    return true;
}

bool MainSettings::getRecordVideos(){
    return this->recordVideos;
}

bool MainSettings::setFileName(std::string newFileName){
    if(!newFileName.empty()){
        fileName = newFileName;
        return true;
    }
    return false;
}

std::string MainSettings::getFileName(){
    return this->fileName;
}

std::string MainSettings::getFileType(){
    return this->fileType;
}

cv::Size MainSettings::getFrameSize(){
    return framesize;
}
