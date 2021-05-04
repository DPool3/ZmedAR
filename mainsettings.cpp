#include "mainsettings.h"

MainSettings::MainSettings(){

}

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

std::string MainSettings::getVideoFileName(){
    return this->videoFileName;
}

std::string MainSettings::getVideoFileType(){
    return this->videoFileType;
}

std::string MainSettings::getImageFileType(){
    return this->imageFileType;
}

cv::Size MainSettings::getFrameSize(){
    return framesize;
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

bool MainSettings::setVideoFileName(std::string newFileName){
    if(!newFileName.empty()){
        videoFileName = newFileName;
        return true;
    }
    return false;
}
