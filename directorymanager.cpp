#include "directorymanager.h"

DirectoryManager::DirectoryManager()
{

}

//Init
void DirectoryManager::initDirectoryHierarchy(){
    MainSettings settings;
    //check for path of basic directories. If the don't exist, create them
    if(!pathOrFileExists(settings.getRootPath())){
        createDirectory(settings.getRootPath());
    }

    if(!pathOrFileExists(settings.getImageSetsPath())){
        createDirectory(settings.getImageSetsPath());
    }

    if(!pathOrFileExists(settings.getVideosPath())){
        createDirectory(settings.getVideosPath());
    }
}

//Create Methods
void DirectoryManager::createDirectory(std::string path){
    std::string command = "mkdir " + path;
    system(command.c_str());
    return;
}

void DirectoryManager::createStereoDirectory(std::string path){
    //create current date and time directory
    createDirectory(path);
    //create Left dir
    std::string commandLeft = "mkdir " + path + "/Left";
    system(commandLeft.c_str());
    //create Right dir
    std::string commandRight = "mkdir " + path + "/Right";
    system(commandRight.c_str());
}

//Check Methods
bool DirectoryManager::pathOrFileExists(std::string path){
    struct stat buffer;

    //check if path is empty
    if(path == "")
        throw std::invalid_argument("Error: Mindestens ein angegebener Pfad ist leer.");

    if(!(stat (path.c_str(), &buffer) == 0))
        throw std::invalid_argument("Error: Mindestens eine angegebene Datei konnte nicht gefunden werden.");

    return true;
}
