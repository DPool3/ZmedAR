#include "directorymanager.h"

/**
 * @brief DirectoryManager::DirectoryManager ist der Konstruktor
 */
DirectoryManager::DirectoryManager()
{

}

//Init
/**
 * @brief DirectoryManager::initDirectoryHierarchy prüft ob die Ordner existieren,
 * falls nicht, werden diese erstellt.
 */
void DirectoryManager::initDirectoryHierarchy(){
    MainSettings settings;
    //check for path of basic directories. If the don't exist, create them
    if(!pathOrFileExists(settings.getRootPath())){
        createDirectory(settings.getRootPath());
    }

    if(!pathOrFileExists(settings.getTrackingFilePath())){
        createDirectory(settings.getTrackingFilePath());
    }

    if(!pathOrFileExists(settings.getImageSetsPath())){
        createDirectory(settings.getImageSetsPath());
    }

    if(!pathOrFileExists(settings.getVideosPath())){
        createDirectory(settings.getVideosPath());
    }
}

//Create Methods
/**
 * @brief DirectoryManager::createDirectory erstellt einen Ordner mittels
 * angegebenem Pfad.
 * @param path
 */
void DirectoryManager::createDirectory(std::string path){
    std::string command = "mkdir " + path;
    system(command.c_str());
    return;
}

/**
 * @brief DirectoryManager::createStereoDirectory erstellt einen Ordner
 * und seine zwei Unterordner "Links" und "Rechts" unter einem
 * angegebenen Pfad.
 * @param path
 */
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

/**
 * @brief DirectoryManager::createVideoDirectory erstellt einen Ordner
 * speziell für die beiden Videosequenzen.
 * @param dirNameAddition
 * @return
 */
std::string DirectoryManager::createVideoDirectory(std::string dirNameAddition){
    std::string dateAndTime = getCurrentDateAsString();
    std::string videoPath = MainSettings().getVideosPath();
    createDirectory(videoPath + "/" + dateAndTime + dirNameAddition);
    return videoPath + "/" + dateAndTime + dirNameAddition;
}

/**
 * @brief DirectoryManager::createImageSetDirectory erstellt einen Ordner
 * speziell für das ImageSet
 * @return
 */
std::string DirectoryManager::createImageSetDirectory(){
    std::string dateAndTime = getCurrentDateAsString();
    std::string imageSetsPath = MainSettings().getImageSetsPath();
    createStereoDirectory(imageSetsPath + "/" + dateAndTime);
    return imageSetsPath + "/" + dateAndTime;
}

//Check Methods
/**
 * @brief DirectoryManager::pathOrFileExists prüft die Existenz des Ordners
 * oder der Datei im Pfad.
 * @param path
 * @return true falls alles okay ist. Sonst Fehlermeldung.
 */
bool DirectoryManager::pathOrFileExists(std::string path){
    struct stat buffer;

    //check if path is empty
    if(path == "")
        throw std::invalid_argument("Error: Mindestens ein angegebener Pfad ist leer.");

    if(!(stat (path.c_str(), &buffer) == 0))
        throw std::invalid_argument("Error: Mindestens eine angegebene Datei konnte nicht gefunden werden.");

    return true;
}

//Time and Date
/**
 * @brief DirectoryManager::getCurrentDateAsString erstellt einen Strin mit
 * Datum und Uhrzeit, der für die Benennung der Ordner verwendet wird.
 * @return String der Datum und Uhrzeit enthält.
 */
std::string DirectoryManager::getCurrentDateAsString(){
    char some_buffer[64];
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    auto as_time_t = std::chrono::system_clock::to_time_t(now);
    struct tm tm;
    if(::gmtime_r(&as_time_t, &tm))
        if(std::strftime(some_buffer, sizeof(some_buffer), "%d-%m-%Y-%H-%M-%S", &tm))
            return std::string{some_buffer};
    throw std::runtime_error("Failed to get current date as string");
}
