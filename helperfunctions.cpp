#include "helperfunctions.h"

MainSettings settings = MainSettings();

HelperFunctions::HelperFunctions()
{

}

//Create Methods
void HelperFunctions::createDirectory(std::string directoryPath){
    std::string command = "mkdir " + directoryPath;
    system(command.c_str());
    return;
}

void HelperFunctions::createStereoDirectory(std::string directoryPath){
    //create current date and time directory
    createDirectory(directoryPath);
    //create Left dir
    std::string commandLeft = "mkdir " + directoryPath + "/Left";
    system(commandLeft.c_str());
    //create Right dir
    std::string commandRight = "mkdir " + directoryPath + "/Right";
    system(commandRight.c_str());
}

int HelperFunctions::createVideoWriter(cv::VideoWriter &witerLeft,
                                       cv::VideoWriter &wirterRight)
{
    //Create paths to save videos
    std::string fileName = settings.getFileName();
    std::string fileType = settings.getFileType();

    std::string savePath = getVideoSavePath();
    std::string leftSavePath = savePath + "/" + fileName + "_Links" + fileType;
    std::string rightSavePath = savePath + "/" + fileName + "_Rechts" + fileType;

    //Get codec for video format (Important! lower case)
    int codec = cv::VideoWriter::fourcc('m','p','4','v');

    //Create the VideoWriter object.
    //Error messages can be ignored. Videos are still saved successfully.
    int fps = settings.getFps();
    cv::VideoWriter videoLeft(leftSavePath, codec, fps, cv::Size(1920, 1200));
    cv::VideoWriter videoRight(rightSavePath, codec, fps, cv::Size(1920, 1200));

    //copy to referenced objects. Otherwise it causes an error.
    witerLeft = videoLeft;
    wirterRight = videoRight;

    return 0;
}

//Get Methods
std::string HelperFunctions::getVideoSavePath(){
    //Default Video file path
    std::string VideoPath = settings.getVideosPath();
    //Build complete path with directory for current date and time
    VideoPath = VideoPath + "/" + getCurrentDateAsString();
    //Create one directory for left and right videos
    createDirectory(VideoPath);
    return VideoPath;
}

std::string HelperFunctions::getCurrentDateAsString(){
    char some_buffer[64];
    std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
    auto as_time_t = std::chrono::system_clock::to_time_t(now);
    struct tm tm;
    if(::gmtime_r(&as_time_t, &tm))
        if(std::strftime(some_buffer, sizeof(some_buffer), "%d-%m-%Y-%H-%M-%S", &tm))
            return std::string{some_buffer};
    throw std::runtime_error("Failed to get current date as string");
}

std::string HelperFunctions::getCurrentSelectedImageSetPath(){
    return settings.getImageSetSelectionPath();
}

int HelperFunctions::getFpsFromMainSettings(){
    return settings.getFps();
}

//Set Methods
void HelperFunctions::setCurrentSelectedImageSetPath(std::string newPath){
    settings.setImageSetSelectionPath(newPath);
}

//Check Methods
void HelperFunctions::checkVideoCaptureOpened(cv::VideoCapture cap)
{
    if(!cap.isOpened()){
        throw std::runtime_error("Error: VideoCapture could not be opened. The Camera is likely not accessible. Try a different ID");
    }
}

void HelperFunctions::checkFrameEmpty(cv::Mat frame){
    if(frame.empty()){
        throw std::runtime_error("Error: Frame is empty.");
    }
}

//Camera operations
bool HelperFunctions::testCamera(int id){
    cv::VideoCapture cap;
    cap.open(id);
    if(!cap.isOpened())
        return false;
    return true;
}

void HelperFunctions::setVideoCapture(cv::VideoCapture &captureLeft, cv::VideoCapture &captureRight){
    int idLeft = 1;

    for(int i = idLeft; i < 20; i++){
        if(testCamera(i)){
            idLeft = i;
            break;
        }
    }

    int idRight = idLeft + 1;

    for(int i = idRight; i < 20; i++){
        if(testCamera(i)){
            idRight = i;
            break;
        }
    }

    captureLeft.open(idLeft);
    captureRight.open(idRight);
}

void HelperFunctions::switchCameras(cv::VideoCapture &captureLeft, cv::VideoCapture &captureRight){
    cv::VideoCapture cap;
    cap = captureLeft;
    captureLeft = captureRight;
    captureRight = cap;
}

//Call Dialog
void HelperFunctions::callErrorDialog(std::string errorMsg){
    ErrorDialog errd;
    errd.setModal(true);
    errd.setErrorText(errorMsg.c_str());
    errd.exec();
}

QString HelperFunctions::getPathFromFileSystem(){
    FileSystem fsd;
    fsd.setModal(true);
    fsd.exec();
    return fsd.getSelectedFile();
}
