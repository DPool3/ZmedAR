#include "videomanager.h"

/**
 * @brief VideoManager::VideoManager ist der Konstruktor und setzt alle
 * notwendigen Infos direkt aus den MainSettings.
 */
VideoManager::VideoManager()
{
    MainSettings settings;

    //get current frame rate for saving videos
    this->saveFps = settings.getSaveFps();

    //get current file type for videos
    this->fileType = settings.getVideoFileType();

    //get current video save path
    this->videoSavePath = settings.getVideosPath();

    //get current video file name
    this->fileName = settings.getVideoFileName();

    //get current frame size
    this->frameSize = settings.getFrameSize();

    this->fps = settings.getFps();
}

/**
 * @brief VideoManager::createVideoWriterPair erstellt die beiden VideoWriter für das
 * Speichern der Videos. Verwendet dafür den Aufruf von createVideoWriter.
 * @param writerLeft
 * @param writerRight
 * @param directoryNameAddition
 * @param fps
 */
void VideoManager::createVideoWriterPair(cv::VideoWriter &writerLeft, cv::VideoWriter &writerRight, std::string directoryNameAddition, int fps)
{
    //create a new directory path with the current video save path, time and date and the directory name addition
    std::string videoDirectoryName = DirectoryManager().createVideoDirectory(directoryNameAddition);

    //create the save paths for both files and call the crateVideoWritermethod
    createVideoWriter(writerLeft, videoDirectoryName + "/" + fileName + "_Links" + fileType, fps);
    createVideoWriter(writerRight, videoDirectoryName + "/" + fileName + "_Rechts" + fileType, fps);
}

/**
 * @brief VideoManager::createVideoWriter erstellt einen VideoWriter.
 * @param writer
 * @param path
 * @param fps
 */
void VideoManager::createVideoWriter(cv::VideoWriter & writer, std::string path, int fps)
{
    //get codec for video format (important lower case)
    int codec = cv::VideoWriter::fourcc('m','p','4','v');
    cv::VideoWriter newWriter(path, codec, fps, this->frameSize);
    writer = newWriter;

}

//release VideoWriter
/**
 * @brief VideoManager::releaseVideoWriterPair gibt die beiden VideoWriter wieder frei,
 * um die Videos abzuschließen. Verwendet dafür den Aufruf von releaseVideoWriter.
 * @param writerL
 * @param writerR
 */
void VideoManager::releaseVideoWriterPair(cv::VideoWriter & writerL, cv::VideoWriter & writerR){
    releaseVideoWriter(writerL);
    releaseVideoWriter(writerR);
}

/**
 * @brief VideoManager::releaseVideoWriter gibt einen VideoWriter frei.
 * @param writer
 */
void VideoManager::releaseVideoWriter(cv::VideoWriter & writer){
    writer.release();
}

/**
 * @brief VideoManager::createVideoCapturePair erstellt ein VideoCapture paar.
 * @param captureLeft
 * @param captureRight
 * @param pathLeft
 * @param pathRight
 */
void VideoManager::createVideoCapturePair(cv::VideoCapture &captureLeft, cv::VideoCapture &captureRight, std::string pathLeft, std::string pathRight){
    if(DirectoryManager().pathOrFileExists(pathLeft) &&
       DirectoryManager().pathOrFileExists(pathRight)){
        captureLeft.open(pathLeft);
        captureRight.open(pathRight);

        if(!captureLeft.isOpened() || !captureRight.isOpened()){
            throw std::invalid_argument("Error: Mindestens ein Video konnte nicht geöffnet werden. Prüfen Sie das Dateiformat der Videos auf .mp4.");
        }
    }
}

/**
 * @brief VideoManager::releaseVideoCapturePair gibt das VideoCapture paar frei
 * und verwendet dafür die Methdode releaseVideoCapture.
 * @param captureLeft
 * @param captureRight
 */
void VideoManager::releaseVideoCapturePair(cv::VideoCapture &captureLeft, cv::VideoCapture &captureRight){
    releaseVideoCapture(captureLeft);
    releaseVideoCapture(captureRight);
}

/**
 * @brief VideoManager::releaseVideoCapture gibt ein VideoCapture Objekt frei.
 * @param capture
 */
void VideoManager::releaseVideoCapture(cv::VideoCapture &capture){
    capture.release();
}

//Save Images in VideoWriter
/**
 * @brief VideoManager::saveImages ruft die Methode saveImage in zwei threads für
 * links und rechts auf.
 * @param imgLeft
 * @param imgRight
 * @param videoWriterLeft
 * @param videoWriterRight
 */
void VideoManager::saveImages(cv::Mat imgLeft, cv::Mat imgRight, cv::VideoWriter videoWriterLeft, cv::VideoWriter videoWriterRight){
    std::thread saveLeftThread(&VideoManager::saveImage, this, imgLeft, videoWriterLeft);
    std::thread saveRightThread(&VideoManager::saveImage, this, imgRight, videoWriterRight);
    saveLeftThread.join();
    saveRightThread.join();
}

/**
 * @brief VideoManager::saveImage speichert die Bilder in den Videos.
 * @param img
 * @param writer
 */
void VideoManager::saveImage(cv::Mat img, cv::VideoWriter writer){
    try{
        writer.write(img);
    }catch(std::exception& e){
        std::cerr << "error writing image to video file." << std::endl;
    }
}
