#include "videorecord.h"

VideoRecord::VideoRecord()
{

}

cv::VideoCapture VideoRecord::createVideoCapture(int id){
    cv::VideoCapture cap;
    cap.open(id);
    checkVideoCaptureOpened(cap);
    return cap;
}

void VideoRecord::checkVideoCaptureOpened(cv::VideoCapture cap){
    if(!cap.isOpened()){
        //throw std::runtime_error("Error: VideoCapture could not be opened.");
    }
}
