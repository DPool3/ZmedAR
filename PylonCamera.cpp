#include "PylonCamera.h"

//constructor
pylonCamera::pylonCamera(){}

//public functions
void pylonCamera::initCameras(){
    PylonInitialize();

    // Get the transport layer factory.
    CTlFactory& tlFactory = CTlFactory::GetInstance();

    // Get all attached devices and exit application if no device is found.
    DeviceInfoList_t devices;

    if(tlFactory.EnumerateDevices(devices) < 2){
        throw invalid_argument("Es wurden wengier als zwei Kameras gefunden. Bitte Verbinden Sie die Kameras erneut und versuchen es nochmal.");
    }

    //set converter parameters
    formatConverter.OutputPixelFormat = PixelType_BGR8packed;
    formatConverter.OutputBitAlignment = OutputBitAlignment_MsbAligned;

    cameras.Initialize(c_maxCamerasToUse);
    cout << "Number necessary cameras " << cameras.GetSize() << endl;
    cout << "Number devices found " << devices.size() << endl;

    // Create and attach all Pylon Devices.
    for ( size_t i = 0; i < cameras.GetSize(); ++i)
    {
        cout << "checking device at " << i << "." << endl;
        IPylonDevice *device = tlFactory.CreateDevice(devices[i]);
        cameras[i].Attach(device);
        cout << "Using device " << cameras[i].GetDeviceInfo().GetModelName() << endl;
    }

    cameras.Open();

    GenApi_3_1_Basler_pylon::INodeMap& nodemap1 = cameras[1].GetNodeMap();
    CBooleanParameter(nodemap1, "ReverseX").SetValue(true);
    CBooleanParameter(nodemap1, "ReverseY").SetValue(true);
    CFloatParameter(nodemap1, "ExposureTime").SetValue(10000.0);
    CFloatParameter(nodemap1, "BslBrightness").SetValue(0.1);

    GenApi_3_1_Basler_pylon::INodeMap& nodemap0 = cameras[0].GetNodeMap();
    CFloatParameter(nodemap0, "ExposureTime").SetValue(10000.0);
    CFloatParameter(nodemap0, "BslBrightness").SetValue(0.1);
}

void pylonCamera::startGrabbing(){
    cameras.StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByUser);
}

void pylonCamera::stopGrabbing(){
    cameras.StopGrabbing();
}

bool pylonCamera::grabImages(cv::Mat &imgLeft, cv::Mat &imgRight){
    QElapsedTimer timer;
    timer.start();
    try{
        cameras[0].RetrieveResult(15, pylonResultLeft, TimeoutHandling_ThrowException);
        cameras[1].RetrieveResult(15, pylonResultRight, TimeoutHandling_ThrowException);

        if(cameras.IsGrabbing() && pylonResultLeft->GrabSucceeded() && pylonResultRight->GrabSucceeded()){

            //convert images
            imageFormater(imgLeft, imgRight);

//            if(createVideo){
//                writeImages();
//            }

            cout << "complete run of grabImages takes " << timer.elapsed() << "ms" << endl <<
                    "-------------------------------------------------" << endl;
            completeGrabAndFormat += timer.elapsed();
            executionCounter++;

            return true;
        }
        return false;
    }
    catch (const GenericException &e)
    {
        std::string exceptionMsg = e.what();
        std::string errMsg = "Error: es gab einen Fehler bei dem Beziehend er Bilder. Bitte prüfen Sie die Verbindung der Kameras und starten die Aufnahme erneut.\n\"" +
                             exceptionMsg + "\"";
        throw runtime_error(errMsg);
    }
}

//format functions
void pylonCamera::imageFormater(cv::Mat &imgLeft, cv::Mat &imgRight){
    QElapsedTimer timer;
    timer.start();
    std::thread leftT([&] {imgLeft = imageFormaterLeft();});
    std::thread rightT([&] {imgRight = imageFormaterRight();});
    leftT.join();
    rightT.join();
    cout << "time for format " << timer.elapsed() << "ms" << endl;
    formatTime += timer.elapsed();
    formatCounter++;
}

cv::Mat pylonCamera::imageFormaterLeft(){
    formatConverter.Convert(pylonImageLeft, pylonResultLeft);
    cv::Mat imgLeft(pylonImageLeft.GetHeight(), pylonImageLeft.GetWidth(), CV_8UC3, (uint8_t*)pylonImageLeft.GetBuffer());
    return imgLeft;
}

cv::Mat pylonCamera::imageFormaterRight(){
    formatConverter.Convert(pylonImageRight, pylonResultRight);
    cv::Mat imgRight(pylonImageRight.GetHeight(), pylonImageRight.GetWidth(), CV_8UC3, (uint8_t*)pylonImageRight.GetBuffer());
    return imgRight;
}

//output functions
double pylonCamera::getAverageExecutionTime(){
    if(completeGrabAndFormat == 0 || executionCounter == 0)
        return 0;
    return completeGrabAndFormat / executionCounter;
}

double pylonCamera::getAverageFormatTime(){
    if(formatTime == 0 || formatCounter == 0)
        return 0;
    return formatTime / formatCounter;
}

double pylonCamera::getAverageSaveTime(){
    if(saveTime == 0 || saveCounter == 0)
        return 0;
    return saveTime / saveCounter;
}

//Video Functions
//void pylonCamera::setCreateVideo(bool value){
//    this->createVideo = value;
//}

//int pylonCamera::initVideoWriter(){
//    // Check if CVideoWriter is supported and all DLLs are available.
//    if(! CVideoWriter::IsSupported())
//    {
//        cout << "VideoWriter is not supported at the moment. Please install the pylon Supplementary Package for MPEG-4 which is available on the Basler website." << endl;
//        // Releases all pylon resources.
//        PylonTerminate();
//        // Return with error code 1.
//        return 1;
//    }

//    // The frame rate used for playing the video (playback frame rate).
//    const int cFramesPerSecond = 30;

//    // The quality used for compressing the video.
//    const uint32_t cQuality = 100;

//    // Get the required camera settings.
//    CIntegerParameter width( cameras[0].GetNodeMap(), "Width");
//    CIntegerParameter height( cameras[0].GetNodeMap(), "Height");
//    CEnumParameter pixelFormat( cameras[0].GetNodeMap(), "PixelFormat");

//    // Map the pixelType
//    CPixelTypeMapper pixelTypeMapper(&pixelFormat);
//    EPixelType pixelType = pixelTypeMapper.GetPylonPixelTypeFromNodeValue(pixelFormat.GetIntValue());

//    // Set parameters before opening the video writer.
//    videoWriterLeft.SetParameter(
//        (uint32_t) width.GetValue(),
//        (uint32_t) height.GetValue(),
//        pixelType,
//        cFramesPerSecond,
//        cQuality );

//    videoWriterRight.SetParameter(
//        (uint32_t) width.GetValue(),
//        (uint32_t) height.GetValue(),
//        pixelType,
//        cFramesPerSecond,
//        cQuality );

//    // Open the video writer.
//    string leftSavePath, rightSavePath;
//    helper.getCompleteVideoSavePath(leftSavePath, rightSavePath);
//    videoWriterLeft.Open( String_t(leftSavePath.c_str()) );
//    videoWriterRight.Open( String_t(rightSavePath.c_str()) );

//    return 0;
//}

//void pylonCamera::closeVideoWriter(){
//    videoWriterLeft.Close();
//    videoWriterRight.Close();
//}

//void pylonCamera::writeImages(){
//    QElapsedTimer timer;
//    timer.start();
//    videoWriterLeft.Add(pylonResultLeft);
//    videoWriterRight.Add(pylonResultRight);
//    saveTime += timer.elapsed();
//    saveCounter++;
//}
