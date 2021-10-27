#include "PylonCamera.h"

//constructor
/**
 * @brief pylonCamera::pylonCamera ist der Konstruktor.
 */
pylonCamera::pylonCamera(){}

//public functions
/**
 * @brief pylonCamera::initCameras initialisiert die Kameras, entsprechend
 * der Initialisierungsbeispiele. Es wird nach mindestens 2 verbundennen
 * Kameras gesucht. Dann wird das Format bestimmt, ein Kamera-Array
 * initialisiert, die Kameras in dem Array gespeichert und geöffnet.
 * Zum schluss wird eine der Kameras um 180° gedreht, um die Rotation
 * im Prototypen auszugleichen.
 */
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

    try{
        // Create and attach all Pylon Devices.
        for ( size_t i = 0; i < cameras.GetSize(); ++i)
        {
            cout << "checking device at " << i << "." << endl;
            IPylonDevice *device = tlFactory.CreateDevice(devices[i]);
            cameras[i].Attach(device);
            cout << "Using device " << cameras[i].GetDeviceInfo().GetModelName() << endl;
        }
    }catch(const GenericException &e){
        std::string exceptionMsg = e.what();
        throw runtime_error(exceptionMsg);
    }

    cameras.Open();

    GenApi_3_1_Basler_pylon::INodeMap& nodemap0 = cameras[0].GetNodeMap();
    CBooleanParameter(nodemap0, "ReverseX").SetValue(true);
    CBooleanParameter(nodemap0, "ReverseY").SetValue(true);
}

/**
 * @brief pylonCamera::setBrightness setzt die Helligkeit
 * @param newValue
 */
void pylonCamera::setBrightness(float newValue){
    GenApi_3_1_Basler_pylon::INodeMap& nodemap0 = cameras[0].GetNodeMap();
    GenApi_3_1_Basler_pylon::INodeMap& nodemap1 = cameras[1].GetNodeMap();
    CFloatParameter(nodemap0, "BslBrightness").SetValue(newValue);
    CFloatParameter(nodemap1, "BslBrightness").SetValue(newValue);
}

/**
 * @brief pylonCamera::setExposure setzt die Belichtungszeit
 * @param newValue
 */
void pylonCamera::setExposure(float newValue){
    GenApi_3_1_Basler_pylon::INodeMap& nodemap0 = cameras[0].GetNodeMap();
    GenApi_3_1_Basler_pylon::INodeMap& nodemap1 = cameras[1].GetNodeMap();
    CFloatParameter(nodemap0, "ExposureTime").SetValue(newValue);
    CFloatParameter(nodemap1, "ExposureTime").SetValue(newValue);
}

/**
 * @brief pylonCamera::setContrast setzt den Kontrast
 * @param newValue
 */
void pylonCamera::setContrast(float newValue){
    GenApi_3_1_Basler_pylon::INodeMap& nodemap0 = cameras[0].GetNodeMap();
    GenApi_3_1_Basler_pylon::INodeMap& nodemap1 = cameras[1].GetNodeMap();
    CFloatParameter(nodemap0, "BslContrast").SetValue(newValue);
    CFloatParameter(nodemap1, "BslContrast").SetValue(newValue);
}

/**
 * @brief pylonCamera::setSaturation setzt die Sättigung
 * @param newValue
 */
void pylonCamera::setSaturation(float newValue){
    GenApi_3_1_Basler_pylon::INodeMap& nodemap0 = cameras[0].GetNodeMap();
    GenApi_3_1_Basler_pylon::INodeMap& nodemap1 = cameras[1].GetNodeMap();
    CFloatParameter(nodemap0, "BslSaturationValue").SetValue(newValue);
    CFloatParameter(nodemap1, "BslSaturationValue").SetValue(newValue);
}

/**
 * @brief pylonCamera::startGrabbing startet die Bildaufnahme der Kameras
 */
void pylonCamera::startGrabbing(){
    cameras.StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByUser);
}

/**
 * @brief pylonCamera::stopGrabbing stoppt die Bildaufnahme der Kameras
 */
void pylonCamera::stopGrabbing(){
    cameras.StopGrabbing();
}

/**
 * @brief pylonCamera::grabImages fragt gleichzeitig Bilder der Kameras ab,
 * bevor die Bilder in cv::Mat formatiert werden.
 * @param imgLeft
 * @param imgRight
 * @return true falls alles okay war, false falls es nicht erfolgreich war
 * die Bilder zu erhalten.
 */
bool pylonCamera::grabImages(cv::Mat &imgLeft, cv::Mat &imgRight){
    QElapsedTimer timer;
    timer.start();
    try{
        cameras[0].RetrieveResult(10, pylonResultLeft, TimeoutHandling_ThrowException);
        cameras[1].RetrieveResult(10, pylonResultRight, TimeoutHandling_ThrowException);

//        std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;
//        std::cout << "Cameras is grabbing: " << cameras.IsGrabbing() << endl;
//        std::cout << "Result left grab succeeded: " << pylonResultLeft->GrabSucceeded() << endl;
//        std::cout << "Result right grab succeeded: " << pylonResultRight->GrabSucceeded() << endl;
//        std::cout << "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%" << endl;

        if(pylonResultLeft->GrabSucceeded() && pylonResultRight->GrabSucceeded()){

            //convert images
            imageFormater(imgLeft, imgRight);

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
        std::string errMsg = "Es gab einen Fehler bei dem Beziehend er Bilder. Bitte prüfen Sie die Verbindung der Kameras und starten die Aufnahme erneut.\n\"" +
                             exceptionMsg + "\"";
        throw runtime_error(errMsg);
    }
}

/**
 * @brief pylonCamera::imageFormater ruft imageFormaterLeft/Right in threads auf.
 * @param imgLeft
 * @param imgRight
 */
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

/**
 * @brief pylonCamera::imageFormaterLeft formatiert das linke bild.
 * @return das cv::Mat des linken Bildes.
 */
cv::Mat pylonCamera::imageFormaterLeft(){
    formatConverter.Convert(pylonImageLeft, pylonResultLeft);
    cv::Mat imgLeft(pylonImageLeft.GetHeight(), pylonImageLeft.GetWidth(), CV_8UC3, (uint8_t*)pylonImageLeft.GetBuffer());
    return imgLeft;
}

/**
 * @brief pylonCamera::imageFormaterRight formatiert das rechte bild.
 * @return das cv::Mat des rechten Bildes.
 */
cv::Mat pylonCamera::imageFormaterRight(){
    formatConverter.Convert(pylonImageRight, pylonResultRight);
    cv::Mat imgRight(pylonImageRight.GetHeight(), pylonImageRight.GetWidth(), CV_8UC3, (uint8_t*)pylonImageRight.GetBuffer());
    return imgRight;
}

/**
 * @brief pylonCamera::getAverageExecutionTime berechnet die durchschnittliche
 * Durchführungsdauer des grabs und formats.
 * @return durchschnittliche grab und format Zeit.
 */
double pylonCamera::getAverageExecutionTime(){
    if(completeGrabAndFormat == 0 || executionCounter == 0)
        return 0;
    return completeGrabAndFormat / executionCounter;
}

/**
 * @brief pylonCamera::getAverageFormatTime berechnet die durchschnittliche
 * Format Zeit.
 * @return durchschnittliche Format Zeit.
 */
double pylonCamera::getAverageFormatTime(){
    if(formatTime == 0 || formatCounter == 0)
        return 0;
    return formatTime / formatCounter;
}
