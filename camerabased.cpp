#include "camerabased.h"
#include "ui_camerabased.h"
using namespace Pylon;
using namespace std;

CameraBased::CameraBased(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CameraBased)
{
    ui->setupUi(this);

    PylonInitialize();

    try{
        initPylon();
    }
    catch (const GenericException &e)
    {
        // Error handling
        cerr << "An exception occurred during initialization of the cameras." << endl
        << e.GetDescription() << endl;
        return;
    }

    //Set frames per second in ms
    const int fps = 1000/HelperFunctions().getFpsFromMainSettings();

    //set update timer
    retrieveImagesTimer = new QTimer(this);
    retrieveImagesTimer->setInterval(fps);
    connect(retrieveImagesTimer, SIGNAL(timeout()), this, SLOT(retrieveImages()));

    //connect(this, SIGNAL(displayNext()), this, SLOT(displayImages()));
    //connect(this, SIGNAL(saveNext()), this, SLOT(saveImages()));
}

CameraBased::~CameraBased()
{
    cameras.StopGrabbing();
    delete ui;
}

void CameraBased::on_startStopRecording_clicked()
{
    //Initialize recording and displaying and start timer if not active
    if(!retrieveImagesTimer->isActive()){

        //disable checkboxes
        ui->showVideosCheckBox->setEnabled(false);
        ui->saveVideoCheckBox->setEnabled(false);

        //change buttont text
        ui->startStopRecording->setText(QString::fromStdString("Stoppe Aufnahme"));

        //reset VideoWriter if used
        if(saveVideo) HelperFunctions().createVideoWriter(writerLeft, writerRight);

        //start grabbing images from cameras
        cameras.StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByUser);

        //start timer. Display timer started in update.
        retrieveImagesTimer->start();
        overallTimer.start();
    }
    else{
        //Stop videoWriter
        writerLeft.release();
        writerRight.release();

        //stop retrieving images
        retrieveImagesTimer->stop();
        int i = overallTimer.elapsed();

        //stop grabbing images from cameras
        cameras.StopGrabbing();

        //calculate framerate of this run
        double seconds = i / 1000;
        double fpsPerSecond = executionCounter/seconds;
        std::cout << "update has been executed " << executionCounter << " times in " << seconds << "seconds." << endl;
        std::cout << "This equals to " << fpsPerSecond << " frames per second." << endl;

        double convertTimeAverage = convertTimeOverall/executionCounter;
        double displayTimeAverage = displayTimeOverall/executionCounter;
        double saveTimeAverage = saveTimeOverall/executionCounter;
        double completeTimeAverage = completeTimeOverall/executionCounter;

        std::cout << "average convert time was " << convertTimeAverage << " ms" << endl;
        std::cout << "average display time was " << displayTimeAverage << " ms" << endl;
        std::cout << "average save time was " << saveTimeAverage << " ms" << endl;
        std::cout << "average complete time was " << completeTimeAverage << " ms" << endl;

        //reset variables for framerate calculation
        executionCounter = 0;
        convertTimeOverall = 0;
        displayTimeOverall = 0;
        saveTimeOverall = 0;
        completeTimeOverall = 0;

        //enable checkboxes
        ui->showVideosCheckBox->setEnabled(true);
        ui->saveVideoCheckBox->setEnabled(true);

        //change button text
        ui->startStopRecording->setText(QString::fromStdString("Starte Videoaufnahme"));
    }

}

void CameraBased::on_showVideosCheckBox_toggled(bool checked)
{
    //toggle show video boolean
    this->showVideo = checked;
}

void CameraBased::on_saveVideoCheckBox_toggled(bool checked)
{
    //toggle save video boolean
    this->saveVideo = checked;
}

void CameraBased::initPylon(){
    // Get the transport layer factory.
    CTlFactory& tlFactory = CTlFactory::GetInstance();

    // Get all attached devices and exit application if no device is found.
    DeviceInfoList_t devices;

    if(tlFactory.EnumerateDevices(devices) < 2){
        throw RUNTIME_EXCEPTION("Less than two camera found");
    }

    //set converter parameters
    formatConverter.OutputPixelFormat = PixelType_RGB8packed;
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
        // Print the model name of the camera.
        cout << "Using device "
             << cameras[i].GetDeviceInfo().GetModelName() << " | "
             << cameras[i].GetDeviceInfo().GetDeviceID() << endl;
    }

    cameras.Open();
}

void CameraBased::retrieveImages(){
    QElapsedTimer timer;
    timer.start();
    try{
        cameras[0].RetrieveResult(15, pylonResultLeft, TimeoutHandling_ThrowException);
        cameras[1].RetrieveResult(15, pylonResultRight, TimeoutHandling_ThrowException);

        if(cameras.IsGrabbing() && pylonResultLeft->GrabSucceeded() && pylonResultRight->GrabSucceeded()){
            cv::Mat imgLeft, imgRight;

            //convert images
            formatImages(imgLeft, imgRight);

            //display images
            displayImages(imgLeft, imgRight);

            //save images
            saveImages(imgLeft, imgRight);
        }
    }
    catch (const GenericException &e)
    {
        // Error handling
        cerr << "An exception occurred in update." << endl
        << e.GetDescription() << endl;
        cout << "after update in catch " << timer.elapsed() << "ms" << endl <<
                "-------------------------------------------------" << endl;
    }
    cout << "complete run of retrieve takes " << timer.elapsed() << "ms" << endl <<
            "-------------------------------------------------" << endl;
    completeTimeOverall += timer.elapsed();
    executionCounter++;
}

void CameraBased::formatImages(cv::Mat& imgLeft, cv::Mat& imgRight){
    QElapsedTimer timer;
    timer.start();
    std::thread leftT([&] {imgLeft = formatLeft();});
    std::thread rightT([&] {imgRight = formatRight();});
    leftT.join();
    rightT.join();
    cout << "time for format " << timer.elapsed() << "ms" << endl;
    convertTimeOverall += timer.elapsed();;
}

cv::Mat CameraBased::formatLeft(){
    formatConverter.Convert(pylonImageLeft, pylonResultLeft);
    cv::Mat imgLeft(pylonImageLeft.GetHeight(), pylonImageLeft.GetWidth(), CV_8UC3, (uint8_t*)pylonImageLeft.GetBuffer());
    return imgLeft;
}

cv::Mat CameraBased::formatRight(){
    formatConverter.Convert(pylonImageRight, pylonResultRight);
    cv::Mat imgRight(pylonImageRight.GetHeight(), pylonImageRight.GetWidth(), CV_8UC3, (uint8_t*)pylonImageRight.GetBuffer());
    cv::rotate(imgRight, imgRight, cv::ROTATE_180);
    return imgRight;
}

void CameraBased::displayImages(cv::Mat imgLeft, cv::Mat imgRight){
    if(this->showVideo){
        QElapsedTimer timer;
        timer.start();
        //Resize Images
        cv::resize(imgLeft, imgLeft, cv::Size(480, 320), 0, 0);
        cv::resize(imgRight, imgRight, cv::Size(480, 320), 0, 0);

        //Convert to QImage
        QImage qimgLeft((const unsigned char*) imgLeft.data, imgLeft.cols, imgLeft.rows, QImage::Format_RGB888);
        QImage qimgRight((const unsigned char*) imgRight.data, imgRight.cols, imgRight.rows, QImage::Format_RGB888);

        //Display on Input Label
        ui->videoLabelLeft->setPixmap(QPixmap::fromImage(qimgLeft));
        ui->videoLabelRight->setPixmap(QPixmap::fromImage(qimgRight));

        //Resize the label to fit the image
        ui->videoLabelLeft->resize(ui->videoLabelLeft->pixmap()->size());
        ui->videoLabelRight->resize(ui->videoLabelRight->pixmap()->size());

        std::cout << "time for displaying " << timer.elapsed() << " ms" << endl;
        displayTimeOverall += timer.elapsed();
    }
}

void CameraBased::saveImages(cv::Mat imgLeft, cv::Mat imgRight){
    if(this->saveVideo){
        QElapsedTimer timer;
        timer.start();
        std::thread saveLeftThread(&CameraBased::saveLeft, this, imgLeft);
        std::thread saveRightThread(&CameraBased::saveRight, this, imgRight);
        saveLeftThread.join();
        saveRightThread.join();
        std::cout << "time for saving " << timer.elapsed() << " ms" << endl;
        saveTimeOverall += timer.elapsed();
    }
}

void CameraBased::saveLeft(cv::Mat imgLeft){
    cv::cvtColor(imgLeft, imgLeft, CV_BGR2RGB);
    writerLeft.write(imgLeft);
}

void CameraBased::saveRight(cv::Mat imgRight){
    cv::cvtColor(imgRight, imgRight, CV_BGR2RGB);
    writerRight.write(imgRight);
}

void CameraBased::displayImageLeft(cv::Mat imgLeft){
    //Resize Images
    cv::resize(imgLeft, imgLeft, cv::Size(480, 320), 0, 0);

    //Change to RGB format & save it in global Mat
    cv::cvtColor(imgLeft, imgLeft, CV_BGR2RGB);

    //Convert to QImage
    QImage qimgLeft((const unsigned char*) imgLeft.data, imgLeft.cols, imgLeft.rows, QImage::Format_RGB888);

    //Display on Input Label
    ui->videoLabelLeft->setPixmap(QPixmap::fromImage(qimgLeft));

    //Resize the label to fit the image
    ui->videoLabelLeft->resize(ui->videoLabelLeft->pixmap()->size());
}

void CameraBased::displayImageRight(cv::Mat imgRight){
    //Resize Images
    cv::resize(imgRight, imgRight, cv::Size(480, 320), 0, 0);

    //Change to RGB format & save it in global Mat
    cv::cvtColor(imgRight, imgRight, CV_BGR2RGB);

    //Convert to QImage
    QImage qimgRight((const unsigned char*) imgRight.data, imgRight.cols, imgRight.rows, QImage::Format_RGB888);

    //Display on Input Label
    ui->videoLabelRight->setPixmap(QPixmap::fromImage(qimgRight));

    //Resize the label to fit the image
    ui->videoLabelRight->resize(ui->videoLabelRight->pixmap()->size());
}
