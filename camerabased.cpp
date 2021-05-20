#include "camerabased.h"
#include "ui_camerabased.h"

CameraBased::CameraBased(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CameraBased)
{
    ui->setupUi(this);

    //set update timer
    retrieveImagesTimer = new QTimer(this);
    retrieveImagesTimer->setInterval(1000/videoManager.fps);
    connect(retrieveImagesTimer, SIGNAL(timeout()), this, SLOT(retrieveImages()));

    saveImageTimer = new QTimer(this);
    saveImageTimer->setInterval(1000/videoManager.fps);
    connect(saveImageTimer, SIGNAL(timeout()), this, SLOT(saveImages()));
}

CameraBased::~CameraBased()
{
    cameras.stopGrabbing();
    delete ui;
}

void CameraBased::on_startStopRecording_clicked()
{
    //Initialize recording and displaying and start timer if not active
    if(!retrieveImagesTimer->isActive()){
        start();
    }
    else{
        stop();
    }
}

void CameraBased::on_showVideosCheckBox_toggled(bool checked)
{
    //toggle show video boolean
    this->showVideo = checked;
}

void CameraBased::on_saveVideoButton_clicked()
{
    if(saveImageTimer->isActive()){
        stopSave();
    }
    else{
        startSave();
    }
}

void CameraBased::startSave(){
    if(!(this->selectedDisplayMethod == 1) && this->retrieveImagesTimer->isActive()){
        ui->saveVideoButton->setText("Speichern läuft..");
        videoManager.createVideoWriterPair(writerLeft, writerRight, "", videoWriterFps);
        saveImageTimer->start();
    }
}

void CameraBased::stopSave(){
    ui->saveVideoButton->setText("Videos speichern");
    saveImageTimer->stop();
    videoManager.releaseVideoWriterPair(writerLeft, writerRight);
}

void CameraBased::on_normalVideoRadioButton_clicked()
{
    stopSave();
    videoWriterFps = 30;
    selectedDisplayMethod = -1;
}

void CameraBased::on_cannyEdgeRadioButton_clicked()
{
    stopSave();
    selectedDisplayMethod = 1;
}

void CameraBased::on_keyPointsRadioButton_clicked()
{
    stopSave();
    videoWriterFps = 10;
    selectedDisplayMethod = 2;
}

void CameraBased::lockUi(){
    //disable checkboxes
    ui->showVideosCheckBox->setEnabled(false);

    //change buttont text
    ui->startStopRecording->setText(QString::fromStdString("Stoppe Aufnahme"));
}

void CameraBased::releaseUi(){
    //enable checkboxes
    ui->showVideosCheckBox->setEnabled(true);

    //change button text
    ui->startStopRecording->setText(QString::fromStdString("Starte Videoaufnahme"));
}

void CameraBased::start(){

    //Initialize cameras
    try{
        cameras.initCameras();
    }
    catch (const exception &e)
    {
        std::string exceptionMsg = e.what();
        std::string errMsg = "Error: Es gabe einen Fehler während der Inititalisierung der Kameras.\n\"" + exceptionMsg + "\"";
        DialogManager().callErrorDialog(errMsg);
        return;
    }

    //start grabbing images of the cameras
    cameras.startGrabbing();

    lockUi();

    imageProcessor = ImageProcessor();

    errorCounter = 0;
    //start timer. Display timer started in update.
    retrieveImagesTimer->start();
    overallTimer.start();
}

void CameraBased::stop(){

    stopSave();

    //stop retrieving images -> stops calling grab function
    retrieveImagesTimer->stop();

    cameras.stopGrabbing();

    //clear last images
    this->imageLeft.release();
    this->imageRight.release();

    calcTimes();

    releaseUi();
}

void CameraBased::retrieveImages(){
    QElapsedTimer timer;
    timer.start();

    try{
        if(cameras.grabImages(this->imageLeft, this->imageRight)){
            displayImages(this->imageLeft, this->imageRight);
        }
        else{
            cerr << "No Image could be grabed" << endl;
            return ;
        }
    }catch(std::invalid_argument& e){
        if(errorCounter >= 100){
            DialogManager().callErrorDialog(e.what());
            stop();
            return;
        }
        else
            errorCounter++;
    }catch(std::exception& e){
        DialogManager().callErrorDialog(e.what());
        stop();
        cerr << e.what() << endl;
        return;
    }

    cout << "complete run of retrieve takes " << timer.elapsed() << "ms" << endl <<
                "-------------------------------------------------" << endl;
    completeTimeOverall += timer.elapsed();
    executionCounter++;
}

void CameraBased::displayImages(cv::Mat imgLeft, cv::Mat imgRight){
    QElapsedTimer timer;
    timer.start();

    QImage qLeft, qRight;

    switch (selectedDisplayMethod) {
        case 1:{
            imageProcessor.cannyEdgeOnImagePair(imgLeft, imgRight);
            qLeft = imageProcessor.prepImageForDisplay(imgLeft, "GRAY");
            qRight = imageProcessor.prepImageForDisplay(imgRight, "GRAY");
            break;
        }
        case 2:{
            imageProcessor.stereoVisualOdometry(imgLeft, imgRight);
            this->saveImageLeft = imgLeft;
            this->saveImageRight = imgRight;
            qLeft = imageProcessor.prepImageForDisplay(imgLeft, "BGR2RGB");
            qRight = imageProcessor.prepImageForDisplay(imgRight, "BGR2RGB");
            break;
        }
        default:{
            this->saveImageLeft = imgLeft;
            this->saveImageRight = imgRight;
            qLeft = imageProcessor.prepImageForDisplay(imgLeft, "BGR2RGB");
            qRight = imageProcessor.prepImageForDisplay(imgRight, "BGR2RGB");
            break;
        }
    }

    if(this->showVideo){
        //Display on Input Label
        ui->videoLabelLeft->setPixmap(QPixmap::fromImage(qLeft));
        ui->videoLabelRight->setPixmap(QPixmap::fromImage(qRight));

        //Resize the label to fit the image
        ui->videoLabelLeft->resize(ui->videoLabelLeft->pixmap()->size());
        ui->videoLabelRight->resize(ui->videoLabelRight->pixmap()->size());
    }

    std::cout << "time for displaying " << timer.elapsed() << " ms" << endl;
    displayTimeOverall += timer.elapsed();
}

void CameraBased::saveImages(){
    if(!this->saveImageLeft.empty() && !this->saveImageRight.empty())
        videoManager.saveImages(this->saveImageLeft, this->saveImageRight, writerLeft, writerRight);
}

void CameraBased::calcTimes(){
    int i = overallTimer.elapsed();

    //calculate framerate of this run
    double seconds = i / 1000;
    double fpsPerSecond = executionCounter/seconds;
    std::cout << "update has been executed " << executionCounter << " times in " << seconds << "seconds." << endl;
    std::cout << "This equals to " << fpsPerSecond << " frames per second." << endl;

    double displayTimeAverage = displayTimeOverall/executionCounter;
    //double saveTimeAverage = saveTimeOverall/saveExecutionCounter;
    double completeTimeAverage = completeTimeOverall/executionCounter;

    std::cout << "average fromat time was " << cameras.getAverageFormatTime() << " ms" << endl;
    std::cout << "average display time was " << displayTimeAverage << " ms" << endl;
    //std::cout << "average save time was " << saveTimeAverage << " ms" << endl;
    std::cout << "average complete time was " << completeTimeAverage << " ms" << endl;

    //reset variables for framerate calculation
    executionCounter = 0;
    saveExecutionCounter = 0;
    displayTimeOverall = 0;
    saveTimeOverall = 0;
    completeTimeOverall = 0;
}
