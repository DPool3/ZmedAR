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

void CameraBased::on_saveVideoCheckBox_toggled(bool checked)
{
    //toggle save video boolean
    this->saveVideo = checked;
}

void CameraBased::lockUi(){
    //disable checkboxes
    ui->showVideosCheckBox->setEnabled(false);
    ui->saveVideoCheckBox->setEnabled(false);

    //change buttont text
    ui->startStopRecording->setText(QString::fromStdString("Stoppe Aufnahme"));
}

void CameraBased::releaseUi(){
    //enable checkboxes
    ui->showVideosCheckBox->setEnabled(true);
    ui->saveVideoCheckBox->setEnabled(true);

    //change button text
    ui->startStopRecording->setText(QString::fromStdString("Starte Videoaufnahme"));
}

void CameraBased::start(){
    lockUi();

    imageProcessor = ImageProcessor();

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

    //start timer. Display timer started in update.
    retrieveImagesTimer->start();
    overallTimer.start();

    //reset VideoWriter if used
    if(saveVideo){
        videoManager.createVideoWriterPair(writerLeft, writerRight, "");
        saveImageTimer->start();
    }
}

void CameraBased::stop(){

    if(saveVideo){
        saveImageTimer->stop();
        videoManager.releaseVideoWriterPair(writerLeft, writerRight);
    }

    //stop retrieving images -> stops calling grab function
    retrieveImagesTimer->stop();

    //stop grabbing images of the cameras -> disables grabbing of the cameras internaly
    cameras.stopGrabbing();

    //clear last images
    this->imageLeft.release();
    this->imageRight.release();

    int i = overallTimer.elapsed();

    //calculate framerate of this run
    double seconds = i / 1000;
    double fpsPerSecond = executionCounter/seconds;
    std::cout << "update has been executed " << executionCounter << " times in " << seconds << "seconds." << endl;
    std::cout << "This equals to " << fpsPerSecond << " frames per second." << endl;

    double displayTimeAverage = displayTimeOverall/executionCounter;
    double saveTimeAverage = saveTimeOverall/saveExecutionCounter;
    double completeTimeAverage = completeTimeOverall/executionCounter;

    std::cout << "average fromat time was " << cameras.getAverageFormatTime() << " ms" << endl;
    std::cout << "average display time was " << displayTimeAverage << " ms" << endl;
    std::cout << "average save time was " << saveTimeAverage << " ms" << endl;
    std::cout << "average complete time was " << completeTimeAverage << " ms" << endl;

    //reset variables for framerate calculation
    executionCounter = 0;
    saveExecutionCounter = 0;
    displayTimeOverall = 0;
    saveTimeOverall = 0;
    completeTimeOverall = 0;

    releaseUi();
}

void CameraBased::retrieveImages(){
    QElapsedTimer timer;
    timer.start();

    try{
        if(cameras.grabImages(this->imageLeft, this->imageRight)){
            displayImages(this->imageLeft, this->imageRight);
        }
    }catch(std::exception& e){
        DialogManager().callErrorDialog(e.what());
        stop();
        return;
    }

    cout << "complete run of retrieve takes " << timer.elapsed() << "ms" << endl <<
                "-------------------------------------------------" << endl;
    completeTimeOverall += timer.elapsed();
    executionCounter++;
}

void CameraBased::displayImages(cv::Mat imgLeft, cv::Mat imgRight){
    if(this->showVideo){
        QElapsedTimer timer;
        timer.start();

        imageProcessor.stereoVisualOdometry(imgLeft, imgRight);

        QImage qLeft = imageProcessor.prepImageForDisplay(imgLeft);
        QImage qRight = imageProcessor.prepImageForDisplay(imgRight);

        //Display on Input Label
        ui->videoLabelLeft->setPixmap(QPixmap::fromImage(qLeft));
        ui->videoLabelRight->setPixmap(QPixmap::fromImage(qRight));

        //Resize the label to fit the image
        ui->videoLabelLeft->resize(ui->videoLabelLeft->pixmap()->size());
        ui->videoLabelRight->resize(ui->videoLabelRight->pixmap()->size());

        std::cout << "time for displaying " << timer.elapsed() << " ms" << endl;
        displayTimeOverall += timer.elapsed();
    }
}

void CameraBased::saveImages(){
    if(!this->imageLeft.empty() && !this->imageLeft.empty())
        videoManager.saveImages(this->imageLeft, this->imageRight, writerLeft, writerRight);
}
