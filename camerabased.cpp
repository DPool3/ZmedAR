#include "camerabased.h"
#include "ui_camerabased.h"

CameraBased::CameraBased(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CameraBased)
{
    ui->setupUi(this);

    //set update timer
    retrieveImagesTimer = new QTimer(this);
    retrieveImagesTimer->setInterval(1000/HelperFunctions().getFpsFromMainSettings());
    connect(retrieveImagesTimer, SIGNAL(timeout()), this, SLOT(retrieveImages()));

    saveImageTimer = new QTimer(this);
    saveImageTimer->setInterval(1000/HelperFunctions().getSaveFpsFromMainSettings());
    connect(saveImageTimer, SIGNAL(timeout()), this, SLOT(saveImagesThread()));
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

        //Initialize cameras
        try{
            cameras.initCameras();
        }
        catch (const GenericException &e)
        {
            // Error handling
            cerr << "An exception occurred during initialization of the cameras." << endl
            << e.GetDescription() << endl;
            return;
        }


        //disable checkboxes
        ui->showVideosCheckBox->setEnabled(false);
        ui->saveVideoCheckBox->setEnabled(false);

        //change buttont text
        ui->startStopRecording->setText(QString::fromStdString("Stoppe Aufnahme"));

        //reset VideoWriter if used
        if(saveVideo){
            //cameras.setCreateVideo(true);
            //cameras.initVideoWriter();
            HelperFunctions().createVideoWriter(writerLeft, writerRight);
            saveImageTimer->start();
        }

        //start grabbing images of the cameras
        cameras.startGrabbing();

        //start timer. Display timer started in update.
        retrieveImagesTimer->start();
        overallTimer.start();
    }
    else{
        //stop retrieving images -> stops calling grab function
        retrieveImagesTimer->stop();

        //stop grabbing images of the cameras -> disables grabbing of the cameras internaly
        cameras.stopGrabbing();

        if(saveVideo){
            //stop saving
            saveImageTimer->stop();

            //Stop videoWriter
            writerLeft.release();
            writerRight.release();

            //close video writer to save video
            //cameras.closeVideoWriter();
        }

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
        //std::cout << "average save time was " << cameras.getAverageSaveTime() << " ms" << endl;
        std::cout << "average complete time was " << completeTimeAverage << " ms" << endl;

        //reset variables for framerate calculation
        executionCounter = 0;
        saveExecutionCounter = 0;
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

void CameraBased::retrieveImages(){
    QElapsedTimer timer;
    timer.start();

    if(cameras.grabImages(this->imageLeft, this->imageRight)){
        displayImages(this->imageLeft, this->imageRight);
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

void CameraBased::saveImagesThread(){
    if(!this->imageLeft.empty() && !this->imageLeft.empty())
        saveImages(this->imageLeft, this->imageRight);
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
        saveExecutionCounter++;
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

void CameraBased::saveImagesWithPylon(){

}
