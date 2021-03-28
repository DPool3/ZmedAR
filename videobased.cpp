#include "videobased.h"
#include "ui_videobased.h"

VideoBased::VideoBased(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::VideoBased)
{
    ui->setupUi(this);

    //Set frames per second in ms
    //const int fps = 1000/HelperFunctions().getFpsFromMainSettings();

    //set update timer
    this->updateTimer = new QTimer(this);
    this->updateTimer->setInterval(1000/11);
    connect(this->updateTimer, SIGNAL(timeout()), this, SLOT(update()));
}

VideoBased::~VideoBased()
{
    delete ui;
}

void VideoBased::on_startVideo_button_clicked()
{
    if(!updateTimer->isActive()){
        //disable checkbox
        ui->showVideosCheckBox->setEnabled(false);

        //change button text
        ui->startVideo_button->setText(QString::fromStdString("Stoppe Videoverarbeitung"));

        //save entered paths
        saveEnteredPaths();

        if(!checkFilePath(this->videoPathLeft) || !checkFilePath(this->videoPathRight)){
            //open error dialog with error message
            std::cout << "Error: Invalid path\n";
            return;
        }

        //initialize video reader left and right for saved paths
        this->captureLeft.open(this->videoPathLeft);
        HelperFunctions().checkVideoCaptureOpened(captureLeft);

        this->captureRight.open(this->videoPathRight);
        HelperFunctions().checkVideoCaptureOpened(captureRight);

        //start timer
        updateTimer->start();
    }
    else {
        //Stop timer
        updateTimer->stop();

        //stop video capture
        this->captureLeft.release();
        this->captureRight.release();

        //enable checkbox
        ui->showVideosCheckBox->setEnabled(true);

        //change button text
        ui->startVideo_button->setText(QString::fromStdString("Starte Videoverarbeitung"));
    }
}

void VideoBased::on_showVideosCheckBox_toggled(bool checked)
{
    this->showVideo = checked;
}

void VideoBased::on_searchFile_button_2_clicked()
{
    //search file and set line edit
    ui->videoFilePathRight_QLineEdit->setText(HelperFunctions().getPathFromFileSystem());
}

void VideoBased::on_searchFile_button_clicked()
{
    //search file and set line edit
    ui->videoFilePathLeft_QLineEdit->setText(HelperFunctions().getPathFromFileSystem());
}

void VideoBased::saveEnteredPaths(){
    this->videoPathLeft = ui->videoFilePathLeft_QLineEdit->text().toStdString();
    this->videoPathRight = ui->videoFilePathRight_QLineEdit->text().toStdString();
}

void VideoBased::update(){

    //read frames
    this->captureLeft >> this->imageLeft;
    this->captureRight >> this->imageRight;

    if(imageLeft.empty() || imageRight.empty()){
        this->updateTimer->stop();
        captureLeft.release();
        captureRight.release();
        ui->showVideosCheckBox->setEnabled(true);
        ui->startVideo_button->setText(QString::fromStdString("Starte Videoverarbeitung"));
        return;
    }

    //display image in ui
    displayImages(this->imageLeft, this->imageRight);

    //do something with the images
    //record and/or display images
}

void VideoBased::displayImages(cv::Mat imageLeft, cv::Mat imageRight){
    if(this->showVideo){
        //Resize Images
        cv::resize(imageLeft, imageLeft, cv::Size(480, 320), 0, 0);
        cv::resize(imageRight, imageRight, cv::Size(480, 320), 0, 0);

        //Change to RGB format & save it in global Mat
        cv::cvtColor(imageLeft, imageLeft, CV_BGR2RGB);
        cv::cvtColor(imageRight, imageRight, CV_BGR2RGB);

        //Convert to QImage
        QImage qimgLeft((const unsigned char*) imageLeft.data, imageLeft.cols, imageLeft.rows, QImage::Format_RGB888);
        QImage qimgRight((const unsigned char*) imageRight.data, imageRight.cols, imageRight.rows, QImage::Format_RGB888);

        //Display on Input Label
        ui->videoLabelLeft->setPixmap(QPixmap::fromImage(qimgLeft));
        ui->videoLabelRight->setPixmap(QPixmap::fromImage(qimgRight));

        //Resize the label to fit the image
        ui->videoLabelLeft->resize(ui->videoLabelLeft->pixmap()->size());
        ui->videoLabelRight->resize(ui->videoLabelRight->pixmap()->size());
    }
}

bool VideoBased::checkFilePath(std::string path){
    struct stat buffer;

    //check if empty
    bool exists = (stat (path.c_str(), &buffer) == 0);

    if(exists){
        return true;
    }

    //no valid path display error dialog
    HelperFunctions().callErrorDialog("Mindestens ein Dateipfad ist nicht valide. Bitte prüfen Sie die gewählten Pfade.");

    return false;
}
