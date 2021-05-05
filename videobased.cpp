#include "videobased.h"
#include "ui_videobased.h"

VideoBased::VideoBased(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::VideoBased)
{
    ui->setupUi(this);

    this->updateTimer = new QTimer(this);
    this->updateTimer->setInterval(1000/videoManager.saveFps);
    connect(this->updateTimer, SIGNAL(timeout()), this, SLOT(update()));
}

VideoBased::~VideoBased()
{
    delete ui;
}

void VideoBased::on_startVideo_button_clicked()
{
    if(!updateTimer->isActive()){
        start();
    }
    else {
        stop();
    }
}

void VideoBased::on_showVideosCheckBox_toggled(bool checked)
{
    this->showVideo = checked;
}

void VideoBased::on_searchFile_button_2_clicked()
{
    ui->videoFilePathRight_QLineEdit->setText(DialogManager().getPathFromFileSystem());
}

void VideoBased::on_searchFile_button_clicked()
{
    ui->videoFilePathLeft_QLineEdit->setText(DialogManager().getPathFromFileSystem());
}

void VideoBased::saveEnteredPaths(){
    this->videoPathLeft = ui->videoFilePathLeft_QLineEdit->text().toStdString();
    this->videoPathRight = ui->videoFilePathRight_QLineEdit->text().toStdString();
}

void VideoBased::releaseUi(){
    ui->showVideosCheckBox->setEnabled(true);
    ui->startVideo_button->setText(QString::fromStdString("Starte Videoverarbeitung"));
    ui->searchFile_button->setEnabled(true);
    ui->searchFile_button_2->setEnabled(true);
    ui->videoFilePathLeft_QLineEdit->setEnabled(true);
    ui->videoFilePathRight_QLineEdit->setEnabled(true);
}

void VideoBased::lockUi(){
    ui->showVideosCheckBox->setEnabled(false);
    ui->startVideo_button->setText(QString::fromStdString("Stoppe Videoverarbeitung"));
    ui->searchFile_button->setEnabled(false);
    ui->searchFile_button_2->setEnabled(false);
    ui->videoFilePathLeft_QLineEdit->setEnabled(false);
    ui->videoFilePathRight_QLineEdit->setEnabled(false);
}

void VideoBased::start(){
    lockUi();
    saveEnteredPaths();
    try{
        videoManager.createVideoCapturePair(this->captureLeft, this->captureRight, this->videoPathLeft, this->videoPathRight);
    }catch(const std::invalid_argument& e){
        DialogManager().callErrorDialog(e.what());
        stop();
        return;
    }
    updateTimer->start();
}

void VideoBased::stop(){
    updateTimer->stop();
    videoManager.releaseVideoCapturePair(this->captureLeft, this->captureRight);
    releaseUi();
}

void VideoBased::update(){
    this->captureLeft >> this->imageLeft;
    this->captureRight >> this->imageRight;

    if(imageLeft.empty() || imageRight.empty()){
        stop();
        return;
    }
    displayImages(this->imageLeft, this->imageRight);

    //do something with the images
}

void VideoBased::displayImages(cv::Mat imageLeft, cv::Mat imageRight){
    if(this->showVideo){
        //Resize Images

        //imageProcessor.cannyEdgeOnImagePair(imageLeft, imageRight);

        QImage qLeft = imageProcessor.prepImageForDisplay(imageLeft);
        QImage qRight = imageProcessor.prepImageForDisplay(imageRight);

        //Display on Input Label
        ui->videoLabelLeft->setPixmap(QPixmap::fromImage(qLeft));
        ui->videoLabelRight->setPixmap(QPixmap::fromImage(qRight));

        //Resize the label to fit the image
        ui->videoLabelLeft->resize(ui->videoLabelLeft->pixmap()->size());
        ui->videoLabelRight->resize(ui->videoLabelRight->pixmap()->size());
    }
}
