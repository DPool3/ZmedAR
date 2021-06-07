#include "camerabasedview.h"
#include "ui_camerabasedview.h"

CameraBasedView::CameraBasedView(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CameraBasedView)
{
    ui->setupUi(this);

    displayImagesTimer = new QTimer(this);
    displayImagesTimer->setInterval(1000/videoManager.fps);
    connect(displayImagesTimer, SIGNAL(timeout()), this, SLOT(displayImages()));
}

CameraBasedView::~CameraBasedView()
{
    if(this->displayImagesTimer->isActive())
        this->displayImagesTimer->stop();

    delete ui;
}

void CameraBasedView::on_showVideosCheckBox_toggled(bool checked)
{
    this->display = checked;
}

void CameraBasedView::on_saveVideoButton_clicked()
{
    cbc.startStopRecording();

    if(cbc.isRecording()){
        ui->saveVideoButton->setText("Speichert Video..");
    }
    else{
        ui->saveVideoButton->setText("Video speichern");
    }
}

void CameraBasedView::on_startStopRecording_clicked()
{
    cbc.startStopCameraBasedProcess();

    if(this->display && !this->displayImagesTimer->isActive())
        this->displayImagesTimer->start();
    else
        this->displayImagesTimer->stop();

    if(cbc.isRunning())
        ui->startStopRecording->setText("Aufnahme läuft..");
    else
        ui->startStopRecording->setText("Aufnahme Starten");

}

void CameraBasedView::on_doubleSpinBox_valueChanged(double arg1)
{
    cbc.setExposure(arg1);
}

void CameraBasedView::on_doubleSpinBox_2_valueChanged(double arg1)
{
    cbc.setBrightness(arg1);
}

void CameraBasedView::on_doubleSpinBox_3_valueChanged(double arg1)
{
    cbc.setSaturation(arg1);
}

void CameraBasedView::aquireProcessedImages(){
    cbc.getProcessedImages(this->imageLeft, this->imageRight);
}

void CameraBasedView::displayImages(){
    if(!cbc.isRunning()){
        this->displayImagesTimer->stop();
        ui->startStopRecording->setText("Aufnahme Starten");
        ui->saveVideoButton->setText("Video speichern");
        return;
    }

    aquireProcessedImages();

    //Display on Input Label
    ui->videoLabelLeft->setPixmap(QPixmap::fromImage(this->imageLeft));
    ui->videoLabelRight->setPixmap(QPixmap::fromImage(this->imageRight));

    //Resize the label to fit the image
    ui->videoLabelLeft->resize(ui->videoLabelLeft->pixmap()->size());
    ui->videoLabelRight->resize(ui->videoLabelRight->pixmap()->size());
}
