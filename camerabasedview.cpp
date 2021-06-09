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

void CameraBasedView::on_viveTracking_Checkbox_toggled(bool checked)
{
    cbc.useViveTracking = checked;
}

void CameraBasedView::on_cameraTracking_CheckBox_toggled(bool checked)
{
    cbc.useCameraTracking = checked;
}

void CameraBasedView::on_saveVideo_CheckBox_toggled(bool checked)
{
    cbc.startStopRecording();
}

void CameraBasedView::on_startStopRecording_clicked()
{
    cbc.startStopCameraBasedProcess();

    if(this->display && !this->displayImagesTimer->isActive()){
        lockAndReleaseUI(false);
        iterationCounter = 0;
        timer.start();
        this->displayImagesTimer->start();
    }
    else{
        this->displayImagesTimer->stop();
        double time = timer.elapsed();
        std::cout << "display executed " << iterationCounter << " times in " << time/1000 << "Seconds. Which results in " << iterationCounter/(time/1000) << "fps." << std::endl;
        lockAndReleaseUI(true);
    }

    if(cbc.isRunning())
        ui->startStopRecording->setText("Aufnahme läuft..");
    else
        ui->startStopRecording->setText("Aufnahme Starten");

}

void CameraBasedView::on_brightness_button_clicked()
{
    double value = ui->brightness_SpinBox->value();
    if(value >= -1 && value <= 1)
        cbc.setBrightness(value);
}

void CameraBasedView::on_contrast_button_clicked()
{
    double value = ui->contrast_SpinBox->value();
    if(value >= -1 && value <= 1)
        cbc.setContrast(value);
}

void CameraBasedView::on_exposure_button_clicked()
{
    double value = ui->exposure_SpinBox->value();
    if(value >= 1 && value <= 10000000)
        cbc.setExposure(value);
}

void CameraBasedView::on_saturation_button_clicked()
{
    double value = ui->saturation_SpinBox->value();
    if(value >= 0 && value <= 1)
        cbc.setSaturation(value);
}

void CameraBasedView::lockAndReleaseUI(bool enabled){
    ui->showVideosCheckBox->setEnabled(enabled);
    ui->saveVideo_CheckBox->setEnabled(enabled);
    ui->viveTracking_Checkbox->setEnabled(enabled);

    ui->brightness_button->setEnabled(!enabled);
    ui->contrast_button->setEnabled(!enabled);
    ui->exposure_button->setEnabled(!enabled);

    ui->brightness_SpinBox->setEnabled(!enabled);
    ui->contrast_SpinBox->setEnabled(!enabled);
    ui->exposure_SpinBox->setEnabled(!enabled);
}

bool CameraBasedView::aquireProcessedImages(){
    return cbc.getProcessedImages(this->imageLeft, this->imageRight);
}

void CameraBasedView::displayImages(){
    if(!cbc.isRunning()){
        this->displayImagesTimer->stop();
        ui->startStopRecording->setText("Aufnahme Starten");
        lockAndReleaseUI(true);
        return;
    }

    if( aquireProcessedImages() && this->display){
        //Display on Input Label
        ui->videoLabelLeft->setPixmap(QPixmap::fromImage(this->imageLeft));
        ui->videoLabelRight->setPixmap(QPixmap::fromImage(this->imageRight));

        //Resize the label to fit the image
        ui->videoLabelLeft->resize(ui->videoLabelLeft->pixmap()->size());
        ui->videoLabelRight->resize(ui->videoLabelRight->pixmap()->size());
    }

    iterationCounter++;
}
