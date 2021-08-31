#include "stereocalibrationview.h"
#include "ui_stereocalibrationview.h"

StereoCalibrationView::StereoCalibrationView(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::StereoCalibrationView)
{
    ui->setupUi(this);

    displayImageTimer = new QTimer(this);
    connect(displayImageTimer, SIGNAL(timeout()), this, SLOT(displayImages()));
}

StereoCalibrationView::~StereoCalibrationView()
{
    delete ui;
}

void StereoCalibrationView::on_searchFile_button_clicked()
{
    if(!stereoCalibrationController.loadImageSet())
        DialogManager().callErrorDialog("Es konnte kein Image Set geladen werden, weil kein Pfad angegeben wurde.");
    else{
        ui->lineEdit->setText(stereoCalibrationController.getImageSetPath());
        loadCalibrationInfo();
    }
}

void StereoCalibrationView::on_startCalibration_button_clicked()
{
    if(!displayImageTimer->isActive())
        startCameraCalibration();
    else
        stopCameraCalibration();
}

void StereoCalibrationView::on_displayImages_checkbox_toggled(bool checked)
{
    this->displayImagesBool = checked;
}

void StereoCalibrationView::startCameraCalibration(){

    lockReleaseUi(false);

    if(!stereoCalibrationController.startStereoCalibration()){
        lockReleaseUi(true);
    }
    else
        displayImageTimer->start();
}

void StereoCalibrationView::stopCameraCalibration(){
    displayImageTimer->stop();

    loadCalibrationInfo();

    lockReleaseUi(true);
}

void StereoCalibrationView::lockReleaseUi(bool checked){
    if(checked)
        ui->startCalibration_button->setText(QString::fromStdString("Stereokalibrierung starten"));
    else
        ui->startCalibration_button->setText(QString::fromStdString("Stereokalibrierung läuft"));

    ui->startCalibration_button->setEnabled(checked);
    ui->displayImages_checkbox->setEnabled(checked);
    ui->searchFile_button->setEnabled(checked);
}

void StereoCalibrationView::loadCalibrationInfo(){
    //load info from controller
    int board_width, board_hight, num_images;
    double stereoReprojectionError;
    float square_size;
    std::string patternType;

    stereoCalibrationController.getCalibrationInfo(board_width, board_hight, num_images, square_size, patternType, stereoReprojectionError);

    ui->numberImages_spinbox->setValue(num_images);
    ui->numbeRows_spinbox->setValue(board_hight);
    ui->numberColumns_spinbox->setValue(board_width);
    ui->squareSize_spinbox->setValue(square_size);
    ui->patternType_lineEdit->setText(QString::fromStdString(patternType));
    ui->reprojectionError_spinbox->setValue(stereoReprojectionError);
}

void StereoCalibrationView::displayImages(){
    if(displayImagesBool && stereoCalibrationController.checkNewImageForDisplay()){

        QImage qimgLeft, qimgRight;
        stereoCalibrationController.getImagesForDisplay(qimgLeft, qimgRight);

        //Display on Input Label
        ui->videoLabelLeft->setPixmap(QPixmap::fromImage(qimgLeft));
        ui->videoLabelRight->setPixmap(QPixmap::fromImage(qimgRight));

        //Resize the label to fit the image
        ui->videoLabelLeft->resize(ui->videoLabelLeft->pixmap()->size());
        ui->videoLabelRight->resize(ui->videoLabelRight->pixmap()->size());
    }

    if(!stereoCalibrationController.checkCalibrationRunning()){
        stopCameraCalibration();
    }
}
