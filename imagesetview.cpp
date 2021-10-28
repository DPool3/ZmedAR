#include "imagesetview.h"
#include "ui_imagesetview.h"

/**
 * @brief ImageSetView::ImageSetView
 * @param parent
 */
ImageSetView::ImageSetView(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ImageSetView)
{
    ui->setupUi(this);
    displayImagesTimer = new QTimer(this);
    displayImagesTimer->setInterval(1000/videoManager.fps);
    connect(displayImagesTimer, SIGNAL(timeout()), this, SLOT(displayImages()));
}

/**
 * @brief ImageSetView::~ImageSetView
 */
ImageSetView::~ImageSetView()
{
    delete ui;
}

/**
 * @brief ImageSetView::lockReleaseUi
 * @param checked
 */
void ImageSetView::lockReleaseUi(bool checked)
{
    ui->showVideosCheckBox->setEnabled(checked);
    ui->setImageSet_checkBox->setEnabled(checked);
    ui->numberImages_spinBox->setEnabled(checked);
    ui->rows_spinBox->setEnabled(checked);
    ui->columns_spinBox->setEnabled(checked);
    ui->squareSize_SpinBox->setEnabled(checked);
    ui->chessboard_radioButton->setEnabled(checked);
    ui->circle_radioButton->setEnabled(checked);
    ui->asymCircles_radioButton->setEnabled(checked);
}

/**
 * @brief ImageSetView::on_showVideosCheckBox_toggled
 * @param checked
 */
void ImageSetView::on_showVideosCheckBox_toggled(bool checked)
{
    this->showVideo = checked;
}

/**
 * @brief ImageSetView::on_setImageSet_checkBox_toggled
 * @param checked
 */
void ImageSetView::on_setImageSet_checkBox_toggled(bool checked)
{
    //set standard image set for easier use in stereo camera calibration
}

/**
 * @brief ImageSetView::on_imageSetRecord_button_clicked
 */
void ImageSetView::on_imageSetRecord_button_clicked()
{
    if(this->showVideo && !this->displayImagesTimer->isActive()){
        lockReleaseUi(false);

        imageSetController.setNumberColumns(ui->columns_spinBox->value());
        imageSetController.setNumberRows(ui->rows_spinBox->value());
        imageSetController.setNumberOfImages(ui->numberImages_spinBox->value());
        imageSetController.setPatternType(getPatternType());
        imageSetController.setSquareSize(ui->squareSize_SpinBox->value());

        this->iteration = 0;

        if(!imageSetController.startRecording()){
            lockReleaseUi(true);
            return;
        }

        this->displayImagesTimer->start();

        startButtonCounter(ui->numberImages_spinBox->value());
    }
    else if(this->displayImagesTimer->isActive() && this->iteration <= ui->numberImages_spinBox->value()) {
        imageSetController.takeImage();
        startButtonCounter(ui->numberImages_spinBox->value());
    }

    if(iteration > ui->numberImages_spinBox->value()){
        this->displayImagesTimer->stop();
        imageSetController.stopRecording();
        ui->imageSetRecord_button->setText("Aufnahme Starten");
        lockReleaseUi(true);
    }
}

std::string ImageSetView::getPatternType()
{
    if(ui->chessboard_radioButton->isChecked()){
        return "chessboard";
    }
    else if(ui->asymCircles_radioButton->isChecked()){
        return "circle asymmetrical";
    }
    else{
        return "circle";
    }
}

void ImageSetView::startButtonCounter(int maxNumberImages){
    std::string currentAmount = std::to_string(iteration);
    std::string goalAmount = std::to_string(maxNumberImages);
    std::string startButtonString = currentAmount + "/" + goalAmount;
    ui->imageSetRecord_button->setText(QString::fromStdString(startButtonString));
    iteration++;
}

void ImageSetView::displayImages()
{
    if(!imageSetController.isRunning()){
        this->displayImagesTimer->stop();
        ui->imageSetRecord_button->setText("Aufnahme Starten");
        lockReleaseUi(true);
        return;
    }

    if(this->showVideo && aquireProcessedImages()){
        //Display on Input Label
        ui->videoLabelLeft->setPixmap(QPixmap::fromImage(this->imageLeft));
        ui->videoLabelRight->setPixmap(QPixmap::fromImage(this->imageRight));

        //Resize the label to fit the image
        ui->videoLabelLeft->resize(ui->videoLabelLeft->pixmap()->size());
        ui->videoLabelRight->resize(ui->videoLabelRight->pixmap()->size());
    }
}

bool ImageSetView::aquireProcessedImages()
{
    return imageSetController.getProcessedImages(this->imageLeft, this->imageRight);
}
