#include "videobasedview.h"
#include "ui_videobasedview.h"

VideoBasedView::VideoBasedView(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::VideoBasedView)
{
    ui->setupUi(this);

    displayImagesTimer = new QTimer(this);
    connect(displayImagesTimer, SIGNAL(timeout()), this, SLOT(displayImages()));
}

VideoBasedView::~VideoBasedView()
{
    delete ui;
}

void VideoBasedView::on_startVideo_button_clicked()
{
    if(!displayImagesTimer->isActive()){
        start();
    }
    else{
        stop();
    }
}

void VideoBasedView::start(){
    //Notwendig, bei jedem neuen Video, weil ImageProcessor Bilder zwischenspeichert.
    try{
        vbc.reinitVideoBasedController(ui->detectorSpinBox->value(), ui->descriptorSpinBox->value(), ui->matcherSpinBox->value());
    }catch(std::exception& e){
        //exception for wrong combination
        return;
    }

    lockOrReleaseUi(false);
    std::cout << "video recorded with " << vbc.getfps() << "fps." << std::endl;
    timer.restart();
    iterationCounter = 0;
    displayImagesTimer->setInterval(1000/vbc.getfps());
    displayImagesTimer->start();
}

void VideoBasedView::stop(){
    displayImagesTimer->stop();
    double time = timer.elapsed();
    std::cout << "display executed " << iterationCounter << " times in " << time/1000 << "Seconds. Which results in " << iterationCounter/(time/1000) << "fps." << std::endl;
    lockOrReleaseUi(true);
}

void VideoBasedView::lockOrReleaseUi(bool isEnabled){
    ui->showVideosCheckBox->setEnabled(isEnabled);
    ui->cameraTracking_CheckBox->setEnabled(isEnabled);
    ui->videoFilePathLeft_QLineEdit->setEnabled(isEnabled);
    ui->videoFilePathRight_QLineEdit->setEnabled(isEnabled);
    ui->searchFile_button->setEnabled(isEnabled);
    ui->searchFile_button_2->setEnabled(isEnabled);
    ui->detectorSpinBox->setEnabled(isEnabled);
    ui->descriptorSpinBox->setEnabled(isEnabled);
    ui->matcherSpinBox->setEnabled(isEnabled);
}

void VideoBasedView::on_showVideosCheckBox_toggled(bool checked)
{
    this->displayVideos = checked;
}

void VideoBasedView::on_cameraTracking_CheckBox_toggled(bool checked)
{
    vbc.useCameraTracking = checked;
}

void VideoBasedView::on_searchFile_button_2_clicked()
{
    QString rightPath = dialogManager.getPathFromFileSystem();
    vbc.videoPathRight = rightPath.toStdString();
    ui->videoFilePathRight_QLineEdit->setText(rightPath);
}

void VideoBasedView::on_searchFile_button_clicked()
{
    QString leftPath = dialogManager.getPathFromFileSystem();
    vbc.videoPathLeft = leftPath.toStdString();
    ui->videoFilePathLeft_QLineEdit->setText(leftPath);
}

void VideoBasedView::aquireProcessedImages(){
    if(!vbc.getProcessedImages(this->imageLeft, this->imageRight)){
        stop();
    }
}

void VideoBasedView::displayImages(){
    aquireProcessedImages();

    if(displayVideos){
        //Display on Input Label
        ui->videoLabelLeft->setPixmap(QPixmap::fromImage(this->imageLeft));
        ui->videoLabelRight->setPixmap(QPixmap::fromImage(this->imageRight));

        //Resize the label to fit the image
        ui->videoLabelLeft->resize(ui->videoLabelLeft->pixmap()->size());
        ui->videoLabelRight->resize(ui->videoLabelRight->pixmap()->size());
    }

    iterationCounter++;
}
