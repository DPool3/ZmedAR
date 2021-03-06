#include "videobasedview.h"
#include "ui_videobasedview.h"

/**
 * @brief VideoBasedView::VideoBasedView initialisiert die Benutzeroberfläche
 * und den displayImageTimer.
 * @param parent
 */
VideoBasedView::VideoBasedView(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::VideoBasedView)
{
    ui->setupUi(this);

    displayImagesTimer = new QTimer(this);
    connect(displayImagesTimer, SIGNAL(timeout()), this, SLOT(displayImages()));
}

/**
 * @brief VideoBasedView::~VideoBasedView ist der Destruktor.
 */
VideoBasedView::~VideoBasedView()
{
    delete ui;
}

/**
 * @brief VideoBasedView::on_startVideo_button_clicked startet den
 * displayImageTimer, falls er noch nicht läuft und stoppt ihn, falls
 * er schon läuft.
 */
void VideoBasedView::on_startVideo_button_clicked()
{
    if(!displayImagesTimer->isActive()){
        start();
    }
    else{
        stop();
    }
}

/**
 * @brief VideoBasedView::start führt den Start aus. Dafür wird der
 * videoBasedController reinitialisiert, die UI gesperrt, der Timer
 * auf die fps Zahl des Videos eingestellt und der timer gestartet.
 */
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

/**
 * @brief VideoBasedView::stop stoppt den Prozessm
 * indem es den timer stoppt und die UI freigibt.
 */
void VideoBasedView::stop(){
    displayImagesTimer->stop();
    double time = timer.elapsed();
    std::cout << "display executed " << iterationCounter << " times in " << time/1000 << "Seconds. Which results in " << iterationCounter/(time/1000) << "fps." << std::endl;
    lockOrReleaseUi(true);
}

/**
 * @brief VideoBasedView::lockOrReleaseUi sperrt die UI oder gibt Sie frei,
 * in Abhängigkeit von dem vorherigen Zustand.
 * @param isEnabled
 */
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
    if(isEnabled){
        ui->startVideo_button->setText("Starte Videoverarbeitung");
    }
    else{
        ui->startVideo_button->setText("Video läuft ...");
    }
}

/**
 * @brief VideoBasedView::on_showVideosCheckBox_toggled setzt den bool
 * für die Darstellung der Bilder.
 * @param checked
 */
void VideoBasedView::on_showVideosCheckBox_toggled(bool checked)
{
    this->displayVideos = checked;
}

/**
 * @brief VideoBasedView::on_cameraTracking_CheckBox_toggled setzt den bool
 * für die Durchführung des Kameratrackings.
 * @param checked
 */
void VideoBasedView::on_cameraTracking_CheckBox_toggled(bool checked)
{
    vbc.useCameraTracking = checked;
}

/**
 * @brief VideoBasedView::on_searchFile_button_2_clicked führt den
 * Dateiexplorer, für die Suche nach einer Datei aus. Hier muss
 * das rechte Video gewählt werden.
 */
void VideoBasedView::on_searchFile_button_2_clicked()
{
    QString rightPath = dialogManager.getPathFromFileSystem();
    vbc.videoPathRight = rightPath.toStdString();
    ui->videoFilePathRight_QLineEdit->setText(rightPath);
}

/**
 * @brief VideoBasedView::on_searchFile_button_clicked führt den
 * Dateiexplorer, für die Suche nach einer Datei aus. Hier muss
 * das linke Video gewählt werden.
 */
void VideoBasedView::on_searchFile_button_clicked()
{
    QString leftPath = dialogManager.getPathFromFileSystem();
    vbc.videoPathLeft = leftPath.toStdString();
    ui->videoFilePathLeft_QLineEdit->setText(leftPath);
}

/**
 * @brief VideoBasedView::aquireProcessedImages fragt die Bilder aus
 * dem videoBasedController ab.
 */
void VideoBasedView::aquireProcessedImages(){
    if(!vbc.getProcessedImages(this->imageLeft, this->imageRight)){
        stop();
    }
}

/**
 * @brief VideoBasedView::displayImages stellt die Bilder in der
 * Benutzeroberfläche das, nachdem es diese mittels
 * aquireProcessedImages abgefragt hat.
 */
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
