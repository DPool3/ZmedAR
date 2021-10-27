#include "stereocalibrationview.h"
#include "ui_stereocalibrationview.h"

/**
 * @brief StereoCalibrationView::StereoCalibrationView erstellt die Benutzeroberfläsche
 * und den displayTimer.
 * @param parent
 */
StereoCalibrationView::StereoCalibrationView(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::StereoCalibrationView)
{
    ui->setupUi(this);

    displayImageTimer = new QTimer(this);
    connect(displayImageTimer, SIGNAL(timeout()), this, SLOT(displayImages()));
}

/**
 * @brief StereoCalibrationView::~StereoCalibrationView ist der Destruktor.
 */
StereoCalibrationView::~StereoCalibrationView()
{
    delete ui;
}

/**
 * @brief StereoCalibrationView::on_searchFile_button_clicked öffnet den
 * Dateiexplorer, für die Suche nach einem ImageSet. Führt anschließend
 * das Laden des ImageSets aus.
 */
void StereoCalibrationView::on_searchFile_button_clicked()
{
    if(!stereoCalibrationController.loadImageSet())
        DialogManager().callErrorDialog("Es konnte kein Image Set geladen werden, weil kein Pfad angegeben wurde.");
    else{
        ui->lineEdit->setText(stereoCalibrationController.getImageSetPath());
        loadCalibrationInfo();
    }
}

/**
 * @brief StereoCalibrationView::on_startCalibration_button_clicked ruft
 * start und stop der Stereokamerakalibrierung auf, abhängig davon
 * ob Sie gerade läuft oder nicht.
 */
void StereoCalibrationView::on_startCalibration_button_clicked()
{
    if(!displayImageTimer->isActive())
        startCameraCalibration();
    else
        stopCameraCalibration();
}

/**
 * @brief StereoCalibrationView::on_displayImages_checkbox_toggled setzt,
 * ob die Videos dargestellt werden sollen oder nicht.
 * @param checked
 */
void StereoCalibrationView::on_displayImages_checkbox_toggled(bool checked)
{
    this->displayImagesBool = checked;
}

/**
 * @brief StereoCalibrationView::startCameraCalibration Führt den Start
 * der Stereokamerakalibrierung durch indem es die UI sperrt und den
 * Prozess startet.
 */
void StereoCalibrationView::startCameraCalibration(){

    lockReleaseUi(false);

    if(!stereoCalibrationController.startStereoCalibration()){
        lockReleaseUi(true);
    }
    else
        displayImageTimer->start();
}

/**
 * @brief StereoCalibrationView::stopCameraCalibration führt den stop des
 * Prozesses aus, indem es den Timer stoppt, die berechneten Infos lädt
 * und die UI wieder freigibt.
 */
void StereoCalibrationView::stopCameraCalibration(){
    displayImageTimer->stop();

    loadCalibrationInfo();

    lockReleaseUi(true);
}

/**
 * @brief StereoCalibrationView::lockReleaseUi gibt UI frei oder sperrt sie
 * in Abhängigkeit des vorherigen Zustands.
 * @param checked
 */
void StereoCalibrationView::lockReleaseUi(bool checked){
    if(checked)
        ui->startCalibration_button->setText(QString::fromStdString("Stereokalibrierung starten"));
    else
        ui->startCalibration_button->setText(QString::fromStdString("Stereokalibrierung läuft"));

    ui->startCalibration_button->setEnabled(checked);
    ui->displayImages_checkbox->setEnabled(checked);
    ui->searchFile_button->setEnabled(checked);
}

/**
 * @brief StereoCalibrationView::loadCalibrationInfo fragt die
 * Kalibrierungsinformationen vom stereoCalibrationController ab und stellt
 * diese in der UI dar.
 */
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

/**
 * @brief StereoCalibrationView::displayImages stellt die Bilder in der UI dar,
 * falls es neue gibt, die dargestellt werden können.
 */
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
