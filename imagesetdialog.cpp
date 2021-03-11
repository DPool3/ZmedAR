#include "imagesetdialog.h"
#include "ui_imagesetdialog.h"
using namespace Pylon;
using namespace std;

ImageSetDialog::ImageSetDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ImageSetDialog)
{
    ui->setupUi(this);

    PylonInitialize();

    try{
        initPylon();
    }
    catch (const GenericException &e)
    {
        // Error handling
        string text = "An exception occurred during initialization of the cameras.\n" + (string)e.GetDescription();
        HelperFunctions().callErrorDialog(text);
        this->close();
    }
    //HelperFunctions().setVideoCapture(this->captureLeft, this->captureRight);

    //Set frames per second in ms
    const int fps = 1000/HelperFunctions().getFpsFromMainSettings();

    //set update timer
    this->updateTimer = new QTimer(this);
    this->updateTimer->setInterval(fps);
    connect(this->updateTimer, SIGNAL(timeout()), this, SLOT(update()));

    //set counter timer
    this->counterTimer = new QTimer(this);
    this->counterTimer->setInterval(fps);
    connect(this->counterTimer, SIGNAL(timeout()), this, SLOT(counter()));
}

ImageSetDialog::~ImageSetDialog()
{
    cameras.StopGrabbing();
    delete ui;
}

//Methods for ui
void ImageSetDialog::on_showVideosCheckBox_toggled(bool checked)
{
    this->showVideo = checked;
}

void ImageSetDialog::on_setImageSet_checkBox_toggled(bool checked)
{
    this->setImageSetDefault = checked;
}

void ImageSetDialog::on_imageSetRecord_button_clicked()
{
    if(!this->counterTimer->isActive()){

        //disable ui
        enableUI(false);

        //create new Image set
        this->imageSet = createImageSet();

        //save all necessary info in the image set
        saveInputInImageSet();

        try{
            //start grabbing images from cameras
            cameras.StartGrabbing(GrabStrategy_LatestImageOnly, GrabLoop_ProvidedByUser);
        }
        catch (const GenericException &e)
        {
            // Error handling
            string text = "An exception occurred while starting camera grab.\n" + (string)e.GetDescription();
            HelperFunctions().callErrorDialog(text);
            this->close();
        }

        //start update timer to display images
        updateTimer->start();

        //start timer for ui counter change
        counterTimer->start();

    }
    else{
        if(this->imageSet.getNumberRecordedImages() < this->imageSet.getNumberOfImages()){
            //Save one image each
            saveImagesInImageSet(this->imageLeft, this->imageRight);
        }
        else{
            counterTimer->stop();
            updateTimer->stop();
            enableUI(true);
            cameras.StopGrabbing();
            ui->imageSetRecord_button->setText("Aufnahme starten");
        }
    }
}

void ImageSetDialog::initPylon(){
    // Get the transport layer factory.
    CTlFactory& tlFactory = CTlFactory::GetInstance();

    // Get all attached devices and exit application if no device is found.
    DeviceInfoList_t devices;

    if(tlFactory.EnumerateDevices(devices) < 2){
        throw RUNTIME_EXCEPTION("Less than two camera found");
    }

    //set converter parameters
    formatConverter.OutputPixelFormat = PixelType_RGB8packed;
    formatConverter.OutputBitAlignment = OutputBitAlignment_MsbAligned;

    cameras.Initialize(c_maxCamerasToUse);
    cout << "Number necessary cameras " << cameras.GetSize() << endl;
    cout << "Number devices found " << devices.size() << endl;

    // Create and attach all Pylon Devices.
    for ( size_t i = 0; i < cameras.GetSize(); ++i)
    {
        cout << "checking device at " << i << "." << endl;
        IPylonDevice *device = tlFactory.CreateDevice(devices[i]);
        cameras[i].Attach(device);
        // Print the model name of the camera.
        cout << "Using device "
             << cameras[i].GetDeviceInfo().GetModelName() << " | "
             << cameras[i].GetDeviceInfo().GetDeviceID() << endl;
    }

    cameras.Open();
}

void ImageSetDialog::saveInputInImageSet(){
    //save ui entries in new image set settings
    this->imageSet.setNumberOfImages(ui->numberImages_spinBox->value());
    this->imageSet.setRows(ui->rows_spinBox->value());
    this->imageSet.setColumns(ui->columns_spinBox->value());
    this->imageSet.setSquareSize(ui->squareSize_SpinBox->value());
    this->imageSet.setPatternType(getPatternType());
}

std::string ImageSetDialog::getPatternType(){
    if(ui->chessboard_radioButton->isChecked())
        return "chessboard";
    else if(ui->circle_radioButton->isChecked())
        return "circle";
    else
        return "circle asymmetrical";
}

void ImageSetDialog::enableUI(bool value){
    //save ui entries in new image set settings
    ui->showVideosCheckBox->setEnabled(value);
    ui->setImageSet_checkBox->setEnabled(value);
    ui->numberImages_spinBox->setEnabled(value);
    ui->rows_spinBox->setEnabled(value);
    ui->columns_spinBox->setEnabled(value);
    ui->squareSize_SpinBox->setEnabled(value);
    ui->chessboard_radioButton->setEnabled(value);
    ui->circle_radioButton->setEnabled(value);
    ui->asymCircles_radioButton->setEnabled(value);
}

void ImageSetDialog::update(){
    try{
        cameras[0].RetrieveResult(1, pylonResultLeft, TimeoutHandling_ThrowException);
        cameras[1].RetrieveResult(1, pylonResultRight, TimeoutHandling_ThrowException);

        if(cameras.IsGrabbing() && pylonResultLeft->GrabSucceeded() && pylonResultRight->GrabSucceeded()){
            //convert pylonimage to opencv mat
            formatConverter.Convert(pylonImageLeft, pylonResultLeft);
            formatConverter.Convert(pylonImageRight, pylonResultRight);
            cv::Mat imgLeft(pylonImageLeft.GetHeight(), pylonImageLeft.GetWidth(), CV_8UC3, (uint8_t*)pylonImageLeft.GetBuffer());
            cv::Mat imgRight(pylonImageRight.GetHeight(), pylonImageRight.GetWidth(), CV_8UC3, (uint8_t*)pylonImageRight.GetBuffer());

            imageLeft = imgLeft;
            imageRight = imgRight;

            //display images
            displayImages(imgLeft, imgRight);
        }
    }
    catch (const GenericException &e)
    {
        // Error handling
        cerr << "An exception occurred in update." << endl
        << e.GetDescription() << endl;
    }
}

void ImageSetDialog::displayImages(cv::Mat imageLeft, cv::Mat imageRight){
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

void ImageSetDialog::counter(){
    std::string currentAmount =   std::to_string(imageSet.getNumberRecordedImages());
    std::string goalAmount = std::to_string(imageSet.getNumberOfImages());
    std::string counter = currentAmount + "/" + goalAmount;

    ui->imageSetRecord_button->setText(QString::fromStdString(counter));
}

void ImageSetDialog::saveImagesInImageSet(cv::Mat leftImage, cv::Mat rightImage){

    //create save path
    std::string savePathLeft =  imageSet.getPath() +
                                "/Left/" +
                                std::to_string((imageSet.getNumberRecordedImages()+1)) +
                                "left" +
                                imageSet.getFileType();

    std::string savePathRight = imageSet.getPath() +
                                "/Right/" +
                                std::to_string((imageSet.getNumberRecordedImages()+1)) +
                                "right" +
                                imageSet.getFileType();

    //save left and right image
    cv::imwrite(savePathLeft, leftImage);
    cv::imwrite(savePathRight, rightImage);

    //increment number recorded images
    imageSet.incrementNumberRecordedImages();
}

ImageSet ImageSetDialog::createImageSet(){
    //Get Current Date as string
    std::string dirName = HelperFunctions().getCurrentDateAsString();

    //Build path for Image set
    std::string path = MainSettings().getImageSetsPath() + "/" + dirName;

    //Create stereo directory for left and right camera where images are stored
    HelperFunctions().createStereoDirectory(path);

    //Create Image set with path
    return ImageSet(path);
}
