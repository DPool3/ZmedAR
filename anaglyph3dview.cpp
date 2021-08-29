#include "anaglyph3dview.h"
#include "ui_anaglyph3dview.h"

Anaglyph3DView::Anaglyph3DView(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Anaglyph3DView)
{
    ui->setupUi(this);

    displayImageTimer = new QTimer(this);
    connect(displayImageTimer, SIGNAL(timeout()), this, SLOT(displayAnaglyph3D()));
}

Anaglyph3DView::~Anaglyph3DView()
{
    delete ui;
}

void Anaglyph3DView::on_startVideo_button_clicked()
{
    if(!displayImageTimer->isActive())
        startAnaglyph3D();
    else
        stopAnaglyph3D();
    lockReleaseUi(!ui->startVideo_button->isEnabled());
}

void Anaglyph3DView::on_searchRigthVideoFile_button_clicked()
{
    QString leftVideoPath = anaglyph3DController.searchVideoPath();
    setLineEditText(leftVideoPath, "right");
    anaglyph3DController.leftVideoPath = leftVideoPath.toStdString();
}

void Anaglyph3DView::on_searchLeftVideoFile_button_clicked()
{
    QString rightVideoPath = anaglyph3DController.searchVideoPath();
    setLineEditText(rightVideoPath, "left");
    anaglyph3DController.rightVideoPath = rightVideoPath.toStdString();
}

void Anaglyph3DView::startAnaglyph3D()
{
    anaglyph3DController.reinitAnaglyph3DController();
    displayImageTimer->setInterval(1000/anaglyph3DController.getPlayFps());
    displayImageTimer->start();
}

void Anaglyph3DView::stopAnaglyph3D()
{
    displayImageTimer->stop();
}

void Anaglyph3DView::lockReleaseUi(bool checked)
{
    ui->videoFilePathLeft_QLineEdit->setEnabled(checked);
    ui->videoFilePathRight_QLineEdit->setEnabled(checked);
    ui->startVideo_button->setEnabled(checked);
    ui->searchLeftVideoFile_button->setEnabled(checked);
    ui->searchRigthVideoFile_button->setEnabled(checked);
}

void Anaglyph3DView::setLineEditText(QString text, std::string leftOrRight)
{
    if(leftOrRight == "left")
        ui->videoFilePathLeft_QLineEdit->setText(text);
    else
        ui->videoFilePathRight_QLineEdit->setText(text);
}

void Anaglyph3DView::displayAnaglyph3D()
{
    if(!anaglyph3DController.getAnaglyphImage(anaglyphImage))
       stopAnaglyph3D();

    ui->videoLabel->setPixmap(QPixmap::fromImage(anaglyphImage));
    ui->videoLabel->resize(ui->videoLabel->pixmap()->size());
}

