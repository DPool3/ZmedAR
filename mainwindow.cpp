#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}

//-----------------------------------------------------------------------------------------
//Dialog call
//-----------------------------------------------------------------------------------------

void MainWindow::on_stereoCalibration_button_clicked()
{
    StereoCalibration scd;
    scd.setModal(true);
    scd.exec();
}

void MainWindow::on_imageSet_button_clicked()
{
    ImageSetDialog isd;
    isd.setModal(true);
    isd.exec();
}

void MainWindow::on_videoBased_button_clicked()
{
    VideoBasedView vbd;
    vbd.setModal(true);
    vbd.exec();
}

void MainWindow::on_cameraBased_button_clicked()
{
    CameraBasedView cbd;
    cbd.setModal(true);
    cbd.exec();
}
