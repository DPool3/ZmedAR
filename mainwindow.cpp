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
    StereoCalibrationView scv;
    scv.setModal(true);
    scv.exec();
}

void MainWindow::on_imageSet_button_clicked()
{
    ImageSetView isv;
    isv.setModal(true);
    isv.exec();
}

void MainWindow::on_videoBased_button_clicked()
{
    VideoBasedView vbd;
    vbd.setModal(true);
    vbd.exec();
}

void MainWindow::on_cameraBased_button_clicked()
{
    CameraBasedView cbv;
    cbv.setModal(true);
    cbv.exec();
}

void MainWindow::on_anaglyph3DButton_clicked()
{
    Anaglyph3DView a3dv;
    a3dv.setModal(true);
    a3dv.exec();
}
