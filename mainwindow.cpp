#include "mainwindow.h"
#include "ui_mainwindow.h"

/**
 * @brief MainWindow::MainWindow erstellt die UI
 * @param parent
 */
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

/**
 * @brief MainWindow::~MainWindow ist der Destruktor.
 */
MainWindow::~MainWindow()
{
    delete ui;
}

//-----------------------------------------------------------------------------------------
//Dialog call
//-----------------------------------------------------------------------------------------
/**
 * @brief MainWindow::on_stereoCalibration_button_clicked ruft das Fenster für die
 * Stereokamerakalibrierung auf.
 */
void MainWindow::on_stereoCalibration_button_clicked()
{
    StereoCalibrationView scv;
    scv.setModal(true);
    scv.exec();
}

/**
 * @brief MainWindow::on_imageSet_button_clicked ruft das Fenster für die
 * Erstellung des ImageSets auf.
 */
void MainWindow::on_imageSet_button_clicked()
{
    ImageSetView isv;
    isv.setModal(true);
    isv.exec();
}

/**
 * @brief MainWindow::on_videoBased_button_clicked ruft das Fenster für die
 * videobasierte Verarbeitung auf.
 */
void MainWindow::on_videoBased_button_clicked()
{
    VideoBasedView vbd;
    vbd.setModal(true);
    vbd.exec();
}

/**
 * @brief MainWindow::on_cameraBased_button_clicked ruft das Fenster für die
 * kamerabasierte Verarbeitung auf.
 */
void MainWindow::on_cameraBased_button_clicked()
{
    CameraBasedView cbv;
    cbv.setModal(true);
    cbv.exec();
}

/**
 * @brief MainWindow::on_anaglyph3DButton_clicked ruft das Fenster für die
 * Darstellung eines Anaglyph 3D Videos auf.
 */
void MainWindow::on_anaglyph3DButton_clicked()
{
    Anaglyph3DView a3dv;
    a3dv.setModal(true);
    a3dv.exec();
}
