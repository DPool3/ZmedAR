#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

#include "videobasedview.h"
#include "camerabasedview.h"
#include "anaglyph3dview.h"
#include "imagesetview.h"
#include "stereocalibrationview.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

    void on_stereoCalibration_button_clicked();

    void on_imageSet_button_clicked();

    void on_videoBased_button_clicked();

    void on_cameraBased_button_clicked();

    void on_anaglyph3DButton_clicked();

public slots:

private:
    Ui::MainWindow *ui;

};

#endif // MAINWINDOW_H
