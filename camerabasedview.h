#ifndef CAMERABASEDUI_H
#define CAMERABASEDUI_H

#include <QDialog>
#include <QTimer>

#include "camerabasedcontroller.h"
#include "videomanager.h"

namespace Ui {
class CameraBasedView;
}

class CameraBasedView : public QDialog
{
    Q_OBJECT

public:
    explicit CameraBasedView(QWidget *parent = nullptr);
    ~CameraBasedView();

private slots:
    void on_showVideosCheckBox_toggled(bool checked);

    void on_saveVideoButton_clicked();

    void on_startStopRecording_clicked();

    void aquireProcessedImages();

    void displayImages();

    void on_doubleSpinBox_valueChanged(double arg1);

    void on_doubleSpinBox_2_valueChanged(double arg1);

    void on_doubleSpinBox_3_valueChanged(double arg1);

private:
    Ui::CameraBasedView *ui;

    CameraBasedController cbc;
    VideoManager videoManager;

    QTimer *displayImagesTimer;

    bool display = false;

    bool recording = false;
    bool running = false;

    QImage imageLeft;
    QImage imageRight;
};

#endif // CAMERABASEDUI_H
