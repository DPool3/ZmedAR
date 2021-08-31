#ifndef STEREOCALIBRATIONVIEW_H
#define STEREOCALIBRATIONVIEW_H

#include <QDialog>
#include <QTimer>

#include <stereocalibrationcontroller.h>

namespace Ui {
class StereoCalibrationView;
}

class StereoCalibrationView : public QDialog
{
    Q_OBJECT

public:
    explicit StereoCalibrationView(QWidget *parent = nullptr);
    ~StereoCalibrationView();

private slots:
    void on_searchFile_button_clicked();

    void on_startCalibration_button_clicked();

    void on_displayImages_checkbox_toggled(bool checked);

    void displayImages();

private:
    void lockReleaseUi(bool checked);

    void startCameraCalibration();

    void stopCameraCalibration();

    void loadCalibrationInfo();

    void displayCalibrationImages();

    StereoCalibrationController stereoCalibrationController;

    QTimer* displayImageTimer;

    Ui::StereoCalibrationView *ui;

    bool displayImagesBool;
};

#endif // STEREOCALIBRATIONVIEW_H
