#ifndef CAMERABASEDUI_H
#define CAMERABASEDUI_H

#include <QDialog>
#include <QTimer>
#include <QElapsedTimer>

#include "camerabasedcontroller.h"
#include "videomanager.h"

namespace Ui {
class CameraBasedView;
}

/**
 * @brief Die CameraBasedView class leitet den Input in der GUI an den Controller und darzustellende
 * Bilder des Controllers an die GUI weiter.
 */
class CameraBasedView : public QDialog
{
    Q_OBJECT

public:
    explicit CameraBasedView(QWidget *parent = nullptr);
    ~CameraBasedView();

private slots:

    void on_startStopRecording_clicked();

    bool aquireProcessedImages();

    void displayImages();

    void on_showVideosCheckBox_toggled(bool checked);

    void on_viveTracking_Checkbox_toggled(bool checked);

    void on_cameraTracking_CheckBox_toggled(bool checked);

    void on_saveVideo_CheckBox_toggled(bool checked);

    void on_brightness_button_clicked();

    void on_exposure_button_clicked();

    void on_contrast_button_clicked();

    void on_saturation_button_clicked();

    void lockAndReleaseUI(bool);

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

    QElapsedTimer timer;
    int iterationCounter = 0;
};

#endif // CAMERABASEDUI_H
