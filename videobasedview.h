#ifndef VIDEOBASEDVIEW_H
#define VIDEOBASEDVIEW_H

#include <QDialog>
#include "videobasedcontroller.h"
#include "videomanager.h"
#include "dialogmanager.h"

#include <QElapsedTimer>

namespace Ui {
class VideoBasedView;
}

/**
 * @brief Die VideoBasedView class leitet Input von der GUI an den Controller
 * weiter und darzustellende Bilder von dem Controller an de GUI.
 */
class VideoBasedView : public QDialog
{
    Q_OBJECT

public:
    explicit VideoBasedView(QWidget *parent = nullptr);
    ~VideoBasedView();

private slots:
    void on_startVideo_button_clicked();
    void on_showVideosCheckBox_toggled(bool checked);
    void on_searchFile_button_2_clicked();
    void on_searchFile_button_clicked();
    void on_cameraTracking_CheckBox_toggled(bool checked);
    void displayImages();

private:
    Ui::VideoBasedView *ui;
    VideoBasedController vbc;
    DialogManager dialogManager;

    QTimer *displayImagesTimer;

    bool displayVideos = false;

    QImage imageLeft;
    QImage imageRight;

    void aquireProcessedImages();
    void start();
    void stop();
    void lockOrReleaseUi(bool isEnabled);

    QElapsedTimer timer;
    int iterationCounter = 0;
};

#endif // VIDEOBASEDVIEW_H
