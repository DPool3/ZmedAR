#ifndef IMAGESETVIEW_H
#define IMAGESETVIEW_H

#include <QDialog>
#include <QTimer>

#include "videomanager.h"
#include "imagesetcontroller.h"

namespace Ui {
class ImageSetView;
}

/**
 * @brief Die ImageSetView class leitet Input von der GUI an den Controller weiter
 * und darzustellende Bilder von dem Controller an die GUI.
 * Desweiteren wird hier mit jedem Knopfdruck die Aufnahme des Controllers gestartet,
 * bis die gewünschte Anzahl an Aufnahmen durchgeführt wurde.
 */
class ImageSetView : public QDialog
{
    Q_OBJECT

public:
    explicit ImageSetView(QWidget *parent = nullptr);
    ~ImageSetView();

private slots:
    void on_showVideosCheckBox_toggled(bool checked);

    void on_setImageSet_checkBox_toggled(bool checked);

    void on_imageSetRecord_button_clicked();

    void displayImages();

private:
    void lockReleaseUi(bool checked);

    bool aquireProcessedImages();

    std::string getPatternType();

    void startButtonCounter(int maxNumberImages);

    ImageSetController imageSetController;
    VideoManager videoManager;
    QTimer *displayImagesTimer;
    Ui::ImageSetView *ui;

    QImage imageLeft, imageRight;

    int iteration = 0;

    bool showVideo = false;
};

#endif // IMAGESETVIEW_H
