#ifndef IMAGESETVIEW_H
#define IMAGESETVIEW_H

#include <QDialog>
#include <QTimer>

#include "videomanager.h"
#include "imagesetcontroller.h"

namespace Ui {
class ImageSetView;
}

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
