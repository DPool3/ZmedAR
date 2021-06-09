#ifndef VIDEOBASEDVIEW_H
#define VIDEOBASEDVIEW_H

#include <QDialog>

namespace Ui {
class VideoBasedView;
}

class VideoBasedView : public QDialog
{
    Q_OBJECT

public:
    explicit VideoBasedView(QWidget *parent = nullptr);
    ~VideoBasedView();

private:
    Ui::VideoBasedView *ui;
};

#endif // VIDEOBASEDVIEW_H
