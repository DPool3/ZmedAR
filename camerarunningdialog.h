#ifndef CAMERARUNNINGDIALOG_H
#define CAMERARUNNINGDIALOG_H

#include <QDialog>

namespace Ui {
class CameraRunningDialog;
}

class CameraRunningDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CameraRunningDialog(QWidget *parent = 0);
    ~CameraRunningDialog();

private:
    Ui::CameraRunningDialog *ui;
};

#endif // CAMERARUNNINGDIALOG_H
