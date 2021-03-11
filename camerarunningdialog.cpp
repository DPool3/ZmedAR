#include "camerarunningdialog.h"
#include "ui_camerarunningdialog.h"

CameraRunningDialog::CameraRunningDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CameraRunningDialog)
{
    ui->setupUi(this);
}

CameraRunningDialog::~CameraRunningDialog()
{
    delete ui;
}
