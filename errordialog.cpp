#include "errordialog.h"
#include "ui_errordialog.h"

ErrorDialog::ErrorDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ErrorDialog)
{
    ui->setupUi(this);
}

ErrorDialog::~ErrorDialog()
{
    delete ui;
}

void ErrorDialog::setErrorText(const char* errorText){
    QString qstr = QString::fromUtf8(errorText);
    ui->ErrorLabel->setText(qstr);
}
