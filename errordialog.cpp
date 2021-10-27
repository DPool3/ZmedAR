#include "errordialog.h"
#include "ui_errordialog.h"

/**
 * @brief ErrorDialog::ErrorDialog ist der Konstruktor des Dialogs.
 * @param parent.
 */
ErrorDialog::ErrorDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ErrorDialog)
{
    ui->setupUi(this);
}

/**
 * @brief ErrorDialog::~ErrorDialog ist der Destruktor des Dialogs.
 */
ErrorDialog::~ErrorDialog()
{
    delete ui;
}

/**
 * @brief ErrorDialog::setErrorText setzt den Text im Dialogfenster.
 * @param errorText.
 */
void ErrorDialog::setErrorText(const char* errorText){
    QString qstr = QString::fromUtf8(errorText);
    ui->ErrorLabel->setText(qstr);
}
