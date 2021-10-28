#ifndef ERRORDIALOG_H
#define ERRORDIALOG_H

#include <QDialog>

namespace Ui {
class ErrorDialog;
}

/**
 * @brief Die ErrorDialog class erstellt ein Dialogfenster und überschreibt
 * den text des labels mit der angegebenen Fehlermeldung.
 */
class ErrorDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ErrorDialog(QWidget *parent = 0);
    ~ErrorDialog();
    void setErrorText(const char*);

private:
    Ui::ErrorDialog *ui;
};

#endif // ERRORDIALOG_H
