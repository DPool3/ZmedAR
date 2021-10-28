#ifndef FILESYSTEM_H
#define FILESYSTEM_H

#include <QDialog>
#include <QtCore>
#include <QtGui>
#include <QFileSystemModel>

namespace Ui {
class FileSystemDialog;
}

/**
 * @brief Die FileSystemDialog class erstellt einen kleinen Dateiexplorer,
 * in dem links Ordnerstruktur undrechts die Dateistruktur innerhalb des gewählten Ordners
 * angezeigt wird. Durch auswählen und bestätigen einer Datei, wird der Pfad zurück gegeben.
 */
class FileSystemDialog : public QDialog
{
    Q_OBJECT

public:
    explicit FileSystemDialog(QWidget *parent = 0);
    ~FileSystemDialog();

    QString getSelectedFile();

private slots:
    void on_treeView_clicked(const QModelIndex &index);
    void on_listView_clicked(const QModelIndex &index);
    void on_pushButton_clicked();

private:
    Ui::FileSystemDialog *ui;
    QFileSystemModel *dirmodel;
    QFileSystemModel *filemodel;
    QString selectedFile;
};

#endif // FILESYSTEM_H
