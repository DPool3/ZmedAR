#ifndef FILESYSTEM_H
#define FILESYSTEM_H

#include <QDialog>
#include <QtCore>
#include <QtGui>
#include <QFileSystemModel>

namespace Ui {
class FileSystem;
}

class FileSystem : public QDialog
{
    Q_OBJECT

public:
    explicit FileSystem(QWidget *parent = 0);
    ~FileSystem();

    QString getSelectedFile();

private slots:
    void on_treeView_clicked(const QModelIndex &index);
    void on_listView_clicked(const QModelIndex &index);
    void on_pushButton_clicked();

private:
    Ui::FileSystem *ui;
    QFileSystemModel *dirmodel;
    QFileSystemModel *filemodel;
    QString selectedFile;
};

#endif // FILESYSTEM_H
