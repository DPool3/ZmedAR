#include "filesystemdialog.h"
#include "ui_filesystemdialog.h"

FileSystemDialog::FileSystemDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::FileSystemDialog)
{
    ui->setupUi(this);

    //Init tree view
    QString sPath = "/home/daniel/ZAR";
    dirmodel = new QFileSystemModel(this);
    dirmodel->setFilter(QDir::NoDotAndDotDot | QDir::AllDirs);
    dirmodel->setRootPath(sPath);

    ui->treeView->setModel(dirmodel);
    QModelIndex index = dirmodel->index(sPath,0);
    ui->treeView->setRootIndex(index);

    //Init list view
    filemodel = new QFileSystemModel(this);
    filemodel->setFilter(QDir::NoDotAndDotDot | QDir::Files);
    filemodel->setRootPath(sPath);

    ui->listView->setModel(filemodel);
    index = filemodel->index(sPath, 0);
    ui->listView->setRootIndex(index);
}

FileSystemDialog::~FileSystemDialog()
{
    delete ui;
}

void FileSystemDialog::on_treeView_clicked(const QModelIndex &index)
{
    QString sPath = dirmodel->fileInfo(index).absoluteFilePath();
    ui->listView->setRootIndex(filemodel->setRootPath(sPath));
}

void FileSystemDialog::on_listView_clicked(const QModelIndex &index)
{
    QString sPath = filemodel->fileInfo(index).absoluteFilePath();
    ui->lineEdit->setText(sPath);
    selectedFile = sPath;
}

void FileSystemDialog::on_pushButton_clicked()
{
    this->accept();
}

QString FileSystemDialog::getSelectedFile(){
    return this->selectedFile;
}
