#include "filesystem.h"
#include "ui_filesystem.h"

FileSystem::FileSystem(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::FileSystem)
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

FileSystem::~FileSystem()
{
    delete ui;
}

void FileSystem::on_treeView_clicked(const QModelIndex &index)
{
    QString sPath = dirmodel->fileInfo(index).absoluteFilePath();
    ui->listView->setRootIndex(filemodel->setRootPath(sPath));
}

void FileSystem::on_listView_clicked(const QModelIndex &index)
{
    QString sPath = filemodel->fileInfo(index).absoluteFilePath();
    ui->lineEdit->setText(sPath);
    selectedFile = sPath;
}

void FileSystem::on_pushButton_clicked()
{
    this->accept();
}

QString FileSystem::getSelectedFile(){
    return this->selectedFile;
}
