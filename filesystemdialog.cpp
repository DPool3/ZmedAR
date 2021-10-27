#include "filesystemdialog.h"
#include "ui_filesystemdialog.h"

/**
 * @brief FileSystemDialog::FileSystemDialog erstellt das Fenster des
 * Dateiexplorers und Initialisiert eine Tree der Ordner View auf der
 * Linken und eine List View der enthaltenen Dateien auf der Rechten
 * Seite des Fensters, mit "sPath" als root Ordner.
 * !WICHTIG! dieser muss gegebenenfalls an einene neuen Rechner
 * angepasst werden.
 * @param parent
 */
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

/**
 * @brief FileSystemDialog::~FileSystemDialog ist der Destruktor.
 */
FileSystemDialog::~FileSystemDialog()
{
    delete ui;
}

/**
 * @brief FileSystemDialog::on_treeView_clicked wählt den Ordner, auf den
 * geklickt wurde und setzt so den Ordner für die List View.
 * @param index
 */
void FileSystemDialog::on_treeView_clicked(const QModelIndex &index)
{
    QString sPath = dirmodel->fileInfo(index).absoluteFilePath();
    ui->listView->setRootIndex(filemodel->setRootPath(sPath));
}

/**
 * @brief FileSystemDialog::on_listView_clicked wählt die Datei, auf die
 * geklickt wurde und wählt so den Pfad, der gewählt wurde.
 * @param index
 */
void FileSystemDialog::on_listView_clicked(const QModelIndex &index)
{
    QString sPath = filemodel->fileInfo(index).absoluteFilePath();
    ui->lineEdit->setText(sPath);
    selectedFile = sPath;
}

/**
 * @brief FileSystemDialog::on_pushButton_clicked akzeptiert die Auswahl
 * und schließt das Fenster.
 */
void FileSystemDialog::on_pushButton_clicked()
{
    this->accept();
}

/**
 * @brief FileSystemDialog::getSelectedFile fragt dan dem Pfad
 * der ausgewählten Datei.
 * @return
 */
QString FileSystemDialog::getSelectedFile(){
    return this->selectedFile;
}
