#include "anaglyph3dview.h"
#include "ui_anaglyph3dview.h"

/**
 * @brief Anaglyph3DView::Anaglyph3DView initialisiert die Benuzteroberfläche
 * und erstellt einen displayImageTimer, der für die Ausführung der Erstellung
 * des Anaglyph 3D Bildes notwendig ist.
 * @param parent
 */
Anaglyph3DView::Anaglyph3DView(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Anaglyph3DView)
{
    ui->setupUi(this);

    displayImageTimer = new QTimer(this);
    connect(displayImageTimer, SIGNAL(timeout()), this, SLOT(displayAnaglyph3D()));
}

/**
 * @brief Anaglyph3DView::~Anaglyph3DView ist der Destruktor für diese Klasse.
 */
Anaglyph3DView::~Anaglyph3DView()
{
    delete ui;
}

/**
 * @brief Anaglyph3DView::on_startVideo_button_clicked startet und stoppt den Prozess.
 * Gleichzeitig wird die UI während des Prozesses gesperrt oder freigegeben.
 */
void Anaglyph3DView::on_startVideo_button_clicked()
{
    if(!displayImageTimer->isActive())
        startAnaglyph3D();
    else
        stopAnaglyph3D();
    lockReleaseUi(!ui->startVideo_button->isEnabled());
}

/**
 * @brief Anaglyph3DView::on_searchRigthVideoFile_button_clicked ermöglicht die Suche eines
 * Videos für das Rechte Video.
 */
void Anaglyph3DView::on_searchRigthVideoFile_button_clicked()
{
    QString leftVideoPath = anaglyph3DController.searchVideoPath();
    setLineEditText(leftVideoPath, "right");
    anaglyph3DController.leftVideoPath = leftVideoPath.toStdString();
}

/**
 * @brief Anaglyph3DView::on_searchLeftVideoFile_button_clicked ermöglicht die Suche eines
 * Videos für das Linke Video.
 */
void Anaglyph3DView::on_searchLeftVideoFile_button_clicked()
{
    QString rightVideoPath = anaglyph3DController.searchVideoPath();
    setLineEditText(rightVideoPath, "left");
    anaglyph3DController.rightVideoPath = rightVideoPath.toStdString();
}

/**
 * @brief Anaglyph3DView::startAnaglyph3D startet die Darstellung der Anaglyph
 * 3D Videos, indem es dem Prozess initialisiert, Die Time Frequenz an die Bildrate
 * des Videos anpasst und den Timer startet.
 */
void Anaglyph3DView::startAnaglyph3D()
{
    anaglyph3DController.reinitAnaglyph3DController();
    displayImageTimer->setInterval(1000/anaglyph3DController.getPlayFps());
    displayImageTimer->start();
}

/**
 * @brief Anaglyph3DView::stopAnaglyph3D stoppt die Darstellung der Anaglyph
 * 3D Videos, indem es den Timer stoppt, der für die Erstellung der Bilder
 * zuständig ist.
 */
void Anaglyph3DView::stopAnaglyph3D()
{
    displayImageTimer->stop();
}

/**
 * @brief Anaglyph3DView::lockReleaseUi sperrt die UI oder gibt sie frei,
 * abhängig vom Ausgangszustand. Ist es aktuell gelockt, würde eine Ausführung
 * die Freigabe der Benutzeroberfläche zur Folge haben und umgekehrt.
 * @param checked
 */
void Anaglyph3DView::lockReleaseUi(bool checked)
{
    ui->videoFilePathLeft_QLineEdit->setEnabled(checked);
    ui->videoFilePathRight_QLineEdit->setEnabled(checked);
    ui->startVideo_button->setEnabled(checked);
    ui->searchLeftVideoFile_button->setEnabled(checked);
    ui->searchRigthVideoFile_button->setEnabled(checked);
}

/**
 * @brief Anaglyph3DView::setLineEditText setzt den Text in den Textbereichen
 * der Suchtfelder.
 * @param text des gefundenen Pfades.
 * @param leftOrRight um zu bestimmen, welcher Textbereich gesetzt werden soll.
 */
void Anaglyph3DView::setLineEditText(QString text, std::string leftOrRight)
{
    if(leftOrRight == "left")
        ui->videoFilePathLeft_QLineEdit->setText(text);
    else
        ui->videoFilePathRight_QLineEdit->setText(text);
}

/**
 * @brief Anaglyph3DView::displayAnaglyph3D führt die getAnaglyphImage Methode
 * des anaglyph3DControllers aus, um neue Anaglyph 3D Bilder der Videos zu erhalten.
 * Stoppt falls es einen Fehler bei der Generierung gab. Zum Schluss wird das Bild
 * in der Benutzeroberfläche dargestell.
 */
void Anaglyph3DView::displayAnaglyph3D()
{
    if(!anaglyph3DController.getAnaglyphImage(anaglyphImage))
       stopAnaglyph3D();

    ui->videoLabel->setPixmap(QPixmap::fromImage(anaglyphImage));
    ui->videoLabel->resize(ui->videoLabel->pixmap()->size());
}

