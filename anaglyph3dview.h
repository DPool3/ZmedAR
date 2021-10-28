#ifndef ANAGLYPH3DVIEW_H
#define ANAGLYPH3DVIEW_H

#include <QDialog>
#include <QTimer>

#include "anaglyph3dcontroller.h"

namespace Ui {

class Anaglyph3DView;
}

/**
 * @brief Die Anaglyph3DView class leitet Input in der GUI an den Controller und
 * darzustellende Bilder des Controllers an die GUI weiter.
 */
class Anaglyph3DView : public QDialog
{
    Q_OBJECT

public:
    explicit Anaglyph3DView(QWidget *parent = nullptr);
    ~Anaglyph3DView();

private slots:
    void on_startVideo_button_clicked();

    void on_searchRigthVideoFile_button_clicked();

    void on_searchLeftVideoFile_button_clicked();

    void displayAnaglyph3D();

private:
    //private methods
    void lockReleaseUi(bool checked);

    void setLineEditText(QString text, std::string leftOrRight);

    void startAnaglyph3D();

    void stopAnaglyph3D();

    //private variables
    Ui::Anaglyph3DView *ui;

    Anaglyph3DController anaglyph3DController;

    QTimer* displayImageTimer;

    QImage anaglyphImage;

};

#endif // ANAGLYPH3DVIEW_H
