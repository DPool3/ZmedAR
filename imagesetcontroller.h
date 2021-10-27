#ifndef IMAGESETCONTROLLER_H
#define IMAGESETCONTROLLER_H

#include<iostream>
#include <QTimer>
#include <QObject>
#include<opencv4/opencv2/opencv.hpp>

#include"PylonCamera.h"
#include"dialogmanager.h"
#include"directorymanager.h"
#include"imageset.h"

/**
 * @brief In der ImageSetController class wird jede Aufnahme durch imagesetview gestartet.
 * Bei der Aufnahme speichert der imagesetcontroller, die zur gleichen Zeit,
 * von den Dart-Kameras erhaltenen Bilder mit angepasstem Bildnahmen,
 * in einem dafür Vorgesehenen Order des image sets.
 * Deweiteren erstellt dieser auch das image set und trägt alle Einträge in das Objekt ein.
 */
class ImageSetController : public QObject
{
    Q_OBJECT

public:
    ImageSetController();
    ~ImageSetController();

    void setNumberOfImages(int numberImgs);
    void setNumberRows(int numberRows);
    void setNumberColumns(int numberColumns);
    void setSquareSize(double squareSize);
    void setPatternType(std::string patternType);

    bool isRunning();

    bool startRecording();
    void stopRecording();

    void takeImage();

    bool getProcessedImages(QImage&, QImage&);

private slots:
    void getCameraImages();

private:

    ImageSet createImageSet();

    void saveInputInImageSet();

    int numberImages;
    int numberRows;
    int numberColumns;
    double squareSize;
    std::string patternType;

    QTimer* cameraImageTimer;

    ImageSet imageSet;

    pylonCamera cameras;

    bool running = false;

    cv::Mat imageLeft, imageRight;
    cv::Mat origLeft, origRight;

};

#endif // IMAGESETCONTROLLER_H
