#ifndef IMAGESET_H
#define IMAGESET_H

#include <QDialog>
#include <QTimer>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>

class ImageSet
{
public:
    ImageSet(std::string);
    ImageSet(){}

    //Image set path
    void setPath(std::string);
    std::string getPath();

    //Image set number recorded Images
    void incrementNumberRecordedImages();
    int getNumberRecordedImages();

    //Image set settings
    //setter
    void setRows(int);
    void setColumns(int);
    void setNumberOfImages(int);
    void setSquareSize(double);
    void setReprojectionError(double);

    void setFileType(std::string);
    void setPatternType(std::string);

    void setCamL(cv::Mat);
    void setCamR(cv::Mat);
    void setDistCoefL(cv::Mat);
    void setDistCoefR(cv::Mat);
    void setR(cv::Mat);
    void setF(cv::Mat);
    void setE(cv::Mat);
    void setT(cv::Mat);

    //getter
    int getRows();
    int getColumns();
    int getNumberOfImages();
    double getSquareSize();
    double getReprojectionError();

    std::string getFileType();
    std::string getPatternType();

    cv::Mat getCamL();
    cv::Mat getCamR();
    cv::Mat getDistCoefL();
    cv::Mat getDistCoefR();
    cv::Mat getR();
    cv::Mat getF();
    cv::Mat getE();
    cv::Mat getT();

private:
    //Image set path
    std::string path;

    //Number recorded images
    int numberRecordedImages = 0;

    //Image set settings
    int rows, columns, numberOfImages;
    float squareSize;
    double reprojectionError;
    cv::Mat CamL, CamR, DistCoefL, DistCoefR, R, F, E, T;
    std::string fileType;
    std::string patternType;

    void updateSettings();
    void setDefault();
};

#endif // IMAGESET_H
