#ifndef IMAGESET_H
#define IMAGESET_H

#include <QDialog>
#include <QTimer>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <fstream>

#include "mainsettings.h"

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
    void setQ(cv::Mat);

    void setNewCamL(cv::Mat);
    void setNewCamR(cv::Mat);
    void setRectL(cv::Mat);
    void setRectR(cv::Mat);
    void setProjMatL(cv::Mat);
    void setProjMatR(cv::Mat);
    void setLeftStereoMap1(cv::Mat);
    void setRightStereoMap1(cv::Mat);
    void setLeftStereoMap2(cv::Mat);
    void setRightStereoMap2(cv::Mat);

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
    cv::Mat getQ();

    cv::Mat getNewCamL();
    cv::Mat getNewCamR();
    cv::Mat getRectL();
    cv::Mat getRectR();
    cv::Mat getProjMatL();
    cv::Mat getProjMatR();
    cv::Mat getLeftStereoMap1();
    cv::Mat getRightStereoMap1();
    cv::Mat getLeftStereoMap2();
    cv::Mat getRightStereoMap2();

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
    cv::Mat new_CamL, new_CamR, rect_l, rect_r, proj_mat_l, proj_mat_r, Q;
    cv::Mat Left_Stereo_Map1, Left_Stereo_Map2, Right_Stereo_Map1, Right_Stereo_Map2;
    std::string fileType;
    std::string patternType;

    void updateSettings();
    void setDefault();
};

#endif // IMAGESET_H
