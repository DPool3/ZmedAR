#include "imageset.h"

ImageSet::ImageSet(std::string path)
{
    //Save path for write
    this->path = path;
    this->fileType = MainSettings().getImageFileType();

    //Create file path for settings file
    std::string settingsPath = path + "/StereoCalibrationSettings.yml";

    //check if file already exists. If it exists read the settings else set default.
    std::ifstream in (settingsPath);
    bool fileIsEmpty = in.peek() == EOF;

    if (!fileIsEmpty) {
        //Save number of rows, columns, number of images and square_size
        cv::FileStorage fsRead(settingsPath, cv::FileStorage::READ);
        fsRead["pattern"] >> this->patternType;
        fsRead["rows"] >> this->rows;
        fsRead["columns"] >> this->columns;
        fsRead["square size"] >> this->squareSize;
        fsRead["numberOfImgs"] >> this->numberOfImages;
        fsRead["reprojectionError"] >> this->reprojectionError;
        fsRead["fileType"] >> this->fileType;
        fsRead["CamL"] >> this->CamL;
        fsRead["CamR"] >> this->CamR;
        fsRead["DistCoefL"] >> this->DistCoefL;
        fsRead["DistCoefR"] >> this->DistCoefR;
        fsRead["R"] >> this->R;
        fsRead["T"] >> this->T;
        fsRead["E"] >> this->E;
        fsRead["F"] >> this->F;
        fsRead["Q"] >> this->Q;
        fsRead["new_CamL"] >> this->new_CamL;
        fsRead["new_CamR"] >> this->new_CamR;
        fsRead["rect_l"] >> this->rect_l;
        fsRead["rect_r"] >> this->rect_r;
        fsRead["proj_mat_l"] >> this->proj_mat_l;
        fsRead["proj_mat_r"] >> this->proj_mat_r;
        fsRead["Left_Stereo_Map1"] >> this->Left_Stereo_Map1;
        fsRead["Left_Stereo_Map2"] >> this->Left_Stereo_Map2;
        fsRead["Right_Stereo_Map1"] >> this->Right_Stereo_Map1;
        fsRead["Right_Stereo_Map2"] >> this->Right_Stereo_Map2;
    }
    else{
        setDefault();
    }
}

//Image set path
void ImageSet::setPath(std::string newPath){
    path = newPath;
    updateSettings();
}

std::string ImageSet::getPath(){
    return path;
}

//Number recorded images
void ImageSet::incrementNumberRecordedImages(){
    numberRecordedImages++;
}

int ImageSet::getNumberRecordedImages(){
    return numberRecordedImages;
}

//Image set settings

//setter
void ImageSet::setNumberOfImages(int newValue){
    numberOfImages = newValue;
    updateSettings();
}

void ImageSet::setRows(int newValue){
    rows = newValue;
    updateSettings();
}

void ImageSet::setColumns(int newValue){
    columns = newValue;
    updateSettings();
}

void ImageSet::setSquareSize(double newValue){
    squareSize = newValue;
    updateSettings();
}

void ImageSet::setFileType(std::string newFileType){
    fileType = newFileType;
    updateSettings();
}

void ImageSet::setReprojectionError(double newValue){
    this->reprojectionError = newValue;
    updateSettings();
}

void ImageSet::setPatternType(std::string newPatternType){
    patternType = newPatternType;
    updateSettings();
}

void ImageSet::setCamL(cv::Mat newMat){
    CamL = newMat;
    updateSettings();
}

void ImageSet::setCamR(cv::Mat newMat){
    CamR = newMat;
    updateSettings();
}

void ImageSet::setDistCoefL(cv::Mat newMat){
    DistCoefL = newMat;
    updateSettings();
}

void ImageSet::setDistCoefR(cv::Mat newMat){
    DistCoefR = newMat;
    updateSettings();
}

void ImageSet::setR(cv::Mat newMat){
    R = newMat;
    updateSettings();
}

void ImageSet::setF(cv::Mat newMat){
    F = newMat;
    updateSettings();
}

void ImageSet::setE(cv::Mat newMat){
    E = newMat;
    updateSettings();
}

void ImageSet::setT(cv::Mat newMat){
    T = newMat;
    updateSettings();
}

void ImageSet::setQ(cv::Mat newMat){
    Q = newMat;
    updateSettings();
}

void ImageSet::setNewCamL(cv::Mat newMat){
    new_CamL = newMat;
    updateSettings();
}

void ImageSet::setNewCamR(cv::Mat newMat){
    new_CamR = newMat;
    updateSettings();
}

void ImageSet::setRectL(cv::Mat newMat){
    rect_l = newMat;
    updateSettings();
}

void ImageSet::setRectR(cv::Mat newMat){
    rect_r = newMat;
    updateSettings();
}

void ImageSet::setProjMatL(cv::Mat newMat){
    proj_mat_l = newMat;
    updateSettings();
}

void ImageSet::setProjMatR(cv::Mat newMat){
    proj_mat_r = newMat;
    updateSettings();
}

void ImageSet::setLeftStereoMap1(cv::Mat newMat){
    Left_Stereo_Map1 = newMat;
    updateSettings();
}

void ImageSet::setLeftStereoMap2(cv::Mat newMat){
    Left_Stereo_Map2 = newMat;
    updateSettings();
}

void ImageSet::setRightStereoMap1(cv::Mat newMat){
    Right_Stereo_Map1 = newMat;
    updateSettings();
}

void ImageSet::setRightStereoMap2(cv::Mat newMat){
    Right_Stereo_Map2 = newMat;
    updateSettings();
}

//getter
int ImageSet::getNumberOfImages(){
    return numberOfImages;
}

int ImageSet::getRows(){
    return rows;
}

int ImageSet::getColumns(){
    return columns;
}

double ImageSet::getSquareSize(){
    return squareSize;
}

std::string ImageSet::getFileType(){
    return fileType;
}

double ImageSet::getReprojectionError(){
    return reprojectionError;
}

std::string ImageSet::getPatternType(){
    return patternType;
}

cv::Mat ImageSet::getCamL(){
    return CamL;
}

cv::Mat ImageSet::getCamR(){
    return CamR;
}

cv::Mat ImageSet::getDistCoefL(){
    return DistCoefL;
}

cv::Mat ImageSet::getDistCoefR(){
    return DistCoefR;
}

cv::Mat ImageSet::getR(){
    return R;
}

cv::Mat ImageSet::getF(){
    return F;
}

cv::Mat ImageSet::getE(){
    return E;
}

cv::Mat ImageSet::getT(){
    return T;
}

cv::Mat ImageSet::getQ(){
    return Q;
}

cv::Mat ImageSet::getNewCamL(){
    return new_CamL;
}

cv::Mat ImageSet::getNewCamR(){
    return new_CamR;
}

cv::Mat ImageSet::getRectL(){
    return rect_l;
}

cv::Mat ImageSet::getRectR(){
    return rect_r;
}

cv::Mat ImageSet::getProjMatL(){
    return proj_mat_l;
}

cv::Mat ImageSet::getProjMatR(){
    return proj_mat_r;
}

cv::Mat ImageSet::getLeftStereoMap1(){
    return Left_Stereo_Map1;
}

cv::Mat ImageSet::getLeftStereoMap2(){
    return Left_Stereo_Map2;
}

cv::Mat ImageSet::getRightStereoMap1(){
    return Right_Stereo_Map1;
}

cv::Mat ImageSet::getRightStereoMap2(){
    return Right_Stereo_Map2;
}

void ImageSet::updateSettings(){
    //

    //Create file path for settings file
    std::string settingsPath = path + "/StereoCalibrationSettings.yml";

    cv::FileStorage fsWrite(settingsPath, cv::FileStorage::WRITE);
    fsWrite << "pattern" << this->patternType;
    fsWrite << "rows" << this->rows;
    fsWrite << "columns" << this->columns;
    fsWrite << "square size" << this->squareSize;
    fsWrite << "numberOfImgs" << this->numberOfImages;
    fsWrite << "fileType" << this->fileType;
    fsWrite << "reprojectionError" << this->reprojectionError;
    fsWrite << "CamL" << this->CamL;
    fsWrite << "CamR" << this->CamR;
    fsWrite << "DistCoefL" << this->DistCoefL;
    fsWrite << "DistCoefR" << this->DistCoefR;
    fsWrite << "R" << this->R;
    fsWrite << "T" << this->T;
    fsWrite << "E" << this->E;
    fsWrite << "F" << this->F;
    fsWrite << "Q" << this->Q;
    fsWrite << "new_CamL" << this->new_CamL;
    fsWrite << "new_CamR" << this->new_CamR;
    fsWrite << "rect_l" << this->rect_l;
    fsWrite << "rect_r" << this->rect_r;
    fsWrite << "proj_mat_l" << this->proj_mat_l;
    fsWrite << "proj_mat_r" << this->proj_mat_r;
    fsWrite << "Left_Stereo_Map1" << this->Left_Stereo_Map1;
    fsWrite << "Left_Stereo_Map2" << this->Left_Stereo_Map2;
    fsWrite << "Right_Stereo_Map1" << this->Right_Stereo_Map1;
    fsWrite << "Right_Stereo_Map2" << this->Right_Stereo_Map2;
}

void ImageSet::setDefault(){
    //set integer and float to default 0
    this->numberOfImages = 0;
    this->rows = 0;
    this->columns = 0;
    this->squareSize = 0.0;

    //Create file path for settings file
    std::string settingsPath = path + "/StereoCalibrationSettings.yml";

    cv::FileStorage fsWrite(settingsPath, cv::FileStorage::WRITE);
    fsWrite << "pattern" << this->patternType;
    fsWrite << "rows" << this->rows;
    fsWrite << "columns" << this->columns;
    fsWrite << "square size" << this->squareSize;
    fsWrite << "numberOfImgs" << this->numberOfImages;
    fsWrite << "fileType" << this->fileType;
    fsWrite << "CamL" << this->CamL;
    fsWrite << "CamR" << this->CamR;
    fsWrite << "DistCoefL" << this->DistCoefL;
    fsWrite << "DistCoefR" << this->DistCoefR;
    fsWrite << "R" << this->R;
    fsWrite << "T" << this->T;
    fsWrite << "E" << this->E;
    fsWrite << "F" << this->F;
    fsWrite << "Q" << this->Q;
    fsWrite << "new_CamL" << this->new_CamL;
    fsWrite << "new_CamR" << this->new_CamR;
    fsWrite << "rect_l" << this->rect_l;
    fsWrite << "rect_r" << this->rect_r;
    fsWrite << "proj_mat_l" << this->proj_mat_l;
    fsWrite << "proj_mat_r" << this->proj_mat_r;
    fsWrite << "Left_Stereo_Map1" << this->Left_Stereo_Map1;
    fsWrite << "Left_Stereo_Map2" << this->Left_Stereo_Map2;
    fsWrite << "Right_Stereo_Map1" << this->Right_Stereo_Map1;
    fsWrite << "Right_Stereo_Map2" << this->Right_Stereo_Map2;
}
