#include "dialogmanager.h"

DialogManager::DialogManager()
{

}

void DialogManager::callErrorDialog(std::string errorMsg){
    ErrorDialog errd;
    errd.setModal(true);
    errd.setErrorText(errorMsg.c_str());
    errd.exec();
}

void DialogManager::callErrorDialog(const char* errorMsg){
    ErrorDialog errd;
    errd.setModal(true);
    errd.setErrorText(errorMsg);
    errd.exec();
}

QString DialogManager::getPathFromFileSystem(){
    FileSystem fsd;
    fsd.setModal(true);
    fsd.exec();
    return fsd.getSelectedFile();
}
