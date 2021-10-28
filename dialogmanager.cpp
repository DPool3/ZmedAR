#include "dialogmanager.h"

/**
 * @brief DialogManager::DialogManager ist der Konstruktor.
 */
DialogManager::DialogManager()
{

}

/**
 * @brief DialogManager::callErrorDialog ruft ein Errordialog auf,
 * indem der string verwendet wird.
 * @param errorMsg
 */
void DialogManager::callErrorDialog(std::string errorMsg){
    ErrorDialog errd;
    errd.setModal(true);
    errd.setErrorText(errorMsg.c_str());
    errd.exec();
}

/**
 * @brief DialogManager::callErrorDialog ruft ein Errordialog auf,
 * indem der const char* verwendet wird.
 * @param errorMsg
 */
void DialogManager::callErrorDialog(const char* errorMsg){
    ErrorDialog errd;
    errd.setModal(true);
    errd.setErrorText(errorMsg);
    errd.exec();
}

/**
 * @brief DialogManager::getPathFromFileSystem ruft den Dateiexplorer auf.
 * @return Pfad der im Explorer ausgewählt wurde.
 */
QString DialogManager::getPathFromFileSystem(){
    FileSystemDialog fsd;
    fsd.setModal(true);
    fsd.exec();
    return fsd.getSelectedFile();
}
