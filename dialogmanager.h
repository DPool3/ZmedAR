#ifndef DIALOGMANAGER_H
#define DIALOGMANAGER_H

#include "errordialog.h"
#include "filesystemdialog.h"

class DialogManager
{
public:
    DialogManager();

    void callErrorDialog(std::string);
    void callErrorDialog(const char*);
    QString getPathFromFileSystem();

};

#endif // DIALOGMANAGER_H
