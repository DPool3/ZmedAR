#ifndef DIALOGMANAGER_H
#define DIALOGMANAGER_H

#include "errordialog.h"
#include "filesystemdialog.h"

/**
 * @brief Die DialogManager class ruft ein Dialog-Fenster für Fehlermeldungen auf oder
 * ein Dateisystem. Kann mit const char* oder string eine Fehlermeldung im Dialog zeigen.
 * Es verwendet für diesen Aufruf den errordialog mit seiner errordialog.ui.
 * Für den Aufruf des Dateisystems wird filesystemdialog mit seine filesystemdialog.ui verwendet.
 */
class DialogManager
{
public:
    DialogManager();

    void callErrorDialog(std::string);
    void callErrorDialog(const char*);
    QString getPathFromFileSystem();

};

#endif // DIALOGMANAGER_H
