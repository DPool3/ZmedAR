#pragma once
#ifndef DIRECTORYMANAGER_H
#define DIRECTORYMANAGER_H

#include <opencv4/opencv2/opencv.hpp>
#include <sys/stat.h>
#include <chrono>
#include <ctime>

#include "mainsettings.h"

/**
 * @brief Die DirectoryManager class prüft Existenz von Ordnern die erstellt werden sollen
 * oder existieren sollten. Kann allgemein Ordner in einem bestimmten Pfad erstellen,
 * aber auch speziell ImageSet-Ordner und Video-Ordner mit ihrer jeweiligen Struktur.
 * Verwendet Datum und Uhrzeit für Ordnernamen.
 */
class DirectoryManager
{
public:
    DirectoryManager();

    void initDirectoryHierarchy();

    std::string createVideoDirectory(std::string);
    std::string createImageSetDirectory();

    void createDirectory(std::string path);
    void createStereoDirectory(std::string path);

    bool pathOrFileExists(std::string path);
    std::string getCurrentDateAsString();
};

#endif // DIRECTORYMANAGER_H
