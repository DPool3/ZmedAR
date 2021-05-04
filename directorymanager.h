#pragma once
#ifndef DIRECTORYMANAGER_H
#define DIRECTORYMANAGER_H

#include <opencv2/opencv.hpp>
#include <sys/stat.h>
#include <chrono>
#include <ctime>

#include "mainsettings.h"

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
