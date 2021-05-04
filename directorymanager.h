#pragma once
#ifndef DIRECTORYMANAGER_H
#define DIRECTORYMANAGER_H

#include <opencv2/opencv.hpp>
#include <sys/stat.h>

#include "mainsettings.h"

class DirectoryManager
{
public:
    DirectoryManager();

    void initDirectoryHierarchy();

    void createDirectory(std::string path);
    void createStereoDirectory(std::string path);

    bool pathOrFileExists(std::string path);
};

#endif // DIRECTORYMANAGER_H
