#-------------------------------------------------
#
# Project created by QtCreator 2021-02-19T20:43:29
#
#-------------------------------------------------

QT       += core gui

#CONFIG   += console

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = ZMAR
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
    camerabasedcontroller.cpp \
    camerabasedview.cpp \
    dialogmanager.cpp \
    directorymanager.cpp \
    imageprocessor.cpp \
        main.cpp \
        mainwindow.cpp \
    mainsettings.cpp \
    errordialog.cpp \
    stereocalibration.cpp \
    imageset.cpp \
    imagesetdialog.cpp \
    filesystem.cpp \
    PylonCamera.cpp \
    videobasedcontroller.cpp \
    videobasedview.cpp \
    videomanager.cpp \
    vivetracking.cpp

HEADERS += \
    camerabasedcontroller.h \
    camerabasedview.h \
    dialogmanager.h \
    directorymanager.h \
    imageprocessor.h \
        mainwindow.h \
    mainsettings.h \
    errordialog.h \
    stereocalibration.h \
    imageset.h \
    imagesetdialog.h \
    filesystem.h \
    PylonCamera.h \
    videobasedcontroller.h \
    videobasedview.h \
    videomanager.h \
    vivetracking.h

FORMS += \
    camerabasedview.ui \
        mainwindow.ui \
    errordialog.ui \
    stereocalibration.ui \
    videobased.ui \
    imagesetdialog.ui \
    filesystem.ui \
    videobasedview.ui

INCLUDEPATH += /usr/local/include/opencv4
#INCLUDEPATH += /usr/local/include/opencv2
INCLUDEPATH += /opt/pylon/include
INCLUDEPATH += /home/daniel/Software/GitProjects/openvr

LIBS += -L/home/daniel/Software/GitProjects/openvr/lib/linux64 -lopenvr_api
LIBS += $(shell pkg-config opencv4 --libs)
#LIBS += $(shell pkg-config opencv --libs)
LIBS += -L/opt/pylon/lib -lpylonbase -lpylonutility -lGenApi_gcc_v3_1_Basler_pylon -lGCBase_gcc_v3_1_Basler_pylon -lpylonc \
        -lNodeMapData_gcc_v3_1_Basler_pylon
LIBS += -L/opt/pylon/lib64 -lavcodec -lavformat -lavresample -lavutil
