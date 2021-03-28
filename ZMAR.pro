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
        main.cpp \
        mainwindow.cpp \
    helperfunctions.cpp \
    mainsettings.cpp \
    errordialog.cpp \
    stereocalibration.cpp \
    videobased.cpp \
    camerabased.cpp \
    imageset.cpp \
    imagesetdialog.cpp \
    filesystem.cpp \
    PylonCamera.cpp

HEADERS += \
        mainwindow.h \
    helperfunctions.h \
    mainsettings.h \
    errordialog.h \
    stereocalibration.h \
    videobased.h \
    camerabased.h \
    imageset.h \
    imagesetdialog.h \
    filesystem.h \
    PylonCamera.h

FORMS += \
        mainwindow.ui \
    errordialog.ui \
    stereocalibration.ui \
    videobased.ui \
    camerabased.ui \
    imagesetdialog.ui \
    filesystem.ui

INCLUDEPATH += /usr/local/include/opencv2
INCLUDEPATH += /opt/pylon5/include

LIBS += $(shell pkg-config opencv --libs)
LIBS += -L/opt/pylon5/lib -lpylonbase -lpylonutility -lGenApi_gcc_v3_1_Basler_pylon -lGCBase_gcc_v3_1_Basler_pylon -lpylonc \
        -lNodeMapData_gcc_v3_1_Basler_pylon
LIBS += -L/opt/pylon5/lib64 -lavcodec -lavformat -lavresample -lavutil
