#-------------------------------------------------
#
# Project created by QtCreator 2015-11-02T09:41:43
#
#-------------------------------------------------

QT          += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
CONFIG      +=c++11
TARGET      = StereoFace
TEMPLATE    = app

SOURCES += main.cpp\
        dialog.cpp \
    cstereomatch.cpp \
    cstereocalib.cpp \
    cparamcalib.cpp \
    qimagemat.cpp

HEADERS  += dialog.h \
    cstereomatch.h \
    cstereocalib.h \
    cparamcalib.h \
    qimagemat.h

FORMS    += dialog.ui

INCLUDEPATH +=/usr/include/opencv \
             /usr/include/opencv2 \
             /usr/include \

LIBS+=/usr/lib/x86_64-linux-gnu/libopencv_*.so \
#LIBS+=/home/wnk/opencv/install/lib/libopencv_*.so \


