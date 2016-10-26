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
    qimagemat.cpp \
    cstitch.cpp

HEADERS  += dialog.h \
    cstereomatch.h \
    cstereocalib.h \
    cparamcalib.h \
    qimagemat.h \
    cstitch.h

FORMS    += dialog.ui

I#NCLUDEPATH +=/usr/local/include/ \
#/usr/local/include/opencv \
#/usr/local/include/opencv2 \

#LIBS+=/usr/local/lib/libopencv_*.so \

INCLUDEPATH +=E:\opencv\build\include\opencv\
             E:\opencv\build\include\opencv2\
             E:\opencv\build\include\
            C:\Qt\tbb44_20151115oss\include

LIBS+=E:\opencv\build\mingw\lib\libopencv_*.dll.a
