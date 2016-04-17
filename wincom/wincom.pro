#-------------------------------------------------
#
# Project created by QtCreator 2010-07-14T17:54:25
#
#-------------------------------------------------

QT       += core gui

TARGET = wincom
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    qextserialport.cpp \
    qextserialbase.cpp \
    aboutdialog.cpp \
    win_qextserialport.cpp

HEADERS  += mainwindow.h \
    qextserialbase.h \
    aboutdialog.h \
    win_qextserialport.h \
    qextserialport.h

FORMS    += mainwindow.ui \
    aboutdialog.ui

RESOURCES += \
    src.qrc
RC_FILE += myico.rc

OTHER_FILES += \
    myico.rc
