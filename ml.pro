TEMPLATE = app
CONFIG += c++11
CONFIG -= app_bundle
CONFIG -= console
CONFIG += qt

QT += core gui widgets

INCLUDEPATH += . \
                $(OPENCV_DIR)/include

CONFIG(debug, debug| release){
    END = 2413d
    DST = debug
}else{
    END = 2413
    DST = release
}

win32{
    INCLUDEPATH += $$PWD/fann/include
#    LIBS += -L$$PWD/fann/ -lfann
}else{
    END =
    LIBS += -lfann
}

LIBS += -L$(OPENCV_DIR)/x64/vc14/lib -lopencv_core$$END -lopencv_ml$$END -lopencv_highgui$$END -lopencv_imgproc$$END

UI_DIR = tmp/$$DST/ui
MOC_DIR = tmp/$$DST/moc
OBJECTS_DIR = tmp/$$DST/obj
RCC_DIR = tmp/$$DST/rcc

SOURCES += main.cpp \
    tracking.cpp \
    mainwindow.cpp \
    painter.cpp \
    worker.cpp

HEADERS += \
    tracking.h \
    mainwindow.h \
    painter.h \
    worker.h

FORMS += \
    mainwindow.ui \
    painter.ui
