#-------------------------------------------------
#
# Project created by QtCreator 2016-03-11T14:49:19
#
#-------------------------------------------------

QT       += core gui
QT       += widgets
QT       += printsupport
QT       += serialport
QT       += network
QT       += opengl
QT       += quick
QT       += quickcontrols2
QT	 += gamepad
QT       +=sql
QT       +=charts
QT       +=sensors

CONFIG   += c++11

DEFINES += HAS_JOYSTICK_CHECK
DEFINES += HAS_JOYSTICK

# Ubuntu
# sudo apt-get install libassimp-dev
#DEFINES += HAS_ASSIMP

# OpenGL support
!android: DEFINES += HAS_OPENGL
win32: LIBS += -lopengl32

#win32:QMAKE_LIBS_QT_ENGTRY -= -lqtmain
#win32-g++:DEFINES -= QT_NEEDS_QMAIN
CONFIG-= windows
QMAKE_LFLAGS += $$QMAKE_LFLAGS_WINDOWS

# Lime SDR support
#DEFINES += HAS_LIME_SDR

# Simulation Scennarios
#DEFINES += HAS_SIM_SCEN
# Usage: From the RControlStation root do:
# git clone https://github.com/esmini/esmini esmini
# and uncomment this define. The the editor will show up
# as the last tab in RControlStation.

TARGET = RControlStation
TEMPLATE = app

#CONFIG(release, debug|release):DEFINES += QT_NO_DEBUG_OUTPUT

release_win {
    DESTDIR = build/win
    OBJECTS_DIR = build/win/obj
    MOC_DIR = build/win/obj
    RCC_DIR = build/win/obj
    UI_DIR = build/win/obj
}

release_lin {
    # http://micro.nicholaswilson.me.uk/post/31855915892/rules-of-static-linking-libstdc-libc-libgcc
    # http://insanecoding.blogspot.se/2012/07/creating-portable-linux-binaries.html
    QMAKE_LFLAGS += -static-libstdc++ -static-libgcc
    DESTDIR = build/lin
    OBJECTS_DIR = build/lin/obj
    MOC_DIR = build/lin/obj
    RCC_DIR = build/lin/obj
    UI_DIR = build/lin/obj
}

release_android {
    DESTDIR = build/android
    OBJECTS_DIR = build/android/obj
    MOC_DIR = build/android/obj
    RCC_DIR = build/android/obj
    UI_DIR = build/android/obj
}

contains(DEFINES, HAS_ASSIMP) {
    LIBS += -lassimp
}

LIBS += -L/home/gunnar/code/openssl-1.1.1q -lcrypto -L/path/to/openssl-1.1.1q -lssl

SOURCES += main.cpp\
    CompassWidget.cpp \
    checkboxdelegate.cpp \
        mainwindow.cpp \
    qcustomplot.cpp \
    packetinterface.cpp \
    rtcm3_simple.cpp \
    sensorplot.cpp \
    utility.cpp \
    mapwidget.cpp \
    carinfo.cpp \
    locpoint.cpp \
    perspectivepixmap.cpp \
    carinterface.cpp \
    nmeaserver.cpp \
    rtcmclient.cpp \
    tcpbroadcast.cpp \
    basestation.cpp \
    ping.cpp \
    networklogger.cpp \
    osmclient.cpp \
    osmtile.cpp \
    tcpserversimple.cpp \
    packet.cpp \
    networkinterface.cpp \
    moteconfig.cpp \
    magcal.cpp \
    imuplot.cpp \
    nmeawidget.cpp \
    confcommonwidget.cpp \
    ublox.cpp \
    intersectiontest.cpp \
    ncom.cpp \
    correctionanalysis.cpp \
    historylineedit.cpp \
    imagewidget.cpp \
    tcpclientmulti.cpp \
    routemagic.cpp \
    task_basestation.cpp \
    wireguard.cpp

HEADERS  += mainwindow.h \
    CompassWidget.h \
    checkboxdelegate.h \
    core/coordinatetransforms.h \
    qcustomplot.h \
    datatypes.h \
    packetinterface.h \
    sensorplot.h \
    utility.h \
    mapwidget.h \
    carinfo.h \
    locpoint.h \
    perspectivepixmap.h \
    carinterface.h \
    nmeaserver.h \
    rtcm3_simple.h \
    rtcmclient.h \
    tcpbroadcast.h \
    basestation.h \
    ping.h \
    networklogger.h \
    osmclient.h \
    osmtile.h \
    tcpserversimple.h \
    packet.h \
    networkinterface.h \
    moteconfig.h \
    magcal.h \
    imuplot.h \
    nmeawidget.h \
    confcommonwidget.h \
    ublox.h \
    intersectiontest.h \
    ncom.h \
    correctionanalysis.h \
    historylineedit.h \
    imagewidget.h \
    tcpclientmulti.h \
    routemagic.h \
    attributes_masks.h \
    task.h \
    task_basestation.h \
    wireguard.h

FORMS    += mainwindow.ui \
    carinterface.ui \
    rtcmwidget.ui \
    basestation.ui \
    networklogger.ui \
    networkinterface.ui \
    moteconfig.ui \
    magcal.ui \
    imuplot.ui \
    nmeawidget.ui \
    confcommonwidget.ui \
    intersectiontest.ui \
    ncom.ui \
    correctionanalysis.ui \
    sensorplot.ui \
    wireguard.ui

contains(DEFINES, HAS_OPENGL) {
    SOURCES += orientationwidget.cpp
    HEADERS += orientationwidget.h
}

contains(DEFINES, HAS_LIME_SDR) {
    SOURCES += gpssim.cpp gpsgen.cpp limesdr.cpp
    HEADERS += gpssim.h gpsgen.h limesdr.h
    FORMS += gpssim.ui
    LIBS += -lLimeSuite
}

contains(DEFINES, HAS_SIM_SCEN) {
    INCLUDEPATH += esmini/EnvironmentSimulator/Libraries/esminiLib \
            esmini/EnvironmentSimulator/Modules/RoadManager \
            esmini/externals/pugixml \
            esmini/EnvironmentSimulator/Modules/CommonMini
    LIBS += -L"esmini/bin/" -lesminiLib
    SOURCES += pagesimscen.cpp
    HEADERS += pagesimscen.h \
            simscentree.h
    FORMS += pagesimscen.ui
}

RESOURCES += \
    resources.qrc

DISTFILES += \
    android/AndroidManifest.xml \
    android/gradle/wrapper/gradle-wrapper.jar \
    android/gradlew \
    android/res/values/libs.xml \
    android/build.gradle \
    android/gradle/wrapper/gradle-wrapper.properties \
    android/gradlew.bat

contains(ANDROID_TARGET_ARCH,armeabi-v7a) {
    ANDROID_PACKAGE_SOURCE_DIR = \
        $$PWD/android
}
