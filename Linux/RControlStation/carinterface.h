/*
    Copyright 2016 - 2019 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#ifndef CARINTERFACE_H
#define CARINTERFACE_H

#include <QWidget>
#include <QVector>
#include <QTimer>
#include <QUdpSocket>
#include <QElapsedTimer>
#include "datatypes.h"
#include "mapwidget.h"
#include "packetinterface.h"
#include "tcpserversimple.h"
#include "imagewidget.h"

#ifdef HAS_OPENGL
#include "orientationwidget.h"
#endif

namespace Ui {
class CarInterface;
}

class CarInterface : public QWidget
{
    Q_OBJECT

public:
    explicit CarInterface(QWidget *parent = 0);
    ~CarInterface();
    void setID(int id);
    int getId();
    bool pollData();
    void setPollData(bool poll);
    bool updateRouteFromMap();
    void setOrientation(double roll, double pitch, double yaw);
    void setStateData(CAR_STATE data);
    void setMap(MapWidget *map);
    void setPacketInterface(PacketInterface *packetInterface);
    void setControlValues(double throttle, double steering, double max, bool currentMode);
    void emergencyStop();
    void setCtrlAp();
    void setCtrlKb();
    bool getCtrlKb();
    bool setAp(bool on, bool resetState = false);
    void setApMode(AP_MODE mode = AP_MODE_FOLLOW_ROUTE);
    void disableKbBox();
    void toggleCameraFullscreen();
    void showAutoPilotConfiguration();
    QPair<int,int> getFirmwareVersion();

    bool getResetApOnEmergencyStop() const;
    void setResetApOnEmergencyStop(bool value);

signals:
    void terminalCmd(quint8 id, QString cmd);
    void forwardVesc(quint8 id, QByteArray data);
    void setRcCurrent(quint8 id, double current, double steering);
    void setRcDuty(quint8 id, double duty, double steering);
    void showStatusInfo(QString str, bool isGood);
    void setServoDirect(quint8 id, double value);
    void ioBoardSetPwm(quint8 id, quint8 board, double value);

private slots:
    void timerSlot();
    void udpReadReady();
    void tcpRx(QByteArray &data);
    void terminalPrint(quint8 id, QString str);
    void vescFwdReceived(quint8 id, QByteArray data);
    void routePointSet(LocPoint pos);
    void lastRoutePointRemoved();
    void nmeaReceived(quint8 id, QByteArray nmea_msg);
    void configurationReceived(quint8 id, MAIN_CONFIG config);
    void loadMagCal();
    void cameraImageReceived(quint8 id, QImage image, int bytes);

    void on_terminalSendButton_clicked();
    void on_terminalSendVescButton_clicked();
    void on_terminalClearButton_clicked();
    void on_idBox_valueChanged(int arg1);
    void on_vescToolTcpBox_toggled(bool checked);
    void on_autopilotBox_toggled(bool checked);
    void on_clearRouteButton_clicked();
    void on_servoDirectSlider_valueChanged(int value);
    void on_servoMappedSlider_valueChanged(int value);
    void on_confReadButton_clicked();
    void on_confReadDefaultButton_clicked();
    void on_confWriteButton_clicked();
    void on_setClockButton_clicked();
    void on_setClockPiButton_clicked();
    void on_rebootPiButton_clicked();
    void on_shutdownPiButton_clicked();
    void on_ubxVersionButton_clicked();
    void on_ubxNavSatButton_clicked();
    void on_ubxSolButton_clicked();
    void on_ubxRelPosNedButton_clicked();
    void on_ubxCfgGnssButton_clicked();
    void on_uwbResetPosButton_clicked();
    void on_uwbUptimeButton_clicked();
    void on_zeroGyroButton_clicked();
    void on_uwbRebootButton_clicked();
    void on_uwbListAnchorsButton_clicked();
    void on_ioBoardPwmSlider_valueChanged(int value);

private:

    Ui::CarInterface *ui;
    MapWidget *mMap;
    PacketInterface *mPacketInterface;

#ifdef HAS_OPENGL
    OrientationWidget *mOrientationWidget;
#endif

    int mId;
    bool resetApOnEmergencyStop;
    CAR_STATE mLastCarState;
    QTimer *mTimer;
    QUdpSocket *mUdpSocket;
    QHostAddress mLastHostAddress;
    quint16 mUdpPort;
    TcpServerSimple *mTcpServer;
    QString mFaultLast;
    bool mSettingsReadDone;
    int mImageByteCnt;
    int mImageCnt;
    QElapsedTimer mImageTimer;
    double mImageFpsFilter;
    ImageWidget *mFullscreenImage;
    MAIN_CONFIG mConfigLast;
    QPair<int,int> mFirmwareVersion;

    void getConfGui(MAIN_CONFIG &conf);
    void setConfGui(MAIN_CONFIG &conf);
    void updateExperimentZoom();
    void setFirmwareVersion(QPair<int,int> firmwareVersion);

};

#endif // CARINTERFACE_H
