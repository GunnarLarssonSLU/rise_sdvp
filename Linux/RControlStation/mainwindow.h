/*
    Copyright 2016-2017 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QList>
#include <QTimer>
#include <QSerialPort>
#include <QLabel>
#include <QTcpSocket>
#include <QGamepad>
#include <QtWidgets>
#include <QtSql>
//#include <QtCharts>
//#include <QtChartView>


#include "carinterface.h"
#include "packetinterface.h"
#include "ping.h"
#include "nmeaserver.h"
#include "rtcm3_simple.h"
#include "intersectiontest.h"
#include "tcpclientmulti.h"
#include "wireguard.h"
#include "checkboxdelegate.h"
#include <memory>
#include "rtcmclient.h"

// #include "rtcmwidget.h"

#ifdef HAS_LIME_SDR
#include "gpssim.h"
#endif

#ifdef HAS_SIM_SCEN
#include "pagesimscen.h"
#endif

//#ifdef HAS_JOYSTICK
//#include "joystick.h"
//#endif

namespace Ui {
class MainWindow;
}


class CustomDelegate;

class FocusEventFilter : public QObject
{
    Q_OBJECT

signals:
    void focusGained();
    void focusLost();

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
};

struct coords_cartesian
{
    float x,y,z;
};

struct coords_polar
{
    float lon,lat,h;
};

struct coords_matrix
{
    float r1c1,r1c2,r1c3;
    float r2c1,r2c2,r2c3;
    float r3c1,r3c2,r3c3;
};

// Constants
#define FE_WGS84						(D(1.0)/D(298.257223563)) // earth flattening (WGS84)
#define RE_WGS84						D(6378137.0)           // earth semimajor axis (WGS84) (m)


class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    bool eventFilter(QObject *object, QEvent *e);

    void addCar(int id, QString name, bool pollData = false);
    bool connectJoystick();
    void addTcpConnection(QString ip, int port);
    void setNetworkTcpEnabled(bool enabled, int port = -1);
    void setNetworkUdpEnabled(bool enabled, int port = -1);
    MapWidget *map();

    //RTCM
    void setRefPos(double lat, double lon, double height, double antenna_height = 0.0);


private slots:
    void serialDataAvailable();
    void serialPortError(QSerialPort::SerialPortError error);
    void timerSlot();
    void sendHeartbeat();
    void showStatusInfo(QString info, bool isGood);
    void packetDataToSend(QByteArray &data);
    void stateReceived(quint8 id, CAR_STATE state);
    void mapPosSet(quint8 id, LocPoint pos);
    void ackReceived(quint8 id, CMD_PACKET cmd, QString msg);
    void rtcmReceived(QByteArray data);
    void rtcmRefPosGet();
    void pingRx(int time, QString msg);
    void pingError(QString msg, QString error);
    void enuRx(quint8 id, double lat, double lon, double height);
    void nmeaGgaRx(int fields, NmeaServer::nmea_gga_info_t gga);
    void routePointAdded(LocPoint pos);
    void infoTraceChanged(int traceNow);
    void jsButtonChanged(int button, bool pressed);

//    void on_carAddButton_clicked();
    void on_disconnectButton_clicked();
    void on_mapRemoveTraceButton_clicked();
    void on_MapRemovePixmapsButton_clicked();
    void on_tcpConnectButton_clicked();
    void on_tcpPingButton_clicked();
    void on_mapZeroButton_clicked();
    void on_mapRemoveRouteButton_clicked();
    void on_mapRouteSpeedBox_valueChanged(double arg1);
    void on_jsConnectButton_clicked();
    void on_jsDisconnectButton_clicked();
    void on_mapAntialiasBox_toggled(bool checked);
    void on_carsWidget_tabCloseRequested(int index);
    void on_genCircButton_clicked();
    void on_mapSetAbsYawButton_clicked();
    void on_mapAbsYawSlider_valueChanged(int value);
    void on_mapAbsYawSlider_sliderReleased();
    void on_stopButton_clicked();
    void on_mapUploadRouteButton_clicked();
    void on_mapGetRouteButton_clicked();
    void on_mapApButton_clicked();
    void on_mapKbButton_clicked();
    void on_mapOffButton_clicked();
    void on_mapUpdateSpeedButton_clicked();
    void on_mapOpenStreetMapBox_toggled(bool checked);
    void on_mapAntialiasOsmBox_toggled(bool checked);
    void on_mapOsmResSlider_valueChanged(int value);
    void on_mapChooseNmeaButton_clicked();
    void on_mapImportNmeaButton_clicked();
    void on_mapRemoveInfoAllButton_clicked();
    void on_traceInfoMinZoomBox_valueChanged(double arg1);
    void on_removeRouteExtraButton_clicked();
    void on_mapOsmClearCacheButton_clicked();
    void on_mapOsmServerOsmButton_toggled(bool checked);
    void on_mapOsmServerHiResButton_toggled(bool checked);
    void on_mapOsmServerVedderButton_toggled(bool checked);
    void on_mapOsmServerVedderHdButton_toggled(bool checked);
    void on_mapOsmMaxZoomBox_valueChanged(int arg1);
    void on_mapDrawGridBox_toggled(bool checked);
    void on_mapGetEnuButton_clicked();
    void on_mapSetEnuButton_clicked();
    void on_mapOsmStatsBox_toggled(bool checked);
    void on_removeTraceExtraButton_clicked();
    void on_mapEditHelpButton_clicked();
    void on_mapStreamNmeaConnectButton_clicked();
    void on_mapStreamNmeaDisconnectButton_clicked();
    void on_mapStreamNmeaClearTraceButton_clicked();
    void on_mapRouteBox_valueChanged(int arg1);
    void on_mapRemoveRouteAllButton_clicked();
    void on_mapUpdateTimeButton_clicked();
    void on_mapRouteTimeEdit_timeChanged(const QTime &time);
    void on_mapTraceMinSpaceCarBox_valueChanged(double arg1);
    void on_mapTraceMinSpaceGpsBox_valueChanged(double arg1);
    void on_mapInfoTraceBox_valueChanged(int arg1);
    void on_removeInfoTraceExtraButton_clicked();
    void on_pollIntervalBox_valueChanged(int arg1);
    void on_actionAbout_triggered();
    void on_actionAboutLibrariesUsed_triggered();
    void on_actionExit_triggered();
    void on_actionSaveRoutes_triggered();
    void on_actionSaveRouteswithIDs_triggered();
    void on_actionLoadRoutes_triggered();
    void on_actionTestIntersection_triggered();
    void on_actionSaveSelectedRouteAsDriveFile_triggered();
    void on_actionLoadDriveFile_triggered();
//    void on_mapSaveAsPdfButton_clicked();
//    void on_mapSaveAsPngButton_clicked();
//    void on_mapSaveRetakeButton_clicked();
    void on_modeRouteButton_toggled(bool checked);
    void on_uploadAnchorButton_clicked();
    void on_anchorIdBox_valueChanged(int arg1);
    void on_anchorHeightBox_valueChanged(double arg1);
    void on_removeAnchorsButton_clicked();
    void on_mapDrawRouteTextBox_toggled(bool checked);
    void on_actionGPSSimulator_triggered();
    void on_mapDrawUwbTraceBox_toggled(bool checked);
    void on_actionToggleFullscreen_triggered();
    void on_mapCameraWidthBox_valueChanged(double arg1);
    void on_mapCameraOpacityBox_valueChanged(double arg1);
    void on_actionToggleCameraFullscreen_triggered();
    void on_tabWidget_currentChanged(int index);
    void on_routeZeroButton_clicked();
    void on_routeZeroAllButton_clicked();
    void on_mapRoutePosAttrBox_currentIndexChanged(int index);
    void on_clearAnchorButton_clicked();
    void on_boundsFillPushButton_clicked();

    void on_lowerToolsCheckBox_stateChanged(int arg1);

    void on_raiseToolsCheckBox_stateChanged(int arg1);

    void on_WgConnectPushButton_clicked();

    void on_WgDisconnectPushButton_clicked();

    void on_AutopilotConfigurePushButton_clicked();

    void on_AutopilotStartPushButton_clicked();

    void on_AutopilotStopPushButton_clicked();

    void on_AutopilotRestartPushButton_clicked();

    void on_AutopilotPausePushButton_clicked();

    void on_actionWireguard_triggered();

    void onLoadLogFile();


    // RTCM
    void timerSlotRtcm();
    void rtcmRx(QByteArray data, int type, bool sync);
    void refPosRx(double lat, double lon, double height, double antenna_height);

    void on_ntripDisconnectButton_clicked();
    void ntripConnect(int rowIndex);

    void on_refGetButton_clicked();
    void on_tcpServerBox_toggled(bool checked);
    void on_gpsOnlyBox_toggled(bool checked);

signals:
    void rtcmReceivedStep1(QByteArray data);
    void refPosGet();

public slots:
    void on_ntripConnectButton_clicked();
    void onMouseClickedFieldSelected(int field);

private:
    Ui::MainWindow *ui;
//    RtcmWidget *rtcmWidget;

    QTimer *mTimer;
    QTimer *mHeartbeatTimer; // periodic heartbeat to vehicles for safety
    const int mHeartbeatMS = 300;
    QSerialPort *mSerialPort;
    PacketInterface *mPacketInterface;
    QList<CarInterface*> mCars;
    QLabel *mStatusLabel;
    int mStatusInfoTime;
    bool mKeyUp;
    bool mKeyDown;
    bool mKeyRight;
    bool mKeyLeft;
    double mThrottle;
    double mSteering;
    Ping *mPing;
    NmeaServer *mNmea;
    QUdpSocket *mUdpSocket;
    TcpClientMulti *mTcpClientMulti;
    QString mVersion;
    rtcm3_state mRtcmState;
    IntersectionTest *mIntersectionTest;
    std::unique_ptr<WireGuard> mWireGuard;
    QString mLastImgFileName;
    QList<QPair<int, int> > mSupportedFirmwares;
    CheckBoxDelegate* checkboxdelegate;

#ifdef HAS_JOYSTICK_CHECK
    QGamepad *mJoystick;
#endif

#ifdef HAS_LIME_SDR
    GpsSim *mGpsSim;
#endif

#ifdef HAS_SIM_SCEN
    PageSimScen *mSimScen;
#endif

    void saveRoutes(bool withId);

protected:
//    bool eventFilter(QObject *object, QEvent *event) override;

private:
    void showError(const QSqlError &err);
    void handleFieldButton();
    void handleLocationButton();
    QSqlRelationalTableModel *modelFarm;
    QSqlRelationalTableModel *modelField;
    QSqlRelationalTableModel *modelPath;
    QSqlRelationalTableModel *modelVehicle;
    QStandardItemModel *modelVehiclestatus;

    // RTCM
    RtcmClient *mRtcm;
    QTimer *mTimerRtcm;
    TcpBroadcast *mTcpServer;
    CustomDelegate *statustocolourDelegate;

    FocusEventFilter filterFieldtable;
    FocusEventFilter filterPathtable;
};

class CustomDelegate : public QStyledItemDelegate {
public:
    explicit CustomDelegate(QObject *parent = nullptr);

    void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
};



void addField(QSqlQuery &q, const QString &title, const QVariant &locationId);
QVariant addLocation(QSqlQuery &q, const QString &name);
void deleteField(const QVariant &fieldId);

coords_matrix toOrientationMatrix(coords_polar cp);
coords_cartesian toCartesian(coords_polar cp);
coords_cartesian toEnu(coords_polar basestation,coords_polar vehicle);
coords_cartesian toEnu(coords_cartesian basestation,coords_matrix orientation,coords_polar vehicle);

QSqlError initDb();


#endif // MAINWINDOW_H
