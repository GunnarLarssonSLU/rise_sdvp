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
#include <QtWidgets>

#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
    #include <SDL2/SDL.h>
    #include <QAction>
#else
    #include <QGamepad>
#endif

#include "carinterface.h"
#include "packetinterface.h"
#include "ping.h"
#include "nmeaserver.h"
#include "rtcm3_simple.h"
#include "intersectiontest.h"
#include "tcpclientmulti.h"
#include "wireguard.h"
#include <memory>
#include "arduinoreader.h"
#include "routegenerator.h"
#include "checkboxdelegate.h"
#include "database.h"

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
/*
class FocusEventFilter : public QObject
{
    Q_OBJECT

signals:
    void focusGained();
    void focusLost();

protected:
    bool eventFilter(QObject* watched, QEvent* event) override;
};

*/
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    bool eventFilter(QObject *object, QEvent *e);

    void addCar(int id, QString name, bool pollData = false);
    void removeCars();
    bool connectJoystick();
    void addTcpConnection(QString ip, int port);
    void setNetworkTcpEnabled(bool enabled, int port = -1);
    void setNetworkUdpEnabled(bool enabled, int port = -1);
    MapWidget *map();
    void addField();
    int currentFarm();
    void setCurrentFarm(int farm);
    void updateFarms();
    QLabel* getLogLabel();

public slots:

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

    void onSelectedFarm(const QModelIndex& current, const QModelIndex& previous);
    void onSelectedField(const QModelIndex& current, const QModelIndex& previous);
    void onSelectedFieldGeneral(QSqlRelationalTableModel *model,QSqlRelationalTableModel *modelPth,const QModelIndex& current, const QModelIndex& previous);
    void on_listLogFilesView_clicked(const QModelIndex& index);


    void handleAddFieldButton();
    void handleAddFarmButton();

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
//    void on_genCircButton_clicked();
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
    void on_mapDrawRouteTextBox_toggled(bool checked);
    void on_actionGPSSimulator_triggered();
    void on_mapDrawUwbTraceBox_toggled(bool checked);
    void on_actionToggleFullscreen_triggered();
    void on_actionToggleCameraFullscreen_triggered();
    void on_tabWidget_currentChanged(int index);
    void on_routeZeroButton_clicked();
    void on_routeZeroAllButton_clicked();
    void on_mapRoutePosAttrBox_currentIndexChanged(int index);
    void on_clearAnchorButton_clicked();
    void on_setBoundsRoutePushButton_clicked();
    void on_boundsFillPushButton_clicked();
    void on_lowerToolsCheckBox_stateChanged(int arg1);
    void on_raiseToolsCheckBox_stateChanged(int arg1);
    void on_AutopilotConfigurePushButton_clicked();
    void on_AutopilotStartPushButton_clicked();
    void on_AutopilotStopPushButton_clicked();
    void on_AutopilotRestartPushButton_clicked();
    void on_AutopilotPausePushButton_clicked();
    void onGeneratePathButtonClicked();
    void onGenerateLineButtonClicked();
    void onCutButtonClicked();
    void onTransformButtonClicked();
    void onAppendButtonClicked();
    void onPrependButtonClicked();
    bool onLoadShapefile();
    bool onShowShapefile();
    bool onLoadLogfile();

    QSqlRelationalTableModel* setupFarmTable(QTableView* uiFarmtable,QString SqlTableName);
    QSqlRelationalTableModel* setupFieldTable(QTableView* uiFieldtable,QString SqlTableName);
    QSqlRelationalTableModel* setupPathTable(QTableView* uiPathTable,QString sqlTablename);

private:
    QSqlRelationalTableModel *modelFarm;
    QSqlRelationalTableModel *modelField;
    QSqlRelationalTableModel *modelPath;
    CheckBoxDelegate* checkboxdelegate;
    QStringListModel* fileModel;  // Model to hold filenames
    QStringList fileList;         // Underlying data

    Ui::MainWindow *ui;
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
    ArduinoReader serialReader;
    database db;

#ifdef HAS_JOYSTICK_CHECK
    bool JSconnected();

    #if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
        SDL_GameController* mController = nullptr;
#else
        QGamepad *mJoystick;
    #endif
#endif

#ifdef HAS_SIM_SCEN
    PageSimScen *mSimScen;
#endif

    void saveRoutes(bool withId);

    CustomDelegate *statustocolourDelegate;

//    FocusEventFilter filterFieldtable;
//    FocusEventFilter filterPathtable;

private slots:
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
    void pollGamepad();
    void handleButtonEvent(const SDL_ControllerButtonEvent& event);
    void handleAxisEvent(const SDL_ControllerAxisEvent& event);
#endif

};

class CustomDelegate : public QStyledItemDelegate {
public:
    explicit CustomDelegate(QObject *parent = nullptr);

    void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
};

#include <QDoubleValidator>

class PrecisionDelegate : public QStyledItemDelegate {
    Q_OBJECT
public:
    explicit PrecisionDelegate(int column, int decimals = 10, QObject *parent = nullptr)
        : QStyledItemDelegate(parent), m_column(column), m_decimals(decimals) {}
    // Display high-precision numbers as text
    QString displayText(const QVariant &value, const QLocale &locale) const override {
        if (value.canConvert<double>())
            return locale.toString(value.toDouble(), 'f', m_decimals);
        return QStyledItemDelegate::displayText(value, locale);
    }

    // Use a QLineEdit for editing (not a spinbox)
    QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &,
                          const QModelIndex &index) const override {
        if (index.column() == m_column) {
            QLineEdit *editor = new QLineEdit(parent);
            auto *validator = new QDoubleValidator(editor);
            validator->setNotation(QDoubleValidator::StandardNotation);
            validator->setDecimals(m_decimals);
            editor->setValidator(validator);
            editor->setAlignment(Qt::AlignRight);
            editor->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
            return editor;
        }
        return QStyledItemDelegate::createEditor(parent, {}, index);
    }

    // Convert text back to double
    void setModelData(QWidget *editor, QAbstractItemModel *model,
                      const QModelIndex &index) const override {
        if (index.column() == m_column) {
            auto *line = qobject_cast<QLineEdit *>(editor);
            if (!line)
                return;
            bool ok;
            double d = line->text().toDouble(&ok);
            if (ok)
                model->setData(index, d, Qt::EditRole);
            return;
        }
        QStyledItemDelegate::setModelData(editor, model, index);
    }
private:
    int m_column;
    int m_decimals;
};


void selectRowByPrimaryKey(QTableView* tableView, QSqlRelationalTableModel* model, const QString& primaryKeyColumnName, const QVariant& primaryKeyValue);
MainWindow* findMainWindow();


#endif // MAINWINDOW_H
