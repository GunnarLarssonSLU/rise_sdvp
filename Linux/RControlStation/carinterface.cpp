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

#include "carinterface.h"
#include "ui_carinterface.h"
#include "carinfo.h"
#include "utility.h"
#include "routemagic.h"
#include <QFileDialog>
#include <QMessageBox>
#include <cmath>
#include <QTime>
#include <QDateTime>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>

namespace {
void faultToStr(mc_fault_code fault, QString &str, bool &isOk)
{
    switch (fault) {
    case FAULT_CODE_NONE: str = "FAULT_CODE_NONE"; isOk = true; break;
    case FAULT_CODE_OVER_VOLTAGE: str = "FAULT_CODE_OVER_VOLTAGE"; isOk = false; break;
    case FAULT_CODE_UNDER_VOLTAGE: str = "FAULT_CODE_UNDER_VOLTAGE"; isOk = false; break;
    case FAULT_CODE_DRV8302: str = "FAULT_CODE_DRV8302"; isOk = false; break;
    case FAULT_CODE_ABS_OVER_CURRENT: str = "FAULT_CODE_ABS_OVER_CURRENT"; isOk = false; break;
    case FAULT_CODE_OVER_TEMP_FET: str = "FAULT_CODE_OVER_TEMP_FET"; isOk = false; break;
    case FAULT_CODE_OVER_TEMP_MOTOR: str = "FAULT_CODE_OVER_TEMP_MOTOR"; isOk = false; break;
    default: str = "Unknown fault"; isOk = false; break;
    }
}
}

CarInterface::CarInterface(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::CarInterface)
{
    ui->setupUi(this);

#ifdef HAS_OPENGL
    /*
    mOrientationWidget = new OrientationWidget(this);
    ui->orientationLayout->removeItem(ui->orientationSpacer);
    ui->orientationLayout->insertWidget(0, mOrientationWidget, 1);
    */
#endif

    memset(&mLastCarState, 0, sizeof(CAR_STATE));

    mMap = 0;
    mPacketInterface = 0;
    mId = 0;
    resetApOnEmergencyStop = true;
    mSettingsReadDone = false;
    mImageByteCnt = 0;
    mImageCnt = 0;
    mImageFpsFilter = 0.0;
    mFullscreenImage = 0;

    mTimer = new QTimer(this);
    mTimer->start(20);

    mUdpSocket = new QUdpSocket(this);
    mLastHostAddress.clear();
    mUdpPort = 27800;
    mTcpServer = new TcpServerSimple(this);
    mFaultLast = "Fault code...";

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    connect(mUdpSocket, SIGNAL(readyRead()), this, SLOT(udpReadReady()));
    connect(mTcpServer->packet(), SIGNAL(packetReceived(QByteArray&)),
            this, SLOT(tcpRx(QByteArray&)));
    connect(ui->confCommonWidget, SIGNAL(loadMagCal()),
            this, SLOT(loadMagCal()));

    mTcpServer->setUsePacket(true);
}

CarInterface::~CarInterface()
{
    if (mMap) {
        mMap->removeCar(mId);
    }

    if (mFullscreenImage) {
        delete mFullscreenImage;
    }

    delete ui;
}

void CarInterface::setID(int id)
{
    ui->idBox->setValue(id);
}

int CarInterface::getId()
{
    return mId;
}

bool CarInterface::pollData()
{
    return ui->pollBox->isChecked();
}

void CarInterface::setPollData(bool poll)
{
    ui->pollBox->setChecked(poll);
}

bool CarInterface::updateRouteFromMap()
{
    return ui->updateRouteFromMapBox->isChecked();
}

void CarInterface::setOrientation(double roll, double pitch, double yaw)
{
#ifdef HAS_OPENGL
    ui->rollBar->setValue(roll);
    ui->pitchBar->setValue(pitch);
    ui->yawBar->setValue(yaw);

    mOrientationWidget->setRollPitchYaw(roll, pitch, yaw);
#else
    (void)roll;
    (void)pitch;
    (void)yaw;
#endif
}

void CarInterface::setStateData(CAR_STATE data)
{
    mLastCarState = data;

    ui->imuPlot->addSample(data.accel, data.gyro, data.mag);

    // Firmware label
    QString fwStr;
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
    fwStr = QString("FW %1.%2").arg(QString::number(data.fw_major)).arg(QString::number(data.fw_minor));
#else
    fwStr.sprintf("FW %d.%d", data.fw_major, data.fw_minor);
#endif
    ui->fwLabel->setText(fwStr);
    setFirmwareVersion(qMakePair(data.fw_major, data.fw_minor));

    QString fwStr2;
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
    fwStr2 = QString("FW %1.%2").arg(QString::number(data.fw_major)).arg(QString::number(data.fw_minor));
#else
    fwStr2.sprintf("%d.%d", data.fw_major, data.fw_minor);
#endif
    ui->fwLabel->setText(fwStr2);

    QString fwStrLog1;
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
    fwStrLog1 = QString("FW %1.%2").arg(QString::number(data.fw_major)).arg(QString::number(data.fw_minor));
#else
    fwStrLog1.sprintf("%d.%d", data.fw_major, data.fw_minor);
#endif
    ui->fwLabel->setText(fwStrLog1);


    // Speed bar
    QString speedTxt;
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
    speedTxt = QString("Speed: %1 km/h").arg(data.speed * 3.6, 0, 'f', 2);
#else
    speedTxt.sprintf("Speed: %.2f km/h", data.speed * 3.6);
#endif
    ui->speedBar->setValue(fabs(data.speed) * 3.6);
    ui->speedBar->setFormat(speedTxt);

    // Temp FET bar
    ui->tempFetBar->setValue(data.temp_fet);

    // Battery bar
    double battp = utility::map(data.vin, 34.0, 42.0, 0.0, 100.0);
    if (battp < 0.0) {
        battp = 0.0;
    }
    QString battTxt;


//    battTxt.sprintf("Battery: %.1f %% (%.2f V), Sensor value: %u, Angle: %.1f, Servo output: %.1f, Debug: %.1f", battp, data.sensor_value, data.vin,data.angle,data.servo_output,data.debugvalue);
//    battTxt.sprintf("Location: %.2f, %.2f , Gps location: %.2f, %.2f", data.px, data.py,data.px_gps, data.py_gps);
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
    battTxt = QString("Battery: %1 %% (%2 V), Sensor value: %3, Angle: %4, Servo output: %5, Debug: %6").arg(battp, 0, 'f', 1).arg(QString::number(data.vin, 'f', 2)).arg(QString::number(data.sensor_value, 'f', 2)).arg(QString::number(data.angle, 'f', 1)).arg(QString::number(data.servo_output, 'f', 1)).arg(QString::number(data.debugvalue, 'f', 1));
#else
    battTxt.sprintf("Battery: %.1f %% (%.2f V), Sensor value: %u, Angle: %.1f, Servo output: %.1f, Debug: %.1f", battp, data.vin, data.sensor_value,data.angle,data.servo_output,data.debugvalue);
#endif

    if (battp > 100.0) {
        battp = 100.0;
    }
    ui->batteryBar->setValue((int)battp);
    ui->batteryBar->setFormat(battTxt);

    // Orientation
//    setOrientation(data.roll, data.pitch, data.yaw);

    // Fault label
    QString fault_str;
    bool isOk;
    faultToStr(data.mc_fault, fault_str, isOk);

    if (mFaultLast != fault_str) {
        mFaultLast = fault_str;
        ui->mcFaultLabel->setText(fault_str);
        if (isOk) {
            ui->mcFaultLabel->setStyleSheet("QLabel { background-color : lightgreen; color : black; }");
        } else {
            ui->mcFaultLabel->setStyleSheet("QLabel { background-color : red; color : black; }");

            QString msg;
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
            msg = QString("Car %1: ").arg(QString::number(mId));
#else
            msg.sprintf("Car %d: ", mId);
#endif
            emit showStatusInfo(msg + fault_str, false);
        }
    }

    if (mMap) {
        CarInfo *car = mMap->getCarInfo(mId);
        LocPoint loc = car->getLocation();
        LocPoint loc_gps = car->getLocationGps();
        LocPoint loc_uwb = car->getLocationUwb();
        LocPoint ap_goal = car->getApGoal();
        loc.setYaw(data.yaw * M_PI / 180.0);
        loc.setXY(data.px, data.py);
        loc.setSpeed(data.speed);
        loc_gps.setXY(data.px_gps, data.py_gps);
        loc_uwb.setXY(data.px_uwb, data.py_uwb);
        ap_goal.setXY(data.ap_goal_px, data.ap_goal_py);
        ap_goal.setRadius(data.ap_rad);
        car->setLocation(loc);
//        qDebug() << "state";
        car->setLocationGps(loc_gps);
        car->setLocationUwb(loc_uwb);
        car->setApGoal(ap_goal);
        car->setTime(data.ms_today);

//        //QList<LocPoint> bounds = mMap->getRoute(ui->boundsRouteSpinBox->value());
//        QList<LocPoint> bounds = mMap->getRoute(0);
//        if (RouteMagic::isPointOutside(loc, bounds))
//        {
//        	setAp(false, false);
//        }


//        //QList<LocPoint> bounds = mMap->getRoute(ui->boundsRouteSpinBox->value());
//        QList<LocPoint> bounds = mMap->getRoute(0);
//        if (RouteMagic::isPointOutside(loc, bounds))
//        {
//        	setAp(false, false);
//        }


//        void MainWindow::on_AutopilotPausePushButton_clicked()
//        {
//            QWidget *tmp = ui->carsWidget->widget(ui->mapCarBox->value());
//            if (tmp) {
//                CarInterface *car = dynamic_cast<CarInterface*>(tmp);
//                car->setAp(false, false);
//            }
//        }

        mMap->update();
    }

    ui->magCal->addSample(data.mag[0], data.mag[1], data.mag[2]);

    // Clock
    if (data.ms_today >= 0) {
        QTime time = QTime::fromMSecsSinceStartOfDay(data.ms_today);
        QDateTime date = QDateTime::currentDateTime();
        QTime current = QTime::currentTime().addSecs(-date.offsetFromUtc());

        int diff = data.ms_today - current.msecsSinceStartOfDay();
        ui->clockLabel->setText(time.toString("HH:mm:ss:zzz") + " " +
                                QString("%1").arg(diff, 6, 10, QChar('0')) + " ms");
    } else {
        ui->clockLabel->setText("00:00:00:000");
    }
    /*
    */
}

void CarInterface::setMap(MapWidget *map)
{
    mMap = map;
    CarInfo car(mId);
    mMap->addCar(car);

    connect(mMap, SIGNAL(routePointAdded(LocPoint)),
            this, SLOT(routePointSet(LocPoint)));
    connect(mMap, SIGNAL(lastRoutePointRemoved(LocPoint)),
            this, SLOT(lastRoutePointRemoved()));
}

void CarInterface::setPacketInterface(PacketInterface *packetInterface)
{
    mPacketInterface = packetInterface;

    connect(this, SIGNAL(terminalCmd(quint8,QString)),
            mPacketInterface, SLOT(sendTerminalCmd(quint8,QString)));
    connect(mPacketInterface, SIGNAL(printReceived(quint8,QString)),
            this, SLOT(terminalPrint(quint8,QString)));
    connect(this, SIGNAL(forwardVesc(quint8,QByteArray)),
            mPacketInterface, SLOT(forwardVesc(quint8,QByteArray)));
    connect(mPacketInterface, SIGNAL(vescFwdReceived(quint8,QByteArray)),
            this, SLOT(vescFwdReceived(quint8,QByteArray)));
    connect(this, SIGNAL(setRcCurrent(quint8,double,double)),
            mPacketInterface, SLOT(setRcControlCurrent(quint8,double,double)));
    connect(this, SIGNAL(setRcDuty(quint8,double,double)),
            mPacketInterface, SLOT(setRcControlDuty(quint8,double,double)));
    connect(this, SIGNAL(setServoDirect(quint8,double)),
            mPacketInterface, SLOT(setServoDirect(quint8,double)));
    connect(mPacketInterface, SIGNAL(nmeaRadioReceived(quint8,QByteArray)),
            this, SLOT(nmeaReceived(quint8,QByteArray)));
    connect(mPacketInterface, SIGNAL(configurationReceived(quint8,MAIN_CONFIG)),
            this, SLOT(configurationReceived(quint8,MAIN_CONFIG)));
/*    connect(mPacketInterface, SIGNAL(plotInitReceived(quint8,QString,QString)),
            this, SLOT(plotInitReceived(quint8,QString,QString)));
    connect(mPacketInterface, SIGNAL(plotDataReceived(quint8,double,double)),
            SLOT(plotDataReceived(quint8,double,double)));
    connect(mPacketInterface, SIGNAL(plotAddGraphReceived(quint8,QString)),
            this, SLOT(plotAddGraphReceived(quint8,QString)));
    connect(mPacketInterface, SIGNAL(plotSetGraphReceived(quint8,int)),
            this, SLOT(plotSetGraphReceived(quint8,int)));*/
    connect(mPacketInterface, SIGNAL(cameraImageReceived(quint8,QImage,int)),
            this, SLOT(cameraImageReceived(quint8,QImage,int)));
    connect(this, SIGNAL(ioBoardSetPwm(quint8,quint8,double)),
            mPacketInterface, SLOT(ioBoardSetPwmDuty(quint8,quint8,double)));
}

void CarInterface::setControlValues(double throttle, double steering, double max, bool currentMode)
{
    if (ui->keyboardControlBox->isChecked()) {
        if (fabs(throttle) < 0.005) {
            emit setRcCurrent(mId, 0.0, steering);
        } else {
            if (currentMode) {
                emit setRcCurrent(mId, throttle * 80.0 * max, steering);
            } else {
                emit setRcDuty(mId, throttle * max, steering);
            }
        }
    }
}

void CarInterface::emergencyStop()
{
    if (ui->autopilotBox->isChecked()) {
        ui->autopilotBox->setChecked(false);
    } else {
        // Send the AP stop command even if the autopilot was not active in the UI.
        if (mPacketInterface) {
            mPacketInterface->setApActive(mId, false, resetApOnEmergencyStop);
        }
    }

    ui->keyboardControlBox->setChecked(false);
}

void CarInterface::setCtrlAp()
{
    ui->keyboardControlBox->setChecked(false);
    ui->autopilotBox->setChecked(true);
}

void CarInterface::setCtrlKb()
{
    ui->autopilotBox->setChecked(false);
    ui->keyboardControlBox->setChecked(true);
}

bool CarInterface::getCtrlKb()
{
    return ui->keyboardControlBox->isChecked();
}

bool CarInterface::setAp(bool on, bool resetState)
{
    bool ok = false;

    if (mPacketInterface) {
        ok = mPacketInterface->setApActive(mId, on, resetState);

        if (ok) {
            if (on)
                ui->keyboardControlBox->setChecked(false);
            //ui->autopilotBox->setChecked(on);
        }
    }

    return ok;
}

void CarInterface::setApMode(AP_MODE mode)
{
    if (mPacketInterface)
        mPacketInterface->setApMode(mId, mode);
}

void CarInterface::disableKbBox()
{
    ui->keyboardControlBox->setChecked(false);
}

void CarInterface::toggleCameraFullscreen()
{
    if (mFullscreenImage) {
        delete mFullscreenImage;
    } else {
        mFullscreenImage = new ImageWidget();
        mFullscreenImage->setWindowFlags(mFullscreenImage->windowFlags() |
                                         Qt::WindowStaysOnTopHint);
        mFullscreenImage->showFullScreen();

        connect(mFullscreenImage, &ImageWidget::destroyed, [=]() {
            mFullscreenImage = 0;
        });
    }
}

void CarInterface::showAutoPilotConfiguration()
{
    on_confReadButton_clicked();
    ui->tabWidget->setCurrentIndex(ui->tabWidget->indexOf(ui->tab_5));
    ui->confCommonWidget->showAutoPilotConfiguration();

}

QPair<int,int> CarInterface::getFirmwareVersion()
{
    return mFirmwareVersion;
}

void CarInterface::setFirmwareVersion(QPair<int,int> firmwareVersion)
{
    mFirmwareVersion = firmwareVersion;
}

void CarInterface::timerSlot()
{   
}

void CarInterface::udpReadReady()
{
    while (mUdpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(mUdpSocket->pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort;

        mUdpSocket->readDatagram(datagram.data(), datagram.size(),
                                 &sender, &senderPort);
        mLastHostAddress = sender;

        emit forwardVesc(mId, datagram);
    }
}

void CarInterface::tcpRx(QByteArray &data)
{
    emit forwardVesc(mId, data);
}

void CarInterface::terminalPrint(quint8 id, QString str)
{
    if (id == mId || id == 255) {
        ui->terminalBrowser->append(str);
    }
}

void CarInterface::vescFwdReceived(quint8 id, QByteArray data)
{
    if (id == mId) {
        if (QString::compare(mLastHostAddress.toString(), "0.0.0.0") != 0) {
            mUdpSocket->writeDatagram(data, mLastHostAddress, mUdpPort + 1);
        }

        mTcpServer->packet()->sendPacket(data);
    }
}

void CarInterface::routePointSet(LocPoint pos)
{
    if (mMap && mPacketInterface && ui->updateRouteFromMapBox->isChecked()) {
        QList<LocPoint> points;
        points.append(pos);

        mMap->setEnabled(false);
        bool ok = mPacketInterface->setRoutePoints(mId, points);
        mMap->setEnabled(true);

        if (!ok) {
            QMessageBox::warning(this, "Autopilot",
                                 "No ack received, so the the last route point was most likely not set.");
        }
    }
}

void CarInterface::lastRoutePointRemoved()
{
    if (mMap && mPacketInterface && ui->updateRouteFromMapBox->isChecked()) {
        mMap->setEnabled(false);
        bool ok = mPacketInterface->removeLastRoutePoint(mId);
        mMap->setEnabled(true);

        if (!ok) {
            QMessageBox::warning(this, "Autopilot",
                                 "No ack received, so the the last route point was most likely not removed.");
        }
    }
}

void CarInterface::nmeaReceived(quint8 id, QByteArray nmea_msg)
{
//    if (id == mId) {
        ui->nmeaWidget->inputNmea(nmea_msg);

        if (mMap) {
            CarInfo *car = mMap->getCarInfo(mId);

            if (car) {
                LocPoint loc_gps = car->getLocationGps();
                loc_gps.setInfo(ui->nmeaWidget->fixType());
//                qDebug() << "nmea";
                car->setLocationGps(loc_gps);
            }
        }
//    }
}

void CarInterface::configurationReceived(quint8 id, MAIN_CONFIG config)
{
    if (id == mId) {
        mSettingsReadDone = true;
        mConfigLast = config;
        setConfGui(config);
        QString str;
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
        str = QString("Car %1: Configuration Received").arg(QString::number(id));
#else
        str.sprintf("Car %d: Configuration Received", id);
#endif
        emit showStatusInfo(str, true);
    }
}

void CarInterface::loadMagCal()
{
    if (!ui->magCal->calculateCompensation()) {
        QMessageBox::warning(this, "Load Magnetometer Calibration",
                             "Magnetometer calibration is not done. Please go to "
                             "the calibration tab and collect "
                             "samples, or load a file.");
        return;
    }

    ui->confCommonWidget->setMagComp(ui->magCal->getComp());
    ui->confCommonWidget->setMagCompCenter(ui->magCal->getCenter());
}

void CarInterface::cameraImageReceived(quint8 id, QImage image, int bytes)
{
    if (id == mId || id == 255) {
        mImageByteCnt += bytes;
        mImageCnt++;

        mPacketInterface->sendCameraFrameAck(mId);

        mImageFpsFilter -= 0.1 * (mImageFpsFilter - 1000.0 / (double)mImageTimer.restart());

        ui->camInfoLabel->setText(QString("Total RX: %1 MB | Last RX: %2 KB | "
                                          "IMG CNT: %3 | FPS: %4").
                                  arg((double)mImageByteCnt / 1024.0 / 1024.0, 0, 'f', 1).
                                  arg(bytes / 1024).
                                  arg(mImageCnt).arg(mImageFpsFilter, 0, 'f', 1));

        if (mFullscreenImage) {
            mFullscreenImage->setPixmap(QPixmap::fromImage(image));
        } else {
            ui->camWidget->setPixmap(QPixmap::fromImage(image));

/*            if (mMap && ui->camShowMapBox->isChecked()) {
                mMap->setLastCameraImage(image);
            }*/
        }
    }
}

void CarInterface::on_terminalSendButton_clicked()
{
    emit terminalCmd(mId, ui->terminalEdit->text());
    ui->terminalEdit->clear();
}

void CarInterface::on_terminalSendVescButton_clicked()
{
    emit terminalCmd(mId, "vesc " + ui->terminalEditVesc->text());
    ui->terminalEditVesc->clear();
}

void CarInterface::on_terminalClearButton_clicked()
{
    ui->terminalBrowser->clear();
}

void CarInterface::on_idBox_valueChanged(int arg1)
{
    if (mMap) {
        CarInfo *car = mMap->getCarInfo(mId);
        car->setId(arg1, true);
    }

    mId = arg1;
}

void CarInterface::on_vescToolTcpBox_toggled(bool checked)
{
    if (checked) {
        if (!mTcpServer->startServer(65102)) {
            qWarning() << "Starting TCP server failed:" << mTcpServer->errorString();
            QMessageBox::warning(this, "TCP Server Error",
                                 tr("Starting TCP server failed. Make sure that the port is not "
                                    "already in use. Error: %1").arg(mTcpServer->errorString()));
            ui->vescToolTcpBox->setChecked(false);
        }
    } else {
        mTcpServer->stopServer();
    }
}

void CarInterface::on_autopilotBox_toggled(bool checked)
{
    if (!ui->autopilotBox->isEnabled()) {
        return;
    }

    if (mPacketInterface) {
        ui->autopilotBox->setEnabled(false);
        bool ok = mPacketInterface->setApActive(mId, checked);

        if (!ok) {
            ui->autopilotBox->setChecked(!checked);
            QMessageBox::warning(this, "Autopilot",
                                 "No ack received, so the autopilot state is unknown.");
        }

        ui->autopilotBox->setEnabled(true);
    }
}

void CarInterface::on_clearRouteButton_clicked()
{
    if (mPacketInterface) {
        ui->clearRouteButton->setEnabled(false);
        bool ok = mPacketInterface->clearRoute(mId);
        ui->clearRouteButton->setEnabled(true);

        if (!ok) {
            QMessageBox::warning(this, "Autopilot",
                                 "No ack received on clear route, so the route is most likely not cleared.");
        }
    }
}

void CarInterface::on_servoDirectSlider_valueChanged(int value)
{
    double val_mapped = (double)value / 1000.0;
    ui->servoDirectNumber->display(val_mapped);
    emit setServoDirect(mId, val_mapped);
}

void CarInterface::on_servoMappedSlider_valueChanged(int value)
{
    double val_mapped = (double)value / 1000.0;
    ui->servoMappedNumber->display(val_mapped);
    emit setRcCurrent(mId, 0.0, val_mapped);
}

void CarInterface::on_confReadButton_clicked()
{
    if (mPacketInterface) {
        mPacketInterface->getConfiguration(mId);
    }
}

void CarInterface::on_confReadDefaultButton_clicked()
{
    if (mPacketInterface) {
        mPacketInterface->getDefaultConfiguration(mId);
    }
}

void CarInterface::on_confWriteButton_clicked()
{
    if (!mSettingsReadDone) {
        QMessageBox::warning(this, "Configuration",
                             "You must read the configuration at least once before writing it. "
                             "Otherwise everything would be set to 0.");
        return;
    }

    if (mPacketInterface) {
        MAIN_CONFIG conf = mConfigLast;
        getConfGui(conf);
        ui->confWriteButton->setEnabled(false);
        bool ok = mPacketInterface->setConfiguration(mId, conf, 5);
        ui->confWriteButton->setEnabled(true);

        if (!ok) {
            QMessageBox::warning(this, "Configuration",
                                 "Could not write configuration.");
        }
    }
}

void CarInterface::getConfGui(MAIN_CONFIG &conf)
{
    conf.car.yaw_use_odometry = ui->confOdometryYawBox->isChecked();
    conf.car.yaw_imu_gain = ui->confYawImuGainBox->value();
    conf.car.disable_motor = ui->confMiscDisableMotorBox->isChecked();
    conf.car.simulate_motor = ui->confMiscSimulateMotorBox->isChecked();
    conf.car.clamp_imu_yaw_stationary = ui->confClampImuYawBox->isChecked();
    conf.car.use_uwb_pos = ui->confUseUwbPosBox->isChecked();

    conf.car.gear_ratio = ui->confGearRatioBox->value();
    conf.car.wheel_diam = ui->confWheelDiamBox->value();
    conf.car.motor_poles = ui->confMotorPoleBox->value();
    conf.car.steering_center = ui->confServoCenterBox->value();
    conf.car.steering_range = ui->confServoRangeBox->value();
    conf.car.steering_ramp_time = ui->confSteeringRampBox->value();
    conf.car.axis_distance = ui->confAxisDistanceBox->value();
    conf.car.vesc_p_gain = ui->confServoPGainBox->value();
    conf.car.vesc_i_gain = ui->confServoIGainBox->value();
    conf.car.vesc_d_gain = ui->confServoDGainBox->value();
    conf.car.sensorcentre = ui->confSensorCentreBox->value();
    conf.car.sensorinterval = ui->confSensorIntervalBox->value();
    conf.car.degreeinterval = ui->confDegreeIntervalBox->value();

    conf.car.steering_max_angle_rad = atan(ui->confAxisDistanceBox->value() / ui->confTurnRadBox->value());

    ui->confCommonWidget->getConfGui(conf);
}

void CarInterface::setConfGui(MAIN_CONFIG &conf)
{
    ui->confOdometryYawBox->setChecked(conf.car.yaw_use_odometry);
    ui->confYawImuGainBox->setValue(conf.car.yaw_imu_gain);
    ui->confMiscDisableMotorBox->setChecked(conf.car.disable_motor);
    ui->confMiscSimulateMotorBox->setChecked(conf.car.simulate_motor);
    ui->confClampImuYawBox->setChecked(conf.car.clamp_imu_yaw_stationary);
    ui->confUseUwbPosBox->setChecked(conf.car.use_uwb_pos);

    ui->confGearRatioBox->setValue(conf.car.gear_ratio);
    ui->confWheelDiamBox->setValue(conf.car.wheel_diam);
    ui->confMotorPoleBox->setValue(conf.car.motor_poles);
    ui->confServoCenterBox->setValue(conf.car.steering_center);
    ui->confServoRangeBox->setValue(conf.car.steering_range);
    ui->confSteeringRampBox->setValue(conf.car.steering_ramp_time);
    ui->confAxisDistanceBox->setValue(conf.car.axis_distance);
    ui->confServoPGainBox->setValue(conf.car.vesc_p_gain);
    ui->confServoIGainBox->setValue(conf.car.vesc_i_gain);
    ui->confServoDGainBox->setValue(conf.car.vesc_d_gain);
    ui->confSensorCentreBox->setValue(conf.car.sensorcentre);
    ui->confSensorIntervalBox->setValue(conf.car.sensorinterval);
    ui->confDegreeIntervalBox->setValue(conf.car.degreeinterval);

    ui->confTurnRadBox->setValue(conf.car.axis_distance / tan(conf.car.steering_max_angle_rad));

    ui->confCommonWidget->setConfGui(conf);
}

void CarInterface::on_setClockButton_clicked()
{
    if (mPacketInterface) {
        QDateTime date = QDateTime::currentDateTime();
        QTime current = QTime::currentTime().addSecs(-date.offsetFromUtc());
        mPacketInterface->setMsToday(mId, current.msecsSinceStartOfDay());
    }
}

void CarInterface::on_setClockPiButton_clicked()
{
    if (mPacketInterface) {
        QDateTime date = QDateTime::currentDateTime();
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
            bool res = mPacketInterface->setSystemTime(mId, date.toSecsSinceEpoch(), date.time().msec() * 1000.0);
#else
            bool res = mPacketInterface->setSystemTime(mId, date.toTime_t(), date.time().msec() * 1000.0);
#endif
        if (!res) {
            QMessageBox::warning(this, "Set time on Raspberry Pi",
                                 "Could not set time, no ack received. Make sure that the "
                                 "connection works.");
        }
    }
}

void CarInterface::on_rebootPiButton_clicked()
{
    if (mPacketInterface) {
        bool res = mPacketInterface->sendReboot(mId, false);
        if (!res) {
            QMessageBox::warning(this, "Reboot Raspberry Pi",
                                 "Could not reboot the Raspberry Pi, no ack received. Make sure that the "
                                 "connection works.");
        }
    }
}

void CarInterface::on_shutdownPiButton_clicked()
{
    if (mPacketInterface) {
        bool res = mPacketInterface->sendReboot(mId, true);
        if (!res) {
            QMessageBox::warning(this, "Shutdown Raspberry Pi",
                                 "Could not shut down the Raspberry Pi, no ack received. Make sure that the "
                                 "connection works.");
        }
    }
}

/*
void CarInterface::on_camStartButton_clicked()
{
    mImageByteCnt = 0;
    mImageCnt = 0;
    mImageTimer.restart();

    if (mPacketInterface) {
        ui->pollBox->setChecked(false);
        ui->keyboardControlBox->setChecked(false);
        mPacketInterface->startCameraStream(mId, ui->camCamBox->value(),
                                            ui->camQualityBox->value(),
                                            ui->camWidthBox->value(),
                                            ui->camHeightBox->value(),
                                            ui->camFpsBox->value(),
                                            ui->camSkipBox->value());
    }
}


void CarInterface::on_camStopButton_clicked()
{
    mPacketInterface->startCameraStream(mId, -1, 0, 0, 0, 0, 0);
    ui->camWidget->setPixmap(QPixmap());

    if (mMap) {
        mMap->setLastCameraImage(QImage());
    }
}

void CarInterface::on_camShowMapBox_toggled(bool checked)
{
    if (mMap && !checked) {
        mMap->setLastCameraImage(QImage());
    }
}
*/
void CarInterface::on_ubxVersionButton_clicked()
{
    emit terminalCmd(uint8_t(mId), "ubx_poll UBX_MON_VER");
}

void CarInterface::on_ubxNavSatButton_clicked()
{
    emit terminalCmd(uint8_t(mId), "ubx_poll UBX_NAV_SAT");
}

void CarInterface::on_ubxSolButton_clicked()
{
    emit terminalCmd(uint8_t(mId), "ubx_poll UBX_NAV_SOL");
}

void CarInterface::on_ubxRelPosNedButton_clicked()
{
    emit terminalCmd(uint8_t(mId), "ubx_poll UBX_NAV_RELPOSNED");
}

void CarInterface::on_ubxCfgGnssButton_clicked()
{
    emit terminalCmd(uint8_t(mId), "ubx_poll UBX_CFG_GNSS");
}

void CarInterface::on_zeroGyroButton_clicked()
{
    emit terminalCmd(uint8_t(mId), "zero_gyro");
}

void CarInterface::on_uwbUptimeButton_clicked()
{
    emit terminalCmd(uint8_t(mId), "dw_uptime");
}

void CarInterface::on_uwbResetPosButton_clicked()
{
    emit terminalCmd(uint8_t(mId), "pos_uwb_reset_pos");
}

void CarInterface::on_uwbRebootButton_clicked()
{
    emit terminalCmd(uint8_t(mId), "dw_reboot");
}

void CarInterface::on_uwbListAnchorsButton_clicked()
{
    emit terminalCmd(uint8_t(mId), "pos_uwb_anchors");
}

void CarInterface::on_ioBoardPwmSlider_valueChanged(int value)
{
    double val_mapped = (double)value / 1000.0;
    ui->ioBoardPwmNumber->display(val_mapped);
    emit ioBoardSetPwm(mId, 0, val_mapped);
}

bool CarInterface::getResetApOnEmergencyStop() const
{
    return resetApOnEmergencyStop;
}

void CarInterface::setResetApOnEmergencyStop(bool value)
{
    resetApOnEmergencyStop = value;
}
