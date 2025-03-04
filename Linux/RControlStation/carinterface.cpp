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
    mOrientationWidget = new OrientationWidget(this);
    mCompassWidget = new CompassWidget(this);
    ui->orientationLayout->removeItem(ui->orientationSpacer);
    ui->orientationLayout->insertWidget(0, mCompassWidget, 1);
    ui->orientationLayout->insertWidget(1, mOrientationWidget, 2);
#endif

//    ui->tabWidget->removeTab(6);
//    ui->tabWidget->removeTab(5); //
//    ui->tabWidget->removeTab(4);
//    ui->tabWidget->removeTab(2); //
//    ui->tabWidget->removeTab(0); //

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
    qDebug() << ":::CarInterface::setStateData:::";
    qDebug() << "x (car): " << data.px << ", y (car): " << data.py;
    qDebug() << "x (gps): " << data.px_gps << ", y (gps): " << data.py_gps;
    qDebug() << "x (uwb): " << data.px_uwb << ", " << ", y (uwb): " << data.py_uwb;
    qDebug() << ":::   :::";

    mLastCarState = data;

    ui->imuPlot->addSample(data.accel, data.gyro, data.mag);
    ui->sensorPlot->addSample(data.accel, data.gyro, data.mag);

    // Firmware label
    QString fwStr;
    fwStr.sprintf("FW %d.%d", data.fw_major, data.fw_minor);
    ui->fwLabel->setText(fwStr);
    setFirmwareVersion(qMakePair(data.fw_major, data.fw_minor));

    QString fwStr2;
    fwStr2.sprintf("Ver. %d.%d", data.fw_major, data.fw_minor);
    ui->fwLabel->setText(fwStr2);
/*
    QString fwStrLog1;
    fwStrLog1.sprintf("ZZ %d.%d", data.fw_major, data.fw_minor);
    ui->fwLabel->setText(fwStrLog1);
*/
    // Speed bar
    QString speedTxt;
    speedTxt.sprintf("Speed: %.2f km/h", data.speed * 3.6);
    ui->speedBar->setValue(fabs(data.speed) * 3.6);
    ui->speedBar->setFormat(speedTxt);

    // Temp FET bar
    ui->tempFetBar->setValue(data.temp_fet);

    // Battery bar
//    qDebug() << "V in: " << data.vin;
    double battp = utility::map(data.vin, 34.0, 42.0, 0.0, 100.0);
    if (battp < 0.0) {
        battp = 0.0;
    }
    QString battTxt;
    battTxt.sprintf("Battery: %.1f %% (%.2f V)", battp, data.vin);

    if (battp > 100.0) {
        battp = 100.0;
    }
    ui->batteryBar->setValue((int)battp);
    ui->batteryBar->setFormat(battTxt);

    // Orientation
    setOrientation(data.roll, data.pitch, data.yaw);

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
            msg.sprintf("Car %d: ", mId);
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


        qDebug() << "px (gps): " << data.px_gps << ", py (gps): " << data.py_gps ;
        qDebug() << "px (uwb): " << data.px_uwb << ", py (uwb): " << data.py_uwb ;
        qDebug() << "roll: " << data.roll << ", pitch: " << data.pitch << ", yaw: " << data.yaw;
        qDebug() << "accel. [0]: " << data.accel[0] << ", [1]: " << data.accel[1] << ", [2]: " << data.accel[0];
        qDebug() << "gyro. [0]: " << data.gyro[0] << ", [1]: " << data.gyro[1] << ", [2]: " << data.gyro[0];
        qDebug() << "mag. [0]: " << data.mag[0] << ", [1]: " << data.mag[1] << ", [2]: " << data.mag[0];

        ap_goal.setRadius(data.ap_rad);
        car->setLocation(loc);
        car->setLocationGps(loc_gps);
        car->setLocationUwb(loc_uwb);
        car->setApGoal(ap_goal);
        car->setTime(data.ms_today);
        /*
        //QList<LocPoint> bounds = mMap->getRoute(ui->boundsRouteSpinBox->value());
        QList<LocPoint> bounds = mMap->getRoute(0);
        if (RouteMagic::isPointOutside(loc, bounds))
        {
        	setAp(false, false);
        }
*/

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
    connect(mPacketInterface, SIGNAL(logReceived(quint8,QString)),
            this, SLOT(logPrint(quint8,QString)));
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
    connect(mPacketInterface, SIGNAL(cameraImageReceived(quint8,QImage,int)),
            this, SLOT(cameraImageReceived(quint8,QImage,int)));
    connect(this, SIGNAL(ioBoardSetPwm(quint8,quint8,double)),
            mPacketInterface, SLOT(ioBoardSetPwmDuty(quint8,quint8,double)));
}

void CarInterface::setControlValues(double throttle, double steering, double max, bool currentMode)
{
    if (ui->keyboardControlBox->isChecked()) {
        if (fabs(throttle) < 0.005) {
//            qDebug() << "emit setRcCurrent: 0 :: " << steering;
            emit setRcCurrent(mId, 0.0, steering);
        } else {
            if (currentMode) {
 //               qDebug() << "emit setRcCurrent: " << (throttle * 80.0 * max) << " :: " << steering;
                emit setRcCurrent(mId, throttle * 80.0 * max, steering);
            } else {
 //               qDebug() << "emit setRcDuty: " << (throttle * max) << " :: " << steering;
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
    ui->tabWidget->setCurrentIndex(ui->tabWidget->indexOf(ui->tabSettings));
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

void CarInterface::logPrint(quint8 id, QString str)
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
    if (id == mId) {
        ui->nmeaWidget->inputNmea(nmea_msg);
        if (ui->calibratingCheckBox->isChecked())
        {
//            ui->nmeaTextBrowser->append(QString::fromLocal8Bit(nmea_msg));

            QString nmea_msg_string = QString(nmea_msg);
            QList<QString> elements = nmea_msg_string.split(',');
            bool ok;
            double Xvalue=elements[2].toDouble(&ok);
   //         Xs.append(Xvalue);
            double Yvalue=elements[4].toDouble(&ok);
   //         Ys.append(Yvalue);

//            qDebug() << "X: " << QString("%1").arg(Xvalue,0,'g',13);
//            qDebug() << "Y: " << QString("%1").arg(Yvalue,0,'g',13);
            ui->nmeaTextBrowser->append(QString("%1").arg(Xvalue,0,'g',13)+','+QString("%1").arg(Yvalue,0,'g',13));
        }


        if (mMap) {
            CarInfo *car = mMap->getCarInfo(mId);

            if (car) {
                LocPoint loc_gps = car->getLocationGps();
                loc_gps.setInfo(ui->nmeaWidget->fixType());
//                car->setLocationGps(loc_gps);
            }
        }
    }
}

void CarInterface::configurationReceived(quint8 id, MAIN_CONFIG config)
{
    if (id == mId) {
        mSettingsReadDone = true;
        mConfigLast = config;
        setConfGui(config);

        if (ui->currentCalibrationEdit->text()=="1")
        {
            ui->voltage1Edit->setText(QString("%1").arg(config.car.centrevoltage,0,'g',3));
        };

        if (ui->currentCalibrationEdit->text()=="2")
        {
            ui->voltage2Edit->setText(QString("%1").arg(config.car.centrevoltage,0,'g',3));
        };

        QString str;
        str.sprintf("Car %d: Configuration Received", id);
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
    qDebug() << "In cameraImageReceived!";
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

            if (mMap && ui->camShowMapBox->isChecked()) {
                mMap->setLastCameraImage(image);
            }
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
    conf.car.anglemin = ui->confAngleMinBox->value();
    conf.car.anglemax = ui->confAngleMaxBox->value();
//    conf.car.angledegrees = ui->confAngleDegreesBox->value();
    conf.car.centrevoltage = ui->confCentreVoltageBox->value();

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
    ui->confAngleMinBox->setValue(conf.car.anglemin);
    ui->confAngleMaxBox->setValue(conf.car.anglemax);
  //  ui->confAngleDegreesBox->setValue(conf.car.angledegrees);
    ui->confCentreVoltageBox->setValue(conf.car.centrevoltage);
    ui->confTurnRadBox->setValue(conf.car.axis_distance / tan(conf.car.steering_max_angle_rad));

    ui->confCommonWidget->setConfGui(conf);
}
/*
void CarInterface::on_setClockButton_clicked()
{
    if (mPacketInterface) {
        QDateTime date = QDateTime::currentDateTime();
        QTime current = QTime::currentTime().addSecs(-date.offsetFromUtc());
        mPacketInterface->setMsToday(mId, current.msecsSinceStartOfDay());
    }
}
*/
void CarInterface::on_startCalibration1Button_clicked()
{
    ui->calibratingCheckBox->setChecked(true);

//    ui->pollBox->setChecked(poll);

    if (mPacketInterface) {
        mPacketInterface->getConfiguration(mId);
    }

    ui->currentCalibrationEdit->setText("1");
    on_confReadButton_clicked();
}

void CarInterface::on_startCalibration2Button_clicked()
{
    ui->calibratingCheckBox->setChecked(true);
    ui->currentCalibrationEdit->setText("2");
    on_confReadButton_clicked();
}

void CarInterface::on_endCalibration1Button_clicked()
{
    ui->calibratingCheckBox->setChecked(false);
    ui->currentCalibrationEdit->setText("0");

    std::vector<double> time;
    std::vector<double> velocity;


    QList<QString> lines = ui->nmeaTextBrowser->toPlainText().split("\n");
    int iDatapoints=lines.size();

    qDebug() << "Datapoints: " << QString("%1").arg(lines.size(),0,'g',13);
    qDebug() << "Datapoints: " << ui->nmeaTextBrowser->toPlainText();

    std::vector<double> alphas;

    bool ok;
    int c=0;
    int iWaitSteps=4;
    double oldalpha=0;
    //    while (!file.atEnd()) {
    for (int i=0;i<iDatapoints;i++) {
        qDebug() << c++;
        QString line = lines[i];
        QList<QString> elements = line.split(',');
        double Xvalue=elements[0].toDouble(&ok);
        time.push_back(Xvalue);
        double Yvalue=elements[1].toDouble(&ok);
        velocity.push_back(Yvalue);
        if (c>iWaitSteps)
        {
           double dy_long=velocity[velocity.size()]-velocity[velocity.size()-iWaitSteps];
           double dx_long=time[time.size()]-time[time.size()-iWaitSteps];
           double alpha=atan(dy_long/dx_long);
           double dy=velocity[velocity.size()]-velocity[velocity.size()-1];
           double dx=velocity[velocity.size()]-velocity[velocity.size()-1];
           double stepsize_m=sqrt(dx*dx+dy*dy);
           double dalpha=(alpha-oldalpha)/stepsize_m;
           double dalpha2=std::fmod(dalpha+M_PI,2*M_PI)-M_PI;
           double dalpha3=std::fmod(dalpha2+M_PI/2,M_PI)-M_PI/2;
           alphas.push_back(dalpha3);
           oldalpha=alpha;
        }
    }
    double averageangle = std::accumulate(alphas.begin(), alphas.end(), 0.0) / alphas.size();
    ui->angle1Edit->setText(QString("%1").arg(averageangle,0,'g',13));

    ui->nmeaTextBrowser->clear();
}

void CarInterface::on_endCalibration2Button_clicked()
{
    ui->calibratingCheckBox->setChecked(false);
    ui->currentCalibrationEdit->setText("0");

    std::vector<double> time;
    std::vector<double> velocity;


    QList<QString> lines = ui->nmeaTextBrowser->toPlainText().split("\n");
    int iDatapoints=lines.size();

    qDebug() << "Datapoints: " << QString("%1").arg(lines.size(),0,'g',13);
    qDebug() << "Datapoints: " << ui->nmeaTextBrowser->toPlainText();

    std::vector<double> alphas;

    bool ok;
    int c=0;
    int iWaitSteps=4;
    double oldalpha=0;
    //    while (!file.atEnd()) {
    for (int i=0;i<iDatapoints;i++) {
        qDebug() << c++;
        QString line = lines[i];
        QList<QString> elements = line.split(',');
        double Xvalue=elements[0].toDouble(&ok);
        time.push_back(Xvalue);
        double Yvalue=elements[1].toDouble(&ok);
        velocity.push_back(Yvalue);
        if (c>iWaitSteps)
        {
           double dy_long=velocity[velocity.size()]-velocity[velocity.size()-iWaitSteps];
           double dx_long=time[time.size()]-time[time.size()-iWaitSteps];
           double alpha=atan(dy_long/dx_long);
           double dy=velocity[velocity.size()]-velocity[velocity.size()-1];
           double dx=velocity[velocity.size()]-velocity[velocity.size()-1];
           double stepsize_m=sqrt(dx*dx+dy*dy);
           double dalpha=(alpha-oldalpha)/stepsize_m;
           double dalpha2=std::fmod(dalpha+M_PI,2*M_PI)-M_PI;
           double dalpha3=std::fmod(dalpha2+M_PI/2,M_PI)-M_PI/2;
           alphas.push_back(dalpha3);
           oldalpha=alpha;
           qDebug() << "X: " << QString("%1").arg(Xvalue,0,'g',13);
           qDebug() << "Y: " << QString("%1").arg(Yvalue,0,'g',13);
           qDebug() << "alpha: " << QString("%1").arg(alpha,0,'g',13);
           qDebug() << "dx: " << QString("%1").arg(dx,0,'g',13);
           qDebug() << "dy: " << QString("%1").arg(dy,0,'g',13);
           qDebug() << "stepsize_m: " << QString("%1").arg(stepsize_m,0,'g',13);
           qDebug() << "dalpha: " << QString("%1").arg(dalpha,0,'g',13);
           qDebug() << "dalpha2: " << QString("%1").arg(dalpha2,0,'g',13);
           qDebug() << "dalpha3: " << QString("%1").arg(dalpha3,0,'g',13);
        }
    }
    double averageangle = std::accumulate(alphas.begin(), alphas.end(), 0.0) / alphas.size();
    ui->angle2Edit->setText(QString("%1").arg(averageangle,0,'g',13));
    ui->nmeaTextBrowser->clear();
}

/*
void CarInterface::on_setClockPiButton_clicked()
{
    if (mPacketInterface) {
        QDateTime date = QDateTime::currentDateTime();
        bool res = mPacketInterface->setSystemTime(mId, date.toTime_t(), date.time().msec() * 1000.0);
        if (!res) {
            QMessageBox::warning(this, "Set time on Raspberry Pi",
                                 "Could not set time, no ack received. Make sure that the "
                                 "connection works.");
        }
    }
}
*/
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
        qDebug() << "Start Camera!";
    }
}

void CarInterface::on_camStopButton_clicked()
{
    qDebug() << "Stop Camera!";
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

/*
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
*/
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
