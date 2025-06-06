/*
    Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#include "rtcmwidget.h"
#include "ui_rtcmwidget.h"
#include <QSerialPortInfo>
#include <QMessageBox>
#include "utility.h"

RtcmWidget::RtcmWidget(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::RtcmWidget)
{
    ui->setupUi(this);
    mRtcm = new RtcmClient(this);
    mTimer = new QTimer(this);
    mTimer->start(20);
    mTcpServer = new TcpBroadcast(this);

    connect(mRtcm, SIGNAL(rtcmReceived(QByteArray,int,bool)),
            this, SLOT(rtcmRx(QByteArray,int,bool)));
    connect(mRtcm, SIGNAL(refPosReceived(double,double,double,double)),
            this, SLOT(refPosRx(double,double,double,double)));
    connect(mTimer, SIGNAL(timeout()),
            this, SLOT(timerSlot()));

    on_rtcmSerialRefreshButton_clicked();
    on_ntripBox_toggled(ui->ntripBox->isChecked());
    on_gpsOnlyBox_toggled(ui->gpsOnlyBox->isChecked());

    // SPT00 default
    ui->refSendLatBox->setValue(57.71495867);
    ui->refSendLonBox->setValue(12.89134921);
    ui->refSendHBox->setValue(219.0);

    // Onsala default
//    double lon, lat, h;
//    utility::xyzToLlh(3370667.1982, 711818.7226, 5349787.8784, &lat, &lon, &h);
//    ui->refSendLatBox->setValue(lat);
//    ui->refSendLonBox->setValue(lon);
//    ui->refSendHBox->setValue(h);
//    qDebug() << fixed << qSetRealNumberPrecision(8) << lat << lon << h;

    // Home
//    ui->refSendLatBox->setValue(57.57848470);
//    ui->refSendLonBox->setValue(13.11463205);
//    ui->refSendHBox->setValue(204.626);
}

RtcmWidget::~RtcmWidget()
{
    delete ui;
}

void RtcmWidget::setRefPos(double lat, double lon, double height, double antenna_height)
{
    ui->refSendLatBox->setValue(lat);
    ui->refSendLonBox->setValue(lon);
    ui->refSendHBox->setValue(height);
    ui->refSendAntHBox->setValue(antenna_height);
}

void RtcmWidget::timerSlot()
{
    // Update ntrip connected label
    static bool wasNtripConnected = false;
    if (wasNtripConnected != mRtcm->isTcpConnected()) {
        wasNtripConnected = mRtcm->isTcpConnected();

        if (wasNtripConnected) {
            ui->ntripConnectedLabel->setText("Connected");
        } else {
            ui->ntripConnectedLabel->setText("Not connected");
        }
    }

    // Update serial connected label
    static bool wasSerialConnected = false;
    if (wasSerialConnected != mRtcm->isSerialConnected()) {
        wasSerialConnected = mRtcm->isSerialConnected();

        if (wasSerialConnected) {
            ui->rtcmSerialConnectedLabel->setText("Connected");
        } else {
            ui->rtcmSerialConnectedLabel->setText("Not connected");
        }
    }

    // Send reference position every 5s
    if (ui->sendRefPosBox->isChecked()) {
        static int cnt = 0;
        cnt++;
        if (cnt >= (5000 / mTimer->interval())) {
            cnt = 0;
            QByteArray data = RtcmClient::encodeBasePos(
                        ui->refSendLatBox->value(),
                        ui->refSendLonBox->value(),
                        ui->refSendHBox->value(),
                        ui->refSendAntHBox->value());

            emit rtcmReceived(data);
            mTcpServer->broadcastData(data);
        }
    }
}

void RtcmWidget::rtcmRx(QByteArray data, int type, bool sync)
{
    (void)sync;

    switch (type) {
    case 1001: ui->rtcm1001Number->display(ui->rtcm1001Number->value() + 1); break;
    case 1002: ui->rtcm1002Number->display(ui->rtcm1002Number->value() + 1); break;
    case 1003: ui->rtcm1003Number->display(ui->rtcm1003Number->value() + 1); break;
    case 1004: ui->rtcm1004Number->display(ui->rtcm1004Number->value() + 1); break;
    case 1074: ui->rtcm1074Number->display(ui->rtcm1074Number->value() + 1); break;
    case 1075: ui->rtcm1075Number->display(ui->rtcm1075Number->value() + 1); break;
    case 1077: ui->rtcm1077Number->display(ui->rtcm1077Number->value() + 1); break;

    case 1009: ui->rtcm1009Number->display(ui->rtcm1009Number->value() + 1); break;
    case 1010: ui->rtcm1010Number->display(ui->rtcm1010Number->value() + 1); break;
    case 1011: ui->rtcm1011Number->display(ui->rtcm1011Number->value() + 1); break;
    case 1012: ui->rtcm1012Number->display(ui->rtcm1012Number->value() + 1); break;
    case 1084: ui->rtcm1084Number->display(ui->rtcm1084Number->value() + 1); break;
    case 1085: ui->rtcm1085Number->display(ui->rtcm1085Number->value() + 1); break;
    case 1087: ui->rtcm1087Number->display(ui->rtcm1087Number->value() + 1); break;
    case 1230: ui->rtcm1230Number->display(ui->rtcm1230Number->value() + 1); break;

    case 1094: ui->rtcm1094Number->display(ui->rtcm1094Number->value() + 1); break;
    case 1095: ui->rtcm1095Number->display(ui->rtcm1095Number->value() + 1); break;
    case 1097: ui->rtcm1097Number->display(ui->rtcm1097Number->value() + 1); break;

    case 1115: ui->rtcm1115Number->display(ui->rtcm1115Number->value() + 1); break;

    case 1124: ui->rtcm1124Number->display(ui->rtcm1124Number->value() + 1); break;
    case 1125: ui->rtcm1125Number->display(ui->rtcm1125Number->value() + 1); break;
    case 1127: ui->rtcm1127Number->display(ui->rtcm1127Number->value() + 1); break;

    case 1005: ui->rtcm1005Number->display(ui->rtcm1005Number->value() + 1); break;
    case 1006: ui->rtcm1006Number->display(ui->rtcm1006Number->value() + 1); break;

    case 1019: ui->rtcm1019Number->display(ui->rtcm1019Number->value() + 1); break;
    case 1020: ui->rtcm1020Number->display(ui->rtcm1020Number->value() + 1); break;
    default:
        break;
    }

    emit rtcmReceived(data);
}

void RtcmWidget::refPosRx(double lat, double lon, double height, double antenna_height)
{
    QString str;
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
    str = QString("Lat:            %1\n"
                  "Lon:            %2\n"
                  "Height:         %3\n"
                  "Antenna Height: %4")
              .arg(lat,0,'f',8)
              .arg(lon,0,'f',8)
              .arg(height,0,'f',3)
              .arg(antenna_height,0,'f',3);
#else
    str.sprintf("Lat:            %.8f\n"
                "Lon:            %.8f\n"
                "Height:         %.3f\n"
                "Antenna Height: %.3f",
                lat, lon, height, antenna_height);
#endif
    ui->lastRefPosLablel->setText(str);
}

void RtcmWidget::on_ntripConnectButton_clicked()
{
    if (ui->ntripBox->isChecked()) {
        mRtcm->connectNtrip(ui->ntripServerEdit->text(),
                            ui->ntripStreamEdit->text(),
                            ui->ntripUserEdit->text(),
                            ui->ntripPasswordEdit->text(),
                            ui->ntripPortBox->value());
    } else {
        mRtcm->connectTcp(ui->ntripServerEdit->text(), ui->ntripPortBox->value());
    }
}

void RtcmWidget::on_ntripDisconnectButton_clicked()
{
    mRtcm->disconnectTcpNtrip();
}

void RtcmWidget::on_resetAllCountersButton_clicked()
{
    ui->rtcm1001Number->display(0);
    ui->rtcm1002Number->display(0);
    ui->rtcm1003Number->display(0);
    ui->rtcm1004Number->display(0);
    ui->rtcm1074Number->display(0);
    ui->rtcm1075Number->display(0);
    ui->rtcm1077Number->display(0);

    ui->rtcm1009Number->display(0);
    ui->rtcm1010Number->display(0);
    ui->rtcm1011Number->display(0);
    ui->rtcm1012Number->display(0);
    ui->rtcm1084Number->display(0);
    ui->rtcm1085Number->display(0);
    ui->rtcm1087Number->display(0);
    ui->rtcm1230Number->display(0);

    ui->rtcm1094Number->display(0);
    ui->rtcm1095Number->display(0);
    ui->rtcm1097Number->display(0);

    ui->rtcm1115Number->display(0);

    ui->rtcm1124Number->display(0);
    ui->rtcm1125Number->display(0);
    ui->rtcm1127Number->display(0);

    ui->rtcm1005Number->display(0);
    ui->rtcm1006Number->display(0);

    ui->rtcm1019Number->display(0);
    ui->rtcm1020Number->display(0);
}

void RtcmWidget::on_ntripBox_toggled(bool checked)
{
    if (checked) {
        ui->ntripServerEdit->setText("www.igs-ip.net");
        ui->ntripPortBox->setValue(80);

        ui->ntripUserEdit->setEnabled(true);
        ui->ntripPasswordEdit->setEnabled(true);
        ui->ntripStreamEdit->setEnabled(true);
    } else {
        ui->ntripServerEdit->setText("192.168.200.1");
//        ui->ntripServerEdit->setText("localhost");
        ui->ntripPortBox->setValue(2101);
//        ui->ntripPortBox->setValue(65300);

        ui->ntripUserEdit->setEnabled(false);
        ui->ntripPasswordEdit->setEnabled(false);
        ui->ntripStreamEdit->setEnabled(false);
    }
}

void RtcmWidget::on_rtcmSerialRefreshButton_clicked()
{
    ui->rtcmSerialPortBox->clear();

    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    foreach(const QSerialPortInfo &port, ports) {
        ui->rtcmSerialPortBox->addItem(port.portName(), port.systemLocation());
    }

    ui->rtcmSerialPortBox->setCurrentIndex(0);
}

void RtcmWidget::on_rtcmSerialDisconnectButton_clicked()
{
    mRtcm->disconnectSerial();
}

void RtcmWidget::on_rtcmSerialConnectButton_clicked()
{
    mRtcm->connectSerial(ui->rtcmSerialPortBox->currentData().toString(),
                         ui->rtcmSerialBaudBox->value());
}

void RtcmWidget::on_refGetButton_clicked()
{
    emit refPosGet();
}

void RtcmWidget::on_tcpServerBox_toggled(bool checked)
{
    if (checked) {
        if (!mTcpServer->startTcpServer(ui->tcpServerPortBox->value())) {
            QMessageBox::warning(this, "TCP Server Error",
                                 "Creating TCP server for RTCM data failed. Make sure that the port is not "
                                 "already in use.");
            ui->tcpServerBox->setChecked(false);
        }
    } else {
        mTcpServer->stopServer();
    }
}

void RtcmWidget::on_gpsOnlyBox_toggled(bool checked)
{
    mRtcm->setGpsOnly(checked);
}
