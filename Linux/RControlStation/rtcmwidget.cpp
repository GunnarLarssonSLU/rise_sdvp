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

    //on_rtcmSerialRefreshButton_clicked();

    on_gpsOnlyBox_toggled(ui->gpsOnlyBox->isChecked());

    // SPT00 default
//    ui->refSendLatBox->setValue(59.81);
//    ui->refSendLonBox->setValue(17.658);
//    ui->refSendHBox->setValue(0);

//    on_ntripConnectButton_clicked();
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
/*
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
*/
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
    qDebug() << "Type: " << type << ", data:" << data;
    emit rtcmReceived(data);
}

void RtcmWidget::refPosRx(double lat, double lon, double height, double antenna_height)
{
    QString str;
    str.sprintf("Lat:            %.8f\n"
                "Lon:            %.8f\n"
                "Height:         %.3f\n"
                "Antenna Height: %.3f",
                lat, lon, height, antenna_height);
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

/*
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
*/

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
