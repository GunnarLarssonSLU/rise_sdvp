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

/*
#include "rtcmclient.h"
#include "rtcm3_simple.h"
#include <QDebug>
#include <QMessageBox>

namespace {
void rtcm_rx(uint8_t *data, int len, int type) {
    if (RtcmClient::currentMsgHandler) {
        QByteArray rtcm_data((const char*)data, len);
        RtcmClient::currentMsgHandler->emitRtcmReceived(rtcm_data, type);
    }
}

void rtcm_rx_1006(rtcm_ref_sta_pos_t *pos) {
    if (RtcmClient::currentMsgHandler) {
        RtcmClient::currentMsgHandler->emitRefPosReceived(
                    pos->lat, pos->lon, pos->height, pos->ant_height);
    }
}
}

// Static member initialization
RtcmClient *RtcmClient::currentMsgHandler = 0;
bool RtcmClient::gpsOnly = false;
rtcm3_state RtcmClient::rtcmState;

RtcmClient::RtcmClient(QObject *parent) : QObject(parent)
{
    mTcpSocket = new QTcpSocket(this);
    mSerialPort = new QSerialPort(this);

    qRegisterMetaType<rtcm_obs_header_t>("rtcm_obs_header_t");
    qRegisterMetaType<rtcm_obs_t>("rtcm_obs_gps_t");

    currentMsgHandler = this;
    rtcm3_init_state(&rtcmState);
    rtcm3_set_rx_callback(rtcm_rx, &rtcmState);
    rtcm3_set_rx_callback_1005_1006(rtcm_rx_1006, &rtcmState);

    connect(mTcpSocket, SIGNAL(readyRead()), this, SLOT(tcpInputDataAvailable()));
    connect(mTcpSocket, SIGNAL(connected()), this, SLOT(tcpInputConnected()));
    connect(mTcpSocket, SIGNAL(disconnected()),
            this, SLOT(tcpInputDisconnected()));
    connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(tcpInputError(QAbstractSocket::SocketError)));
    connect(mSerialPort, SIGNAL(readyRead()), this, SLOT(serialDataAvailable()));
    connect(mSerialPort, SIGNAL(error(QSerialPort::SerialPortError)),
            this, SLOT(serialPortError(QSerialPort::SerialPortError)));
}

bool RtcmClient::connectNtrip(QString server, QString stream, QString user, QString pass, int port)
{
    mNtripUser = user;
    mNtripPassword = pass;
    mNtripStream = stream;
    mNtripServer = server;

    mTcpSocket->abort();
    mTcpSocket->connectToHost(server, port);

    return true;
}

bool RtcmClient::connectTcp(QString server, int port)
{
    mNtripUser = "";
    mNtripPassword = "";
    mNtripStream = "";
    mNtripServer = "";

    mTcpSocket->abort();
    mTcpSocket->connectToHost(server, port);

    return true;
}

bool RtcmClient::connectSerial(QString port, int baudrate)
{
    if(mSerialPort->isOpen()) {
        mSerialPort->close();
    }

    mSerialPort->setPortName(port);
    mSerialPort->open(QIODevice::ReadWrite);

    if(!mSerialPort->isOpen()) {
        return false;
    }

    mSerialPort->setBaudRate(baudrate);
    mSerialPort->setDataBits(QSerialPort::Data8);
    mSerialPort->setParity(QSerialPort::NoParity);
    mSerialPort->setStopBits(QSerialPort::OneStop);
    mSerialPort->setFlowControl(QSerialPort::NoFlowControl);

    return true;
}

bool RtcmClient::isTcpConnected()
{
    // If no stream is selected we have simply connected to a TCP server.
    return mTcpSocket->isOpen();
}

bool RtcmClient::isSerialConnected()
{
    return mSerialPort->isOpen();
}

void RtcmClient::disconnectTcpNtrip()
{
    mTcpSocket->close();
}

void RtcmClient::disconnectSerial()
{
    mSerialPort->close();
}

void RtcmClient::setGpsOnly(bool isGpsOnly)
{
    gpsOnly = isGpsOnly;
}

void RtcmClient::emitRtcmReceived(QByteArray data, int type, bool sync)
{
    emit rtcmReceived(data, type, sync);
}

void RtcmClient::emitRefPosReceived(double lat, double lon, double height, double antenna_height)
{
    emit refPosReceived(lat, lon, height, antenna_height);
}

QByteArray RtcmClient::encodeBasePos(double lat, double lon, double height, double antenna_height)
{
    rtcm_ref_sta_pos_t pos;
    int len;

    pos.staid = 0;
    pos.lat = lat;
    pos.lon = lon;
    pos.height = height;
    pos.ant_height = antenna_height;

    quint8 buffer[40];
    rtcm3_encode_1006(pos, buffer, &len);

    QByteArray rtcm_data((const char*)buffer, len);
    return rtcm_data;
}

void RtcmClient::tcpInputConnected()
{
    qDebug() << "RTCM TCP connected";

    // If no stream is selected we have simply connected to a TCP server.
    if (mNtripStream.size() > 0) {
        QString msg;
        msg += "GET /" + mNtripStream + " HTTP/1.1\r\n";
        msg += "User-Agent: NTRIP " + mNtripServer + "\r\n";

        if (mNtripUser.size() > 0 || mNtripPassword.size() > 0) {
            QString authStr = mNtripUser + ":" + mNtripPassword;
            QByteArray auth;
            auth.append(authStr);
            msg += "Authorization: Basic " + auth.toBase64() + "\r\n";
        }

        msg += "Accept: **\r\nConnection: close\r\n";
        msg += "\r\n";

        mTcpSocket->write(msg.toLocal8Bit());
    }
}

void RtcmClient::tcpInputDisconnected()
{
    qDebug() << "RTCM TCP disconnected";
}

void RtcmClient::tcpInputDataAvailable()
{
    QByteArray data =  mTcpSocket->readAll();

    for (int i = 0;i < data.size();i++) {
        int ret = rtcm3_input_data(data.at(i), &rtcmState);
        if (ret == -1 || ret == -2) {
            //qWarning() << "RTCM decode error:" <<  ret;
        }
    }
}

void RtcmClient::tcpInputError(QAbstractSocket::SocketError socketError)
{
    (void)socketError;

    QString errorStr = mTcpSocket->errorString();
    qWarning() << "RTCM TCP Error:" << errorStr;
    QMessageBox::warning(0, "RTCM TCP Error", errorStr);

    mTcpSocket->close();
}

void RtcmClient::serialDataAvailable()
{
    while (mSerialPort->bytesAvailable() > 0) {
        QByteArray data = mSerialPort->readAll();

        for (int i = 0;i < data.size();i++) {
            int ret = rtcm3_input_data(data.at(i), &rtcmState);
            if (ret == -1 || ret == -2) {
                //qWarning() << "RTCM decode error:" <<  ret;
            }
        }
    }
}

void RtcmClient::serialPortError(QSerialPort::SerialPortError error)
{
    QString message;
    switch (error) {
    case QSerialPort::NoError:
        break;
    case QSerialPort::DeviceNotFoundError:
        message = tr("Device not found");
        break;
    case QSerialPort::OpenError:
        message = tr("Can't open device");
        break;
    case QSerialPort::NotOpenError:
        message = tr("Not open error");
        break;
    case QSerialPort::ResourceError:
        message = tr("Port disconnected");
        break;
    case QSerialPort::PermissionError:
        message = tr("Permission error");
        break;
    case QSerialPort::UnknownError:
        message = tr("Unknown error");
        break;
    default:
        message = "Error number: " + QString::number(error);
        break;
    }

    if(!message.isEmpty()) {
        qDebug() << "Serial error:" << message;

        if(mSerialPort->isOpen()) {
            mSerialPort->close();
        }
    }
}
*/

// The code below is from WayWise (https://raw.githubusercontent.com/RISE-Dependable-Transport-Systems/WayWise/8825fe423dcac2d0ba899ff6f6e69f92160b1650/sensors/gnss/rtcmclient.cpp)

/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 */

#include "rtcmclient.h"
#include "rtcm3_simple.h"
#include <QFile>

RtcmClient::RtcmClient(QObject *parent) : QObject(parent)
{
    qDebug() << "in RtcmClient constructor";
    connect(&mTcpSocket, &QTcpSocket::readyRead, [this]{
        QByteArray data =  mTcpSocket.readAll();
        //        qDebug() << data;

        // Make sure to skip "ICY 200 OK" when connection to NTRIP
        if (!mSkippedFirstReply) {
            mSkippedFirstReply = true;
            return;
        }

        // Try to read 1005 or 1006 from stream to get base station position
        // See RTKLIB for how RTCM is decoded (https://github.com/tomojitakasu/RTKLIB)
        // We are not CRC checking here (getting data via TCP anyways)
        // TODO: wait for rest of incomplete message?
        if (!mFoundReferenceStationInfo) {
            const char* dataPtr = data.constData();
            while ((dataPtr = std::find(dataPtr, data.constEnd(), RTCM3_PREAMBLE)) != data.constEnd()) {
                if (dataPtr + 5 < data.constEnd()) {
                    int length = getbitu(dataPtr, 14, 10) + 1; // number of bytes inkl. crc
                    int type = getbitu(dataPtr, 24, 12);
                    if (dataPtr + length < data.constEnd() && (type == 1005 || type == 1006)) {
                        llh_t baseLlh = decodeLllhFromReferenceStationInfo(data.mid(dataPtr - data.constData(), length));
                        //                        qDebug() << baseLlh.latitude << baseLlh.longitude << baseLlh.height << dataPtr - data.data() << length;
                        qDebug() << "RtcmClient got base station position:" << baseLlh.latitude << baseLlh.longitude << baseLlh.height;
                        mFoundReferenceStationInfo = true;
                        emit baseStationPosition(baseLlh);
                    }
                }
                dataPtr++;
            }
        }
        emit rtcmData(data);
    });

    connect(&mTcpSocket, &QTcpSocket::connected, [this]{
        // If a stream is selected, we connected to an NTRIP server and potentially need to authenticate.
        if (mCurrentNtripConnectionInfo.stream.size() > 0) {
            QString msg;
            msg += "GET /" + mCurrentNtripConnectionInfo.stream + " HTTP/1.1\r\n";
            msg += "User-Agent: NTRIP " + mCurrentHost + "\r\n";

            if (mCurrentNtripConnectionInfo.user.size() > 0 || mCurrentNtripConnectionInfo.password.size() > 0) {
                QString authStr = mCurrentNtripConnectionInfo.user + ":" + mCurrentNtripConnectionInfo.password;
                QByteArray auth;
                auth.append(authStr.toLocal8Bit());
                msg += "Authorization: Basic " + auth.toBase64() + "\r\n";
            }

            msg += "Accept: */*\r\nConnection: close\r\n";
            msg += "\r\n";

            mTcpSocket.write(msg.toLocal8Bit());
        }
    });
}

void RtcmClient::connectTcp(QString host, qint16 port)
{
    mCurrentHost = host;
    mCurrentPort = port;
    mCurrentNtripConnectionInfo = {"", "", ""};
    mTcpSocket.connectToHost(mCurrentHost, mCurrentPort);
}

void RtcmClient::connectNtrip(QString host, qint16 port, NtripConnectionInfo ntripConnectionInfo)
{
    mCurrentHost = host;
    mCurrentPort = port;
    mCurrentNtripConnectionInfo = ntripConnectionInfo;
    mTcpSocket.connectToHost(mCurrentHost, mCurrentPort);
}

bool RtcmClient::connectWithInfoFromFile(QString filePath)
{
    QFile rtcmServerInfoFile(filePath);
    if (!rtcmServerInfoFile.open(QIODevice::ReadOnly)) {
        qDebug() << "Warning: RtcmClient was unable to open" << filePath;
        return false;
    } else {
        QString serverName = QString(rtcmServerInfoFile.readLine()).trimmed();
        qint16 port = QString(rtcmServerInfoFile.readLine()).trimmed().toShort();
        NtripConnectionInfo ntripConnectionInfo = {QString(rtcmServerInfoFile.readLine()).trimmed(),  // username
                                                   QString(rtcmServerInfoFile.readLine()).trimmed(),  // password
                                                   QString(rtcmServerInfoFile.readLine()).trimmed()}; // stream
        //        qDebug() << serverName << port << ntripConnectionInfo.user << ntripConnectionInfo.password << ntripConnectionInfo.stream;

        connectNtrip(serverName, port, ntripConnectionInfo);
    }

    return isConnected();
}

bool RtcmClient::isConnected()
{
    return mTcpSocket.isOpen() && mTcpSocket.isReadable();
}

void RtcmClient::disconnect()
{
    if (isConnected())
        mTcpSocket.disconnectFromHost();
}

QString RtcmClient::getCurrentHost() const
{
    return mCurrentHost;
}

qint16 RtcmClient::getCurrentPort() const
{
    return mCurrentPort;
}

void RtcmClient::forwardNmeaGgaToServer(const QByteArray &nmeaGgaStr)
{
    // Send NMEA GGA to NTRIP/RTCM server until we got reference station information.
    // Some NTRIP servers will not start sending RTCM unless they got NMEA GGA.
    if (isConnected() && mSkippedFirstReply)
        mTcpSocket.write(nmeaGgaStr);
}

unsigned int RtcmClient::getbitu(const char *buff, int pos, int len) {
    unsigned int bits=0;
    int i;

    for (i = pos;i < pos + len;i++) {
        bits = (bits << 1) + ((buff[i / 8] >> (7 - i % 8)) & 1u);
    }

    return bits;
}

int RtcmClient::getbits(const char *buff, int pos, int len) {
    unsigned int bits = getbitu(buff, pos, len);

    if (len <= 0 || 32 <= len || !(bits & (1u << (len - 1)))) {
        return (int)bits;
    }

    return (int)(bits | (~0u << len)); // extend sign
}

double RtcmClient::getbits_38(const char *buff, int pos) {
    return (double)getbits(buff, pos, 32) * D(64.0) + getbitu(buff, pos + 32, 6);
}

llh_t RtcmClient::decodeLllhFromReferenceStationInfo(const QByteArray data)
{
    double p0 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    int bitIdx = 24 + 12;
    int staid; Q_UNUSED(staid)
    int itrf; Q_UNUSED(itrf)
    llh_t llhResult = {0.0, 0.0, 0.0};

    //    if (bitIdx + 140 <= data.size() * 8) {
    staid = getbitu(data.constData(), bitIdx, 12); bitIdx+=12;
    itrf  = getbitu(data.constData(), bitIdx, 6);  bitIdx+= 6+4;
    p0    = getbits_38(data.constData(), bitIdx);  bitIdx+=38+2;
    p1    = getbits_38(data.constData(), bitIdx);  bitIdx+=38+2;
    p2    = getbits_38(data.constData(), bitIdx);


    p0 *= D(0.0001);
    p1 *= D(0.0001);
    p2 *= D(0.0001);


    // Convert ecef to llh
    double e2 = FE_WGS84 * (D(2.0) - FE_WGS84);
    double r2 = p0 * p0 + p1 * p1;
    double z = p2;
    double zk = 0.0;
    double sinp = 0.0;
    double v = RE_WGS84;

    while (fabs(z - zk) >= D(1E-4)) {
        zk = z;
        sinp = z / sqrt(r2 + z * z);
        v = RE_WGS84 / sqrt(D(1.0) - e2 * sinp * sinp);
        z = p2 + v * e2 * sinp;
    }

    llhResult.latitude = (r2 > D(1E-12) ? atan(z / sqrt(r2)) : (p2 > D(0.0) ? D_PI / D(2.0) : -D_PI / D(2.0))) * D(180.0) / D_PI;
    llhResult.longitude = (r2 > D(1E-12) ? atan2(p1, p0) : D(0.0)) * D(180.0) / D_PI;
    llhResult.height = sqrt(r2 + z * z) - v;
    //    }

    return llhResult;
}

QByteArray RtcmClient::encodeBasePos(double lat, double lon, double height, double antenna_height)
{
    rtcm_ref_sta_pos_t pos;
    int len;

    pos.staid = 0;
    pos.lat = lat;
    pos.lon = lon;
    pos.height = height;
    pos.ant_height = antenna_height;

    quint8 buffer[40];
    rtcm3_encode_1006(pos, buffer, &len);

    QByteArray rtcm_data((const char*)buffer, len);
    return rtcm_data;
}

bool RtcmClient::gpsOnly = false;

void RtcmClient::setGpsOnly(bool isGpsOnly)
{
    gpsOnly = isGpsOnly;
}
