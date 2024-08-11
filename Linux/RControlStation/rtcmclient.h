/*
    Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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
#ifndef RTCMCLIENT_H
#define RTCMCLIENT_H

#include <QObject>
#include <QTcpSocket>
#include <QSerialPort>
#include "datatypes.h"

class RtcmClient : public QObject
{
    Q_OBJECT
public:
    static RtcmClient* currentMsgHandler;
    static bool gpsOnly;
    static rtcm3_state rtcmState;

    explicit RtcmClient(QObject *parent = 0);
    bool connectNtrip(QString server, QString stream, QString user = "", QString pass = "", int port = 80);
    bool connectTcp(QString server, int port = 80);
    bool connectSerial(QString port, int baudrate = 115200);
    bool isTcpConnected();
    bool isSerialConnected();
    void disconnectTcpNtrip();
    void disconnectSerial();
    void setGpsOnly(bool isGpsOnly);

    void emitRtcmReceived(QByteArray data, int type, bool sync = false);
    void emitRefPosReceived(double lat, double lon, double height, double antenna_height);

    static QByteArray encodeBasePos(double lat, double lon, double height, double antenna_height = 0);

signals:
    void rtcmReceived(QByteArray data, int type, bool sync = false);
    void refPosReceived(double lat, double lon, double height, double antenna_height);

private slots:
    void tcpInputConnected();
    void tcpInputDisconnected();
    void tcpInputDataAvailable();
    void tcpInputError(QAbstractSocket::SocketError socketError);
    void serialDataAvailable();
    void serialPortError(QSerialPort::SerialPortError error);

private:
    QString mNtripUser;
    QString mNtripPassword;
    QString mNtripServer;
    QString mNtripStream;
    QTcpSocket *mTcpSocket;
    QSerialPort *mSerialPort;

};

#endif // RTCMCLIENT_H
*/

/*
 *     Copyright 2022 Marvin Damschen   marvin.damschen@ri.se
 *     Published under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
 *
 * Connects to NTRIP / RTCM server (TCP/IP) and streams received messages using singal/slot
 * Some rudimentary parsing of messages.
 */

#ifndef RTCMCLIENT_H
#define RTCMCLIENT_H

#include <QObject>
#include <QTcpSocket>
#include <QHostAddress>
#include "coordinatetransform.h"
#include "datatypes.h"

#ifndef D
#define D(x) 						((double)x##L)
#endif

#ifndef D_PI
#define D_PI						D(3.14159265358979323846)
#endif

struct NtripConnectionInfo {
    QString user;
    QString password;
    QString stream;
};

class RtcmClient : public QObject
{
    Q_OBJECT
public:
    explicit RtcmClient(QObject *parent = nullptr);
    void connectTcp(QString host, qint16 port);
    void connectNtrip(QString host, qint16 port, NtripConnectionInfo ntripInfo);
    bool connectWithInfoFromFile(QString filePath);
    bool isConnected();
    void disconnect();
    static llh_t decodeLllhFromReferenceStationInfo(const QByteArray data);
    QString getCurrentHost() const;
    qint16 getCurrentPort() const;

    void forwardNmeaGgaToServer(const QByteArray& nmeaGgaStr);

    // Put back
    static bool gpsOnly;
    static QByteArray encodeBasePos(double lat, double lon, double height, double antenna_height = 0);
    void setGpsOnly(bool isGpsOnly);

signals:
    void rtcmData(const QByteArray &data);
    void baseStationPosition(const llh_t &baseStationPosition);

private:
    QTcpSocket mTcpSocket;
    QString mCurrentHost;
    qint16 mCurrentPort;
    NtripConnectionInfo mCurrentNtripConnectionInfo;
    bool mFoundReferenceStationInfo = false;
    bool mSkippedFirstReply = false;

    // For parsing RTCMv3 (from RTKLIB)
    const char RTCM3_PREAMBLE = char(0xD3);
    static unsigned int getbitu(const char *buff, int pos, int len);
    static int getbits(const char *buff, int pos, int len);
    static double getbits_38(const char *buff, int pos);

};

#endif // RTCMCLIENT_H
