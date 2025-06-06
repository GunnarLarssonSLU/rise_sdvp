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

#include "carclient.h"
#include <QDebug>
#include <QDateTime>
#include <QDir>
#include <sys/time.h>
#include <sys/reboot.h>
#include <unistd.h>
#include <QEventLoop>
#include <QCoreApplication>
#include <QNetworkInterface>
#include <QBuffer>
#include <QLocalSocket>
#include "rtcm3_simple.h"
#include "utility.h"
#include "datatypes.h"
#include <cstring>

#include <QCoreApplication>
#include <QProcess>
#include <QDebug>
#include <thread>
#include <chrono>

namespace {
void rtcm_rx(uint8_t *data, int len, int type) {
    if (CarClient::currentMsgHandler) {
        QByteArray rtcm_data((const char*)data, len);
        CarClient::currentMsgHandler->rtcmRx(rtcm_data, type);
    }
}
}

// Static member initialization
CarClient *CarClient::currentMsgHandler = 0;
rtcm3_state CarClient::rtcmState;

CarClient::CarClient(QObject *parent) : QObject(parent)
{
    mSettings.nmeaConnect = false;
    mSettings.serialConnect = false;
    mSettings.serialRtcmConnect = false;

    mSerialPort = new SerialPort(this);
    mSerialPortRtcm = new QSerialPort(this);
    mSerialPortArduino = new QSerialPort(this);
    mPacketInterface = new PacketInterface(this);
    mRtcmBroadcaster = new TcpBroadcast(this);
    mUbxBroadcaster = new TcpBroadcast(this);
    mLogBroadcaster = new TcpBroadcast(this);
    mUblox = new Ublox(this);
    mTcpSocket = new QTcpSocket(this);
    mTcpServer = new TcpServerSimple(this);

    mRtcmClient = new RtcmClient(this);
    mCarId = 255;
    mReconnectTimer = new QTimer(this);
    mReconnectTimer->start(2000);
    mTcpConnected = false;
    mLogFlushTimer = new QTimer(this);
    mLogFlushTimer->start(2000);
    mRtklibRunning = false;
    mBatteryCells = 10;
    mCarIdToSet = -1;
    mOverrideUwbPos = false;
    mOverrideUwbX = 0.0;
    mOverrideUwbY = 0.0;

    mHostAddress = QHostAddress("0.0.0.0");
    mUdpPort = 0;
    mUdpSocket = new QUdpSocket(this);

    mRtcmBaseLat = 0.0;
    mRtcmBaseLon = 0.0;
    mRtcmBaseHeight = 0.0;
    mRtcmSendBase = false;

    currentMsgHandler = this;
    rtcm3_init_state(&rtcmState);
    rtcm3_set_rx_callback(rtcm_rx, &rtcmState);

    mTcpServer->setUsePacket(true);

    connect(mSerialPort, SIGNAL(serial_data_available()),
            this, SLOT(serialDataAvailable()));
    connect(mSerialPort, SIGNAL(serial_port_error(int)),
            this, SLOT(serialPortError(int)));
    connect(mSerialPortRtcm, SIGNAL(readyRead()),
            this, SLOT(serialRtcmDataAvailable()));
    connect(mSerialPortArduino, SIGNAL(readyRead()),
            this, SLOT(serialArduinoDataAvailable()));
    connect(mTcpSocket, SIGNAL(readyRead()), this, SLOT(tcpDataAvailable()));
    connect(mTcpSocket, SIGNAL(connected()), this, SLOT(tcpConnected()));
    connect(mTcpSocket, SIGNAL(disconnected()), this, SLOT(tcpDisconnected()));
    connect(mPacketInterface, SIGNAL(rtcmUsbReceived(quint8,QByteArray)),
            this, SLOT(rtcmUsbRx(quint8,QByteArray)));
    connect(mPacketInterface, SIGNAL(dataToSend(QByteArray&)),
            this, SLOT(packetDataToSend(QByteArray&)));
    connect(mReconnectTimer, SIGNAL(timeout()),
            this, SLOT(reconnectTimerSlot()));
    connect(mUdpSocket, SIGNAL(readyRead()),
            this, SLOT(readPendingDatagrams()));
    connect(mPacketInterface, SIGNAL(packetReceived(quint8,CMD_PACKET,QByteArray)),
            this, SLOT(carPacketRx(quint8,CMD_PACKET,QByteArray)));
    connect(mPacketInterface, SIGNAL(logLineUsbReceived(quint8,QString)),
            this, SLOT(logLineUsbReceived(quint8,QString)));
    connect(mLogFlushTimer, SIGNAL(timeout()),
            this, SLOT(logFlushTimerSlot()));
    connect(mPacketInterface, SIGNAL(systemTimeReceived(quint8,qint32,qint32)),
            this, SLOT(systemTimeReceived(quint8,qint32,qint32)));
    connect(mPacketInterface, SIGNAL(rebootSystemReceived(quint8,bool)),
            this, SLOT(rebootSystemReceived(quint8,bool)));
    connect(mUblox, SIGNAL(ubxRx(QByteArray)), this, SLOT(ubxRx(QByteArray)));
    connect(mUblox, SIGNAL(rxRawx(ubx_rxm_rawx)), this, SLOT(rxRawx(ubx_rxm_rawx)));
    connect(mTcpServer->packet(), SIGNAL(packetReceived(QByteArray&)),
            this, SLOT(tcpRx(QByteArray&)));
    connect(mTcpServer, SIGNAL(connectionChanged(bool,QString)),
            this, SLOT(tcpConnectionChanged(bool,QString)));
    connect(mRtcmClient, SIGNAL(rtcmReceived(QByteArray,int,bool)),
            this, SLOT(rtcmReceived(QByteArray,int,bool)));
    connect(mPacketInterface, SIGNAL(logEthernetReceived(quint8,QByteArray)),
            this, SLOT(logEthernetReceived(quint8,QByteArray)));
    connect(mLogBroadcaster, SIGNAL(dataReceived(QByteArray&)),
            this, SLOT(logBroadcasterDataReceived(QByteArray&)));

    ros2Server = new QLocalServer(this);
    connect(ros2Server, &QLocalServer::newConnection, this, &CarClient::handleRos2Connection);

    if (!ros2Server->listen("ros2_carclient_channel")) {
        qDebug() << "Unable to start the ROS 2 local server:" << ros2Server->errorString();
        // Handle error
    }
    logLineUsbReceived(0, "All Started!");
}

CarClient::~CarClient()
{
    logStop();
}

void CarClient::handleRos2Connection() {
    QLocalSocket* clientConnection = ros2Server->nextPendingConnection();
    connect(clientConnection, &QLocalSocket::readyRead, this, &CarClient::readRos2Command);
    connect(clientConnection, &QLocalSocket::disconnected, clientConnection, &QLocalSocket::deleteLater);
}

void CarClient::connectSerial(QString port, int baudrate)
{
    qDebug() << "Trying to connect to serial port: " << port;
    if(mSerialPort->isOpen()) {
        mSerialPort->closePort();
    }

    mSerialPort->openPort(port, baudrate);

    mSettings.serialConnect = true;
    mSettings.serialPort = port;
    mSettings.serialBaud = baudrate;

    if(!mSerialPort->isOpen()) {
//        qDebug() << "Serial port connection failed";
        return;
    }

    qDebug() << "Serial port connected";

    mPacketInterface->stopUdpConnection();
    mPacketInterface->getState(255); // To get car ID
}

void CarClient::connectSerialRtcm(QString port, int baudrate)
{
    if(mSerialPortRtcm->isOpen()) {
        mSerialPortRtcm->close();
    }

    mSerialPortRtcm->setPortName(port);
    mSerialPortRtcm->open(QIODevice::ReadWrite);

    mSettings.serialRtcmConnect = true;
    mSettings.serialRtcmPort = port;
    mSettings.serialRtcmBaud = baudrate;

    if(mSerialPortRtcm->isOpen())
    {
        // Tell RC_Controller about it
    } else {
        return;
    }

    qDebug() << "Serial port RTCM connected";

    mSerialPortRtcm->setBaudRate(baudrate);
    mSerialPortRtcm->setDataBits(QSerialPort::Data8);
    mSerialPortRtcm->setParity(QSerialPort::NoParity);
    mSerialPortRtcm->setStopBits(QSerialPort::OneStop);
    mSerialPortRtcm->setFlowControl(QSerialPort::NoFlowControl);
}

void CarClient::connectSerialArduino(QString port, int baudrate)
{
    if(mSerialPortArduino->isOpen()) {
        mSerialPortArduino->close();
    }

    mSerialPortArduino->setPortName(port);
    mSerialPortArduino->open(QIODevice::ReadWrite);

    mSettings.serialArduinoConnect = true;
    mSettings.serialArduinoPort = port;
    mSettings.serialArduinoBaud = baudrate;

    if(mSerialPortArduino->isOpen())
        {
        QByteArray packet;
        packet.clear();
        packet.append(this->mCarId);
        packet.append((char)CMD_ARDUINO_STATUS);
        packet.append((char)1);
        mPacketInterface->sendPacket(packet);

            // Tell RC_Controller about it
        } else
        {
        QByteArray packet;
        packet.clear();
        packet.append(this->mCarId);
        packet.append((char)CMD_ARDUINO_STATUS);
        packet.append((char)0);
        mPacketInterface->sendPacket(packet);
        return;
        }

    qDebug() << "Serial port Arduino connected";
    qDebug() << "Baudrate: " << baudrate;

    mSerialPortArduino->setBaudRate(baudrate);
    mSerialPortArduino->setDataBits(QSerialPort::Data8);
    mSerialPortArduino->setParity(QSerialPort::NoParity);
    mSerialPortArduino->setStopBits(QSerialPort::OneStop);
    mSerialPortArduino->setFlowControl(QSerialPort::NoFlowControl);
}

void CarClient::startRtcmServer(int port)
{
    mRtcmBroadcaster->startTcpServer(port);
}

void CarClient::startUbxServer(int port)
{
    mUbxBroadcaster->startTcpServer(port);
}

void CarClient::startLogServer(int port)
{
    mLogBroadcaster->startTcpServer(port);
}

void CarClient::connectNmea(QString server, int port)
{
    mTcpSocket->close();
    mTcpSocket->connectToHost(server, port);

    mSettings.nmeaConnect = true;
    mSettings.nmeaServer = server;
    mSettings.nmeaPort = port;
}

void CarClient::startUdpServer(int port)
{
    mUdpPort = port + 1;
    mUdpSocket->close();
    mUdpSocket->bind(QHostAddress::Any, port);
}

bool CarClient::startTcpServer(int port, QHostAddress addr)
{
    qDebug() << "Trying to start TCP server. Port: " << port << ", addr: " << addr;
    bool res = mTcpServer->startServer(port,addr);

    if (!res) {
        qDebug() << "Starting TCP server failed:" << mTcpServer->errorString();
    } else
    {
        qDebug() << "Started :-)";
    }

    return res;
}

bool CarClient::enableLogging(QString directory)
{
    if (mLog.isOpen()) {
        mLog.close();
    }

    QString name = QDateTime::currentDateTime().
            toString("LOG_yyyy-MM-dd_hh.mm.ss.log");

    QDir dir;
    dir.mkpath(directory);

    mLog.setFileName(directory + "/" + name);
    return mLog.open(QIODevice::ReadWrite | QIODevice::Truncate);
}

void CarClient::logStop()
{
    if (mLog.isOpen()) {
        qDebug() << "Closing log:" << mLog.fileName();
        mLog.close();
    } else {
        qDebug() << "Log not open";
    }
}

void CarClient::rtcmRx(QByteArray data, int type)
{
    (void)type;

    QString str;

    // Print the packet in the car terminal. NOTE: This is for debugging
    for (int i = 0;i < data.size();i++) {
        str.append(QString().asprintf("%02X ", data.at(i)));
//        str.append(QString().sprintf("%02X ", data.at(i)));
        if (i >= 50) {
            break;
        }
    }

    printTerminal(str);
}

void CarClient::restartRtklib()
{
    QFile ublox("/dev/ublox");
    if (!ublox.exists()) {
        mRtklibRunning = false;
        return;
    }

    mRtklibRunning = true;

    mUblox->disconnectSerial();

    if (mUblox->connectSerial(ublox.fileName())) {
        // Serial port baud rate
        // if it is too low the buffer will overfill and it won't work properly.
        ubx_cfg_prt_uart uart;
        uart.baudrate = 115200;
        uart.in_ubx = true;
        uart.in_nmea = true;
        uart.in_rtcm2 = false;
        uart.in_rtcm3 = true;
        uart.out_ubx = true;
        uart.out_nmea = true;
        uart.out_rtcm3 = true;
        mUblox->ubxCfgPrtUart(&uart);

        // Set configuration
        // Switch on RAWX and NMEA messages, set rate to 1 Hz and time reference to UTC
        mUblox->ubxCfgRate(200, 1, 0);
        mUblox->ubxCfgMsg(UBX_CLASS_RXM, UBX_RXM_RAWX, 1); // Every second
        mUblox->ubxCfgMsg(UBX_CLASS_RXM, UBX_RXM_SFRBX, 1); // Every second
        mUblox->ubxCfgMsg(UBX_CLASS_NMEA, UBX_NMEA_GGA, 1); // Every second

        // Automotive dynamic model
        ubx_cfg_nav5 nav5;
        memset(&nav5, 0, sizeof(ubx_cfg_nav5));
        nav5.apply_dyn = true;
        nav5.dyn_model = 4;
        mUblox->ubxCfgNav5(&nav5);

        // Time pulse configuration
        ubx_cfg_tp5 tp5;
        memset(&tp5, 0, sizeof(ubx_cfg_tp5));
        tp5.active = true;
        tp5.polarity = true;
        tp5.alignToTow = true;
        tp5.lockGnssFreq = true;
        tp5.lockedOtherSet = true;
        tp5.syncMode = false;
        tp5.isFreq = false;
        tp5.isLength = true;
        tp5.freq_period = 1000000;
        tp5.pulse_len_ratio = 0;
        tp5.freq_period_lock = 1000000;
        tp5.pulse_len_ratio_lock = 100000;
        tp5.gridUtcGnss = 0;
        tp5.user_config_delay = 0;
        tp5.rf_group_delay = 0;
        tp5.ant_cable_delay = 50;
        mUblox->ubloxCfgTp5(&tp5);
    }

    QString user = qgetenv("USER");
    if (user.isEmpty()) {
        user = qgetenv("USERNAME");
    }

    if (user.isEmpty()) {
        QProcess process;
        process.setEnvironment(QProcess::systemEnvironment());
        process.start("whoami");
        waitProcess(process);
        user = QString(process.readAllStandardOutput());
        user.replace("\n", "");
    }

    QProcess process;
    process.setEnvironment(QProcess::systemEnvironment());
    process.start("screen", QStringList() <<
                  "-X" << "-S" << "rtklib" << "quit");
    waitProcess(process);

    QProcess process2;
    process2.setEnvironment(QProcess::systemEnvironment());
    process2.start("killall", QStringList() << "rtkrcv");
    waitProcess(process2);

    QProcess process3;
    process3.setEnvironment(QProcess::systemEnvironment());
    process3.start("screen", QStringList() <<
                   "-d" << "-m" << "-S" << "rtklib" << "bash" << "-c" <<
                   QString("cd /home/%1/rise_sdvp/Linux/RTK/rtkrcv_arm && ./start_ublox ; bash").arg(user));
    waitProcess(process3);
}

PacketInterface *CarClient::packetInterface()
{
    return mPacketInterface;
}

bool CarClient::isRtklibRunning()
{
    return mRtklibRunning;
}

quint8 CarClient::carId()
{
    return mCarId;
}

void CarClient::setCarId(quint8 id)
{
    mCarId = id;
}

void CarClient::connectNtrip(QString server, QString stream, QString user, QString pass, int port)
{
    mRtcmClient->connectNtrip(server, stream, user, pass, port);
}

void CarClient::setSendRtcmBasePos(bool send, double lat, double lon, double height)
{
    mRtcmSendBase = send;
    mRtcmBaseLat = lat;
    mRtcmBaseLon = lon;
    mRtcmBaseHeight = height;
}

void CarClient::rebootSystem(bool powerOff)
{
    // https://askubuntu.com/questions/159007/how-do-i-run-specific-sudo-commands-without-a-password
    QStringList args;
    QString cmd = "sudo";

    if (powerOff) {
        args << "shutdown" << "-h" << "now";
    } else {
        args << "reboot";
    }

    QProcess process;
    process.setEnvironment(QProcess::systemEnvironment());
    process.start(cmd, args);
    waitProcess(process);

    qApp->quit();
}

QVariantList CarClient::getNetworkAddresses()
{
    QVariantList res;

    for(QHostAddress a: QNetworkInterface::allAddresses()) {
        if(!a.isLoopback()) {
            if (a.protocol() == QAbstractSocket::IPv4Protocol) {
                res << a.toString();
            }
        }
    }

    return res;
}

int CarClient::getBatteryCells()
{
    return mBatteryCells;
}

void CarClient::setBatteryCells(int cells)
{
    mBatteryCells = cells;
}

void CarClient::addSimulatedCar(int id)
{
    CarSim *car = new CarSim(this);
    car->setId(id);
    connect(car, SIGNAL(dataToSend(QByteArray)), this, SLOT(processCarData(QByteArray)));
    mSimulatedCars.append(car);
}

CarSim *CarClient::getSimulatedCar(int id)
{
    CarSim *sim = 0;

    for (CarSim *s: mSimulatedCars) {
        if (s->id() == id) {
            sim = s;
            break;
        }
    }

    return sim;
}

void CarClient::serialDataAvailable()
{
    while (mSerialPort->bytesAvailable() > 0) {
        processCarData(mSerialPort->readAll());
    }
}

void CarClient::serialPortError(int error)
{
    qDebug() << "Serial error:" << error;

    if(mSerialPort->isOpen()) {
        mSerialPort->closePort();
    }
}

void CarClient::serialRtcmDataAvailable()
{
    while (mSerialPortRtcm->bytesAvailable() > 0) {
        QByteArray data = mSerialPortRtcm->readAll();
        for (int i = 0;i < data.size();i++) {
            rtcm3_input_data((uint8_t)data.at(i), &rtcmState);
        }
    }
}

void CarClient::readRos2Command() {
    QLocalSocket* clientConnection = qobject_cast<QLocalSocket*>(sender());
    QByteArray rosdata = clientConnection->readAll();

//    const unsigned char *data = reinterpret_cast<const unsigned char *>(rosdata.constData());
//    unsigned int len_packet = std::strlen(reinterpret_cast<const char *>(data));

    rosdata.prepend('\0');
    rosdata.append('\0');
    rosdata.append('\0');
    rosdata.append('\0');
    qDebug() << "Got command 0 via ROS:" << static_cast<unsigned char>(rosdata[0]);
    qDebug() << "Got command 1 via ROS:" << static_cast<unsigned char>(rosdata[1]);
    qDebug() << "Got command 2 via ROS:" << rosdata[2];
    qDebug() << "Got command via ROS:" << rosdata;

    packetDataToSend(rosdata);

    QString response = "OK";

    qDebug() << "Response to ROS: " << response;
    // Send response back to ROS 2 node
    clientConnection->write(response.toUtf8());
    clientConnection->flush();
}


void CarClient::serialArduinoDataAvailable()
{
#define ARDUINO_INTEGER

#ifdef ARDUINO_INTEGER
    while (mSerialPortArduino->bytesAvailable() >= 2) {
        QByteArray data = mSerialPortArduino->read(2);
        if (data.size() == 2) {
                QByteArray packet;
                packet.clear();
                packet.append(this->mCarId);
                packet.append((char)CMD_GETANGLE);
                packet.append(data);

                mPacketInterface->sendPacket(packet);
//            mPacketInterface->sendPacket(data);
        }
    }
#endif

#ifdef ARDUINO_FLOAT
    qDebug() << "Receiving floats!!";

    while (static_cast<quint64>(mSerialPortArduino->bytesAvailable()) >= sizeof(float)) {
        QByteArray data = mSerialPortArduino->read(sizeof(float));

        if (data.size() == sizeof(float)) {
            float value;
            memcpy(&value, data.constData(), sizeof(float));
            qDebug() << "Received float:" << value;

            // Create a QByteArray with the marker byte and the float data
            QByteArray message;
            message.append(static_cast<char>(100)); // Add the marker byte
            message.append((const char*)&value, sizeof(float)); // Add the float data

            // Call tcpRx with the modified message
            tcpRx(message);
        } else {
            qDebug() << "Incomplete data received";
        }
    }

/*
    qDebug() << "Receiving floats!!";

while (static_cast<quint64>(mSerialPortArduino->bytesAvailable()) >= sizeof(float)) {
        QByteArray data = mSerialPortArduino->read(sizeof(float));

        if (data.size() == sizeof(float)) {
            float value;
            memcpy(&value, data.constData(), sizeof(float));
            qDebug() << "Received float:" << value;
        } else {
            qDebug() << "Incomplete data received";
        }
    }
*/
#endif

}

void CarClient::serialRtcmPortError(QSerialPort::SerialPortError error)
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

        if(mSerialPortRtcm->isOpen()) {
            mSerialPortRtcm->close();
        }
    }
}

void CarClient::serialArduinoPortError(QSerialPort::SerialPortError error)
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

        if(mSerialPortArduino->isOpen()) {
            mSerialPortArduino->close();
        }
    }
}


void CarClient::packetDataToSend(QByteArray &data)
{
    // This is a packet from RControlStation going to the car.
    // Inspect data and possibly process it here.
#ifdef JETSON____
   qDebug() << "in CarClient::packetDataToSend. Data: " << data;
#else
    qDebug() << "in CarClient::packetDataToSend. Data: " << data << Qt::flush;
#endif
    bool packetConsumed = false;

    VByteArray vb(data);
    vb.remove(0, data[0]);

    quint8 id = vb.vbPopFrontUint8();
    CMD_PACKET cmd = (CMD_PACKET)vb.vbPopFrontUint8();
    vb.chop(3);

    (void)id;
    qDebug() << "in CarClient::packetDataToSend.... Id: " << id << ", mCarId: " << mCarId << ", cmd: " << cmd;

//    if (id == mCarId || id == 255) {
    if (id == mCarId || mCarId == 255) {
        if (cmd == CMD_CAMERA_STREAM_START) {
        } else if (cmd == CMD_TERMINAL_CMD) {
            QString str(vb);
            qDebug() << "to terminal:" << str;

            if (str == "help") {
                qDebug() << "Read HELP";
                printTerminal("lsusb\n"
                              "  Print information about connected usbs.");
                printTerminal("list_ttys\n"
                              "  Print information about connected ttys.");
                printTerminal("is_arduino_online\n"
                              "  Print information about connected arduino.");
            }
            if (str=="lsusb")
            {
                std::string resultstr=utility::systemcmd("lsusb");
                printTerminal(resultstr.c_str());
            }
            if (str=="list_ttys")
            {
                std::string resultstr=utility::systemcmd("ls /dev/tty*");
                printTerminal(resultstr.c_str());
            }
            if (str=="is_arduino_online")
            {
                if(mSerialPortRtcm->isOpen())
                    {
                    printTerminal("Arduino port open!\n");
                    }
            }
        } else if (cmd == CMD_SET_USER) {
            QString str(vb);
            mUsr=str;
            qDebug() << "User:" << mUsr;
            packetConsumed=true;
        } else if (cmd == CMD_SET_PWD) {
            QString str(vb);
            mPwd=str;
            qDebug() << "Pwd:" << mPwd;
            packetConsumed=true;
        } else if (cmd == CMD_SET_ENU_REF) {
//            int id = vb.vbPopFrontInt16();
            double lat = vb.vbPopFrontDouble64(1.0e16);
            double lon = vb.vbPopFrontDouble64(1.0e16);
            qDebug() << "setting ENU ref";
            stopStr2Str();
            startStr2Str(lat,lon);
        } else if (cmd == CMD_CLEAR_UWB_ANCHORS) {
            mUwbAnchorsNow.clear();
        } else if (cmd == CMD_ADD_UWB_ANCHOR) {
            UWB_ANCHOR a;
            a.id = vb.vbPopFrontInt16();
            a.px = vb.vbPopFrontDouble32Auto();
            a.py = vb.vbPopFrontDouble32Auto();
            a.height = vb.vbPopFrontDouble32Auto();
            mUwbAnchorsNow.append(a);
        }
    }

    if (!packetConsumed) {
        if (mSerialPort->isOpen()) {
            qDebug() << "Packet sent (port open)";
            qDebug() << static_cast<int>(data[2]) << ":" << static_cast<int>(data[3]) << ":" << static_cast<int>(data[4]);
            mSerialPort->writeData(data);
        } else
        {
            qDebug() << "Packet not sent (port closed)";

        }
/*
        for (CarSim *s: mSimulatedCars) {
            s->processData(data);
        }*/
    }
}

void CarClient::tcpDataAvailable()
{
    while (!mTcpSocket->atEnd()) {
        QByteArray data = mTcpSocket->readAll();
        // TODO: Collect data and split at newline. (seems ok)
        mPacketInterface->sendNmeaRadio(mCarId, data);
    }
}

void CarClient::tcpConnected()
{
    qDebug() << "NMEA TCP Connected";
    mTcpConnected = true;
}

void CarClient::tcpDisconnected()
{
    qDebug() << "NMEA TCP Disconnected";
    mTcpConnected = false;
}

void CarClient::rtcmUsbRx(quint8 id, QByteArray data)
{
    mCarId = id;
    mRtcmBroadcaster->broadcastData(data);
}

void CarClient::reconnectTimerSlot()
{
    // Try to reconnect if the connections are lost
    if (mSettings.serialConnect && !mSerialPort->isOpen()) {
 //       qDebug() << "Trying to reconnect serial...";
        connectSerial(mSettings.serialPort, mSettings.serialBaud);
    }

    if (mSettings.serialRtcmConnect && !mSerialPortRtcm->isOpen()) {
//        qDebug() << "Trying to reconnect RTCM serial...";
        connectSerialRtcm(mSettings.serialRtcmPort, mSettings.serialRtcmBaud);
    }

    if (mSettings.serialArduinoConnect && !mSerialPortArduino->isOpen()) {
//        qDebug() << "Trying to reconnect Arduino serial...";
        connectSerialArduino(mSettings.serialArduinoPort, mSettings.serialArduinoBaud);
    }

    if (mSettings.nmeaConnect && !mTcpConnected) {
        qDebug() << "Trying to reconnect nmea tcp...";
        connectNmea(mSettings.nmeaServer, mSettings.nmeaPort);
    }

    if ((mRtcmClient->isTcpConnected() || mRtcmClient->isSerialConnected()) && mRtcmSendBase) {
        mPacketInterface->sendRtcmUsb(255, mRtcmClient->encodeBasePos(mRtcmBaseLat,
                                                                      mRtcmBaseLon,
                                                                      mRtcmBaseHeight,
                                                                      0.0));
    }

    // Set ID of board on regular basis. In case reprogramming is done.
    if (mSerialPort->isOpen() && mCarIdToSet >= 0 && mCarIdToSet < 255) {
        mPacketInterface->sendTerminalCmd(255, QString("set_id_quiet %1").arg(mCarIdToSet));
    }
}

void CarClient::logFlushTimerSlot()
{
    if (mLog.isOpen()) {
//        mLog.flush();
    }
}

void CarClient::readPendingDatagrams()
{
    while (mUdpSocket->hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(mUdpSocket->pendingDatagramSize());
        quint16 senderPort;

        mUdpSocket->readDatagram(datagram.data(), datagram.size(),
                                &mHostAddress, &senderPort);

        mPacketInterface->sendPacket(datagram);
    }
}

void CarClient::carPacketRx(quint8 id, CMD_PACKET cmd, const QByteArray &data)
{
    QByteArray toSend = data;

    /* Uncomment as likely to produce confusing errors when the composition and size of data is changed
    if (cmd == CMD_GET_STATE && mOverrideUwbPos) {
        VByteArray vb;
        vb.append(data.mid(0, 99));
        vb.vbAppendDouble32(mOverrideUwbX, 1e4);
        vb.vbAppendDouble32(mOverrideUwbY, 1e4);
        toSend = vb;
    }
*/
    if (id != 254) {
//        qDebug() << "In CarClient::carPacketRx. Car: " << id;
        mCarId = id;

        if (QString::compare(mHostAddress.toString(), "0.0.0.0") != 0) {
//            qDebug() << "datagramming";
            mUdpSocket->writeDatagram(toSend, mHostAddress, mUdpPort);
        }
//        qDebug() << "tcpservering";
        mTcpServer->packet()->sendPacket(toSend);
    }
}

void CarClient::logLineUsbReceived(quint8 id, QString str)
{
    (void)id;

    if (mLog.isOpen()) {
        qDebug() << "Writing to log: " << str;
        mLog.write(str.toLocal8Bit());
    }
}

void CarClient::systemTimeReceived(quint8 id, qint32 sec, qint32 usec)
{
    (void)id;
    (void)usec;

//    struct timeval now;
//    int rc;
//    now.tv_sec = sec;
//    now.tv_usec = usec;
//    rc = settimeofday(&now, NULL);

    if(setUnixTime(sec)) {
        qDebug() << "Sucessfully set system time";
        restartRtklib();
    } else {
        qDebug() << "Setting system time failed";
    }
}

void CarClient::rebootSystemReceived(quint8 id, bool powerOff)
{
    (void)id;

    logStop();
    sync();

//    if (powerOff) {
//        reboot(RB_POWER_OFF);
//    } else {
//        reboot(RB_AUTOBOOT);
//    }

    rebootSystem(powerOff);
}

void CarClient::ubxRx(const QByteArray &data)
{
    mUbxBroadcaster->broadcastData(data);
}

void CarClient::rxRawx(ubx_rxm_rawx rawx)
{
    if (!rawx.leap_sec) {
        // Leap seconds are not known...
        return;
    }

    QDateTime dateGps(QDate(1980, 1, 6), QTime(0, 0, 0));
    dateGps.setOffsetFromUtc(0);
    dateGps = dateGps.addDays(rawx.week * 7);
    dateGps = dateGps.addMSecs((rawx.rcv_tow - (double)rawx.leaps) * 1000.0);

    QDateTime date = QDateTime::currentDateTime();
    qint64 diff = dateGps.toMSecsSinceEpoch() - date.toMSecsSinceEpoch();

    if (abs(diff) > 60000) {
        // Set the system time if the difference is over 10 seconds.
        qDebug() << "System time is different from GPS time. Difference: " << diff << " ms";

//        struct timeval now;
//        int rc;
//        now.tv_sec = dateGps.toTime_t();
//        now.tv_usec = dateGps.time().msec() * 1000.0;
//        rc = settimeofday(&now, NULL);

//        if(setUnixTime(dateGps.toTime_t())) {
          if(setUnixTime(dateGps.toSecsSinceEpoch())) {
            qDebug() << "Sucessfully updated system time";
            restartRtklib();
        } else {
            qDebug() << "Setting system time failed";
        }
    }
}

void CarClient::tcpRx(QByteArray &data)
{
    qDebug() << "In CarClient::tcpRx";
    qDebug() << "data: " << data;
    mPacketInterface->sendPacket(data);
}

void CarClient::tcpConnectionChanged(bool connected, QString address)
{
    if (connected) {
        qDebug() << "TCP connection from" << address << "accepted";
    } else {
        qDebug() << "Disconnected TCP from" << address;
    }
}

void CarClient::rtcmReceived(QByteArray data, int type, bool sync)
{
    (void)type;
    (void)sync;
    mPacketInterface->sendRtcmUsb(255, data);
}

void CarClient::logEthernetReceived(quint8 id, QByteArray data)
{
    (void)id;
    mLogBroadcaster->broadcastData(data);
}

void CarClient::processCarData(QByteArray data)
{
    mPacketInterface->processData(data);
}

void CarClient::cameraImageCaptured(QImage img)
{
    (void)img;
}

void CarClient::logBroadcasterDataReceived(QByteArray &data)
{
    mLogBroadcasterDataBuffer.append(data);

    int newLineIndex = mLogBroadcasterDataBuffer.indexOf("\n");
    while (newLineIndex >= 0) {
        QString line = mLogBroadcasterDataBuffer.left(newLineIndex);
        mLogBroadcasterDataBuffer.remove(0, newLineIndex + 1);

        QStringList tokens = line.split(" ");

        if (tokens.at(0) == "plot_init") {
            if (tokens.size() == 3) {
                QByteArray data;
                data.append((quint8)mCarId);
                data.append((char)CMD_PLOT_INIT);
                data.append(tokens.at(1).toLocal8Bit());
                data.append('\0');
                data.append(tokens.at(2).toLocal8Bit());
                data.append('\0');
                carPacketRx(mCarId, CMD_PLOT_INIT, data);
            }
        } else if (tokens.at(0) == "plot_add_graph") {
            if (tokens.size() == 2) {
                QByteArray data;
                data.append((quint8)mCarId);
                data.append((char)CMD_PLOT_ADD_GRAPH);
                data.append(tokens.at(1).toLocal8Bit());
                data.append('\0');
                carPacketRx(mCarId, CMD_PLOT_ADD_GRAPH, data);
            }
        } else if (tokens.at(0) == "plot_set_graph") {
            if (tokens.size() == 2) {
                QByteArray data;
                data.append((quint8)mCarId);
                data.append((char)CMD_PLOT_SET_GRAPH);
                data.append(tokens.at(1).toInt());
                data.append('\0');
                carPacketRx(mCarId, CMD_PLOT_SET_GRAPH, data);
            }
        } else if (tokens.at(0) == "plot_add_sample") {
            if (tokens.size() == 3) {
                VByteArray data;
                data.append((quint8)mCarId);
                data.append((char)CMD_PLOT_DATA);
                data.vbAppendDouble32Auto(tokens.at(1).toDouble());
                data.vbAppendDouble32Auto(tokens.at(2).toDouble());
                carPacketRx(mCarId, CMD_PLOT_DATA, data);
            }
        } else if (tokens.at(0) == "uwb_pos_override") {
            if (tokens.size() == 3) {
                mOverrideUwbPos = true;
                mOverrideUwbX = tokens.at(1).toDouble();
                mOverrideUwbY = tokens.at(2).toDouble();
            }
        } else if (tokens.at(0) == "uwb_pos_override_stop") {
            if (tokens.size() == 1) {
                mOverrideUwbPos = false;
            }
        } else if (tokens.at(0) == "get_uwb_anchor_list") {
            QString reply;
            reply.append("anchors");
            for (UWB_ANCHOR a: mUwbAnchorsNow) {
                reply.append(QString(" %1 %2 %3 %4").
                             arg(a.id).
                             arg(a.px, 0, 'f', 3).
                             arg(a.py, 0, 'f', 3).
                             arg(a.height, 0, 'f', 3));
            }
            reply.append("\r\n");

            mLogBroadcaster->broadcastData(reply.toLocal8Bit());
        }

        newLineIndex = mLogBroadcasterDataBuffer.indexOf("\n");
    }
}

int CarClient::getCarIdToSet() const
{
    return mCarIdToSet;
}

/**
 * @brief CarClient::setCarIdToSet
 * Set ID that the connected board will be updated with every 2 seconds.
 *
 * @param carIdToSet
 * The ID.
 */
void CarClient::setCarIdToSet(int carIdToSet)
{
    mCarIdToSet = carIdToSet;
}

bool CarClient::setUnixTime(qint64 t)
{
    // https://askubuntu.com/questions/159007/how-do-i-run-specific-sudo-commands-without-a-password
    QProcess process;
    process.setEnvironment(QProcess::systemEnvironment());
    process.start("sudo", QStringList() << "date" << "+%s" << "-s" << QString("@%1").arg(t));
    return waitProcess(process, 5000);
}

void CarClient::printTerminal(QString str)
{
    QByteArray packet;
    packet.append((quint8)mCarId);
    packet.append((char)CMD_PRINTF);
    packet.append(str.toLocal8Bit());
    carPacketRx(mCarId, CMD_PRINTF, packet);

    // Existing behavior (e.g., print to the terminal)
//    qDebug() << "Terminal Output:" << str;
    // Attempt to connect to the local socket without blocking
    QLocalSocket socket;
    socket.connectToServer("ros2_carclient_terminal_channel");
    if (socket.state() == QLocalSocket::UnconnectedState) {
     //   qDebug() << "No ROS 2 channel available, skipping publication.";
    } else
    {
        if (!socket.waitForConnected(0)) {  // Non-blocking wait
     //   qDebug() << "Failed to connect to ROS 2 channel.";
        } else
        {
        // Publish the message if the server is available
     //   qDebug() << "Published: " << str;
        QByteArray data = str.toUtf8();
        socket.write(data);
        socket.flush();
        socket.disconnectFromServer();
        }
    }

}

void CarClient::printLog(QString str)
{
    QByteArray packet;
    packet.append((quint8)mCarId);
    packet.append((char)CMD_PRINTLOG);
    packet.append(str.toLocal8Bit());
    carPacketRx(mCarId, CMD_PRINTLOG, packet);
}

bool CarClient::waitProcess(QProcess &process, int timeoutMs)
{
    bool killed = false;

    process.waitForStarted();

    QEventLoop loop;
    QTimer timeoutTimer;
    timeoutTimer.setSingleShot(true);
    timeoutTimer.start(timeoutMs);
    connect(&process, SIGNAL(finished(int)), &loop, SLOT(quit()));
    connect(&timeoutTimer, SIGNAL(timeout()), &loop, SLOT(quit()));
    loop.exec();

    if (process.state() == QProcess::Running) {
        process.kill();
        process.waitForFinished();
        killed = true;
    }

    return !killed;
}

void CarClient::startStr2Str(double lat,double lon ) {
    qDebug() << "Lat: " << lat << ", Lon: " << lon;
    QString strLat=QString("%1").arg(lat, 0, 'f', 14);
    QString strLon=QString("%1").arg(lon, 0, 'f', 14);
//    QString exec="str2str -in ntrip://"+mUsr+":"+mPwd+"@nrtk-swepos.lm.se:80/RTCM3_GNSS -p "+strLat+" "+strLon+" 17 -n 1 -out serial://rtk:115200:8:n:1 -msg ""1005,1074,1084,1094,1230""";
//    qDebug() << exec;

    QStringList args;
    args << "-in" << QString("ntrip://%1:%2@nrtk-swepos.lm.se:80/RTCM3_GNSS").arg(mUsr, mPwd)
         << "-p" << strLat << strLon << "17"
         << "-n" << "1"
         << "-out" << "serial://rtk:115200:8:n:1"
         << "-msg" << "1005,1074,1084,1094,1230";

    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    s2sProcess.setProcessEnvironment(env);

    s2sProcess.setProgram("str2str");
    s2sProcess.setArguments(args);
    s2sProcess.start();

//    s2sProcess.start(exec);
    if (!s2sProcess.waitForStarted()) {
        qCritical() << "Failed to start str2str.";
    } else {
        qDebug() << "Started str2str with PID:" << s2sProcess.processId();
    }

//    str2strProcess = std::make_shared<sjs::process>(exec);
//    if (!str2strProcess->running()) {
//        std::cerr << "Failed to start str2str." << std::endl;
//    } else {
//        std::cout << "Started str2str with PID: " << str2strProcess->id() << std::endl;
//    }
}

void CarClient::stopStr2Str() {
    if (s2sProcess.state() == QProcess::Running) {
        s2sProcess.terminate();
        s2sProcess.waitForFinished();
        qDebug() << "Stopped str2str with PID:" << s2sProcess.processId();
    } else {
        qCritical() << "No running instance of str2str to stop.";
    }

//    if (str2strProcess && str2strProcess->running()) {
//        str2strProcess->terminate(); // Terminate the process
//        std::cout << "Stopped str2str with PID: " << str2strProcess->id() << std::endl;
//        str2strProcess.reset(); // Reset the shared pointer
//    } else {
//        std::cerr << "No running instance of str2str to stop." << std::endl;
//    }

}
