#include "arduinoreader.h"

ArduinoReader::ArduinoReader(const QString &portName, qint32 baudRate, QObject *parent = nullptr)
        : QObject(parent), serialPort(new QSerialPort(this))
{
    serialPort->setPortName(portName);
    serialPort->setBaudRate(baudRate);

    if (!serialPort->open(QIODevice::ReadOnly)) {
        qCritical() << "Failed to open serial port2:" << portName << serialPort->errorString();
        return;
    }

    connect(serialPort, &QSerialPort::readyRead, this, &ArduinoReader::readData);


    // Set up a timer to check for the signal
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &ArduinoReader::checkSignal);
    timer->start(1000); // Check every second


}

ArduinoReader::~ArduinoReader()
{
    if (serialPort->isOpen()) {
        serialPort->close();
    }
}

void ArduinoReader::readData()
{
    if (serialPort->bytesAvailable() > 0) {
        QByteArray data = serialPort->readAll();
        qDebug() << data.length();
        if (data[0] == 1) {
            receivedOne = true;
            bEmergencyPressed=false;
        }
    }
}

void ArduinoReader::checkSignal()
{
    if ((!bEmergencyPressed) && (!receivedOne)) {
        bEmergencyPressed=true;
        emit signalLost();
    }
    receivedOne = false; // Reset for the next check
}

