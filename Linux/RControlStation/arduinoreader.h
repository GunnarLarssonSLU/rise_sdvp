#ifndef ARDUINOREADER_H
#define ARDUINOREADER_H

#include <QObject>
#include <QCoreApplication>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QDebug>
#include <QTimer>

class ArduinoReader : public QObject
{
    Q_OBJECT

public:
    ArduinoReader(const QString &portName, qint32 baudRate, QObject *parent = nullptr);
    ~ArduinoReader();

private slots:
    void readData();

    void checkSignal();

signals:
    void signalLost();

private:
    QSerialPort *serialPort;
    QTimer *timer;
    bool receivedOne = true;
    bool bEmergencyPressed=false;

};


#endif // ARDUINOREADER_H
