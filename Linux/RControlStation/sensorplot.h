#ifndef SENSORPLOT_H
#define SENSORPLOT_H

#include <QWidget>

namespace Ui {
class SensorPlot;
}

class SensorPlot : public QWidget
{
    Q_OBJECT

public:
    explicit SensorPlot(QWidget *parent = 0);
    ~SensorPlot();

    void addSample(double *accel, double *gyro, double *mag);

private slots:
    void timerSlot();

private:
    Ui::SensorPlot *ui;
    QTimer *mTimer;
    bool mSensorReplot;

    QVector<double> mAccelXData;
    QVector<double> mAccelYData;
    QVector<double> mAccelZData;
    QVector<double> mGyroXData;
    QVector<double> mGyroYData;
    QVector<double> mGyroZData;
    QVector<double> mMagXData;
    QVector<double> mMagYData;
    QVector<double> mMagZData;
    QVector<double> mAccelGyroMagXAxis;
    int mMaxSampleSize;

};


#endif // SENSORPLOT_H
