/*
    Copyright 2017 Benjamin Vedder	benjamin@vedder.se

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

#include "sensorplot.h"
#include "ui_sensorplot.h"

SensorPlot::SensorPlot(QWidget *parent) :
    QWidget(parent),
    ui(new Ui::SensorPlot)
{
    ui->setupUi(this);
    layout()->setContentsMargins(0, 0, 0, 0);

    mMaxSampleSize = 1000;
    mAccelXData.resize(mMaxSampleSize);
    mAccelYData.resize(mMaxSampleSize);
    mAccelZData.resize(mMaxSampleSize);
    mGyroXData.resize(mMaxSampleSize);
    mGyroYData.resize(mMaxSampleSize);
    mGyroZData.resize(mMaxSampleSize);
    mMagXData.resize(mMaxSampleSize);
    mMagYData.resize(mMaxSampleSize);
    mMagZData.resize(mMaxSampleSize);
    qDebug() << "In SensorPlot consstructor - 3";
    mAccelGyroMagXAxis.resize(mMaxSampleSize);
    for(int i = 0;i < mAccelGyroMagXAxis.size();i++) {
        mAccelGyroMagXAxis[mAccelGyroMagXAxis.size() - i - 1] = (40.0 / 1000.0 * i);
    }
    qDebug() << "In SensorPlot consstructor - 4";

    ui->inst1Plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->inst2Plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
    ui->inst3Plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

    ui->inst1Plot->clearGraphs();
    ui->inst1Plot->addGraph();
    ui->inst1Plot->xAxis->setRangeReversed(true);
    ui->inst1Plot->graph()->setPen(QPen(Qt::black));
    ui->inst1Plot->graph()->setData(mAccelGyroMagXAxis, mAccelXData);
    ui->inst1Plot->graph()->setName(tr("X"));
    ui->inst1Plot->addGraph();
    ui->inst1Plot->graph()->setPen(QPen(Qt::green));
    ui->inst1Plot->graph()->setData(mAccelGyroMagXAxis, mAccelYData);
    ui->inst1Plot->graph()->setName(tr("Y"));
    ui->inst1Plot->addGraph();
    ui->inst1Plot->graph()->setPen(QPen(Qt::blue));
    ui->inst1Plot->graph()->setData(mAccelGyroMagXAxis, mAccelZData);
    ui->inst1Plot->graph()->setName(tr("Z"));
    ui->inst1Plot->rescaleAxes();
    ui->inst1Plot->xAxis->setLabel("Seconds");
    ui->inst1Plot->yAxis->setLabel("Instrument 1 (?)");
    ui->inst1Plot->legend->setVisible(true);
    ui->inst1Plot->replot();

    ui->inst2Plot->clearGraphs();
    ui->inst2Plot->addGraph();
    ui->inst2Plot->xAxis->setRangeReversed(true);
    ui->inst2Plot->graph()->setPen(QPen(Qt::black));
    ui->inst2Plot->graph()->setData(mAccelGyroMagXAxis, mGyroXData);
    ui->inst2Plot->graph()->setName(tr("X"));
    ui->inst2Plot->addGraph();
    ui->inst2Plot->graph()->setPen(QPen(Qt::green));
    ui->inst2Plot->graph()->setData(mAccelGyroMagXAxis, mGyroYData);
    ui->inst2Plot->graph()->setName(tr("Y"));
    ui->inst2Plot->addGraph();
    ui->inst2Plot->graph()->setPen(QPen(Qt::blue));
    ui->inst2Plot->graph()->setData(mAccelGyroMagXAxis, mGyroZData);
    ui->inst2Plot->graph()->setName(tr("Z"));
    ui->inst2Plot->rescaleAxes();
    ui->inst2Plot->xAxis->setLabel("Seconds");
    ui->inst2Plot->yAxis->setLabel("Instrument 2 (?)");
    ui->inst2Plot->legend->setVisible(true);
    ui->inst2Plot->replot();

    ui->inst3Plot->clearGraphs();
    ui->inst3Plot->addGraph();
    ui->inst3Plot->xAxis->setRangeReversed(true);
    ui->inst3Plot->graph()->setPen(QPen(Qt::black));
    ui->inst3Plot->graph()->setData(mAccelGyroMagXAxis, mMagXData);
    ui->inst3Plot->graph()->setName(tr("X"));
    ui->inst3Plot->addGraph();
    ui->inst3Plot->graph()->setPen(QPen(Qt::green));
    ui->inst3Plot->graph()->setData(mAccelGyroMagXAxis, mMagYData);
    ui->inst3Plot->graph()->setName(tr("Y"));
    ui->inst3Plot->addGraph();
    ui->inst3Plot->graph()->setPen(QPen(Qt::blue));
    ui->inst3Plot->graph()->setData(mAccelGyroMagXAxis, mMagZData);
    ui->inst3Plot->graph()->setName(tr("Z"));
    ui->inst3Plot->rescaleAxes();
    ui->inst3Plot->xAxis->setLabel("Seconds");
    ui->inst2Plot->yAxis->setLabel("Instrument 3 (?)");
    ui->inst3Plot->legend->setVisible(true);
    ui->inst3Plot->replot();

    mTimer = new QTimer(this);
    mTimer->start(20);
    mSensorReplot = false;

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
}

SensorPlot::~SensorPlot()
{
    delete ui;
}

void SensorPlot::addSample(double *accel, double *gyro, double *mag)
{
    mAccelXData.append(accel[0]);
    mAccelXData.remove(0, 1);
    mAccelYData.append(accel[1]);
    mAccelYData.remove(0, 1);
    mAccelZData.append(accel[2]);
    mAccelZData.remove(0, 1);

    mGyroXData.append(gyro[0] * 180.0 / M_PI);
    mGyroXData.remove(0, 1);
    mGyroYData.append(gyro[1] * 180.0 / M_PI);
    mGyroYData.remove(0, 1);
    mGyroZData.append(gyro[2] * 180.0 / M_PI);
    mGyroZData.remove(0, 1);

    mMagXData.append(mag[0]);
    mMagXData.remove(0, 1);
    mMagYData.append(mag[1]);
    mMagYData.remove(0, 1);
    mMagZData.append(mag[2]);
    mMagZData.remove(0, 1);

    mSensorReplot = true;
}

void SensorPlot::timerSlot()
{
    if (mSensorReplot) {
        ui->inst1Plot->graph(0)->setData(mAccelGyroMagXAxis, mAccelXData);
        ui->inst1Plot->graph(1)->setData(mAccelGyroMagXAxis, mAccelYData);
        ui->inst1Plot->graph(2)->setData(mAccelGyroMagXAxis, mAccelZData);
        ui->inst1Plot->rescaleAxes();
        ui->inst1Plot->replot();

        ui->inst2Plot->graph(0)->setData(mAccelGyroMagXAxis, mGyroXData);
        ui->inst2Plot->graph(1)->setData(mAccelGyroMagXAxis, mGyroYData);
        ui->inst2Plot->graph(2)->setData(mAccelGyroMagXAxis, mGyroZData);
        ui->inst2Plot->rescaleAxes();
        ui->inst2Plot->replot();

        ui->inst3Plot->graph(0)->setData(mAccelGyroMagXAxis, mMagXData);
        ui->inst3Plot->graph(1)->setData(mAccelGyroMagXAxis, mMagYData);
        ui->inst3Plot->graph(2)->setData(mAccelGyroMagXAxis, mMagZData);
        ui->inst3Plot->rescaleAxes();
        ui->inst3Plot->replot();

        mSensorReplot = false;
    }
}
