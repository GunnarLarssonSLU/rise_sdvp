/********************************************************************************
** Form generated from reading UI file 'carinterface.ui'
**
** Created by: Qt User Interface Compiler version 6.9.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CARINTERFACE_H
#define UI_CARINTERFACE_H

#include <QtCore/QVariant>
#include <QtGui/QIcon>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLCDNumber>
#include <QtWidgets/QLabel>
#include <QtWidgets/QProgressBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>
#include "confcommonwidget.h"
#include "historylineedit.h"
#include "imuplot.h"
#include "magcal.h"
#include "nmeawidget.h"

QT_BEGIN_NAMESPACE

class Ui_CarInterface
{
public:
    QHBoxLayout *horizontalLayout_5;
    QVBoxLayout *verticalLayout;
    QTabWidget *tabWidget;
    QWidget *vehicleStateTab;
    QHBoxLayout *horizontalLayout;
    QVBoxLayout *vehicleStateLeftLayout;
    QSpacerItem *orientationSpacer;
    QProgressBar *rollBar;
    QProgressBar *pitchBar;
    QProgressBar *yawBar;
    QProgressBar *speedBar;
    QGridLayout *gridLayout;
    QLabel *label00;
    QLabel *label01;
    QLabel *label10;
    QLabel *label11;
    QLabel *label20;
    QLabel *label21;
    QLabel *label30;
    QLabel *label31;
    QSpacerItem *gridBottomSpacer;
    QWidget *tab;
    QLabel *label_log1;
    QWidget *tab_2;
    QVBoxLayout *verticalLayout_4;
    ImuPlot *imuPlot;
    QWidget *tab_3;
    QVBoxLayout *verticalLayout_10;
    QHBoxLayout *horizontalLayout_3;
    QTextBrowser *terminalBrowser;
    QVBoxLayout *verticalLayout_3;
    QLabel *label_8;
    QPushButton *zeroGyroButton;
    QLabel *label;
    QPushButton *ubxVersionButton;
    QPushButton *ubxNavSatButton;
    QPushButton *ubxSolButton;
    QPushButton *ubxRelPosNedButton;
    QPushButton *ubxCfgGnssButton;
    QLabel *label_5;
    QPushButton *uwbListAnchorsButton;
    QPushButton *uwbResetPosButton;
    QPushButton *uwbUptimeButton;
    QPushButton *uwbRebootButton;
    QSpacerItem *verticalSpacer_2;
    QHBoxLayout *horizontalLayout_12;
    HistoryLineEdit *terminalEdit;
    QPushButton *terminalSendButton;
    QPushButton *terminalClearButton;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_4;
    HistoryLineEdit *terminalEditVesc;
    QPushButton *terminalSendVescButton;
    QWidget *tab_4;
    QVBoxLayout *verticalLayout_9;
    QGroupBox *groupBox_3;
    QVBoxLayout *verticalLayout_7;
    QGridLayout *gridLayout1;
    QLCDNumber *servoDirectNumber;
    QLabel *label_3;
    QSlider *servoMappedSlider;
    QLabel *label_2;
    QSlider *servoDirectSlider;
    QLCDNumber *servoMappedNumber;
    QLabel *label_9;
    QSlider *ioBoardPwmSlider;
    QLCDNumber *ioBoardPwmNumber;
    QGroupBox *groupBox_2;
    QGridLayout *gridLayout_4;
    MagCal *magCal;
    QWidget *tab_6;
    QVBoxLayout *verticalLayout_8;
    NmeaWidget *nmeaWidget;
    QWidget *tab_5;
    QVBoxLayout *verticalLayout_6;
    QGroupBox *groupBox_4;
    QGridLayout *gridLayout_3;
    QCheckBox *confUseUwbPosBox;
    QDoubleSpinBox *confYawImuGainBox;
    QCheckBox *confClampImuYawBox;
    QCheckBox *confMiscDisableMotorBox;
    QDoubleSpinBox *confMotorPoleBox;
    QCheckBox *confMiscSimulateMotorBox;
    QDoubleSpinBox *confServoDGainBox;
    QDoubleSpinBox *confSteeringRampBox;
    QDoubleSpinBox *confSensorCentreBox;
    QDoubleSpinBox *confWheelDiamBox;
    QCheckBox *confOdometryYawBox;
    QDoubleSpinBox *confDegreeIntervalBox;
    QDoubleSpinBox *confServoRangeBox;
    QDoubleSpinBox *confTurnRadBox;
    QDoubleSpinBox *confGearRatioBox;
    QDoubleSpinBox *confAxisDistanceBox;
    QDoubleSpinBox *confServoCenterBox;
    QDoubleSpinBox *confServoIGainBox;
    QDoubleSpinBox *confServoPGainBox;
    QDoubleSpinBox *confSensorIntervalBox;
    QDoubleSpinBox *confDeadBandBox;
    QDoubleSpinBox *confTimeLimitBox;
    ConfCommonWidget *confCommonWidget;
    QHBoxLayout *horizontalLayout_7;
    QSpacerItem *horizontalSpacer_3;
    QPushButton *confReadButton;
    QPushButton *confReadDefaultButton;
    QPushButton *confWriteButton;
    QWidget *tab_7;
    QLabel *mcFaultLabel;
    QHBoxLayout *horizontalLayout1;
    QLabel *fwLabel;
    QProgressBar *batteryBar;
    QProgressBar *tempFetBar;
    QLabel *clockLabel;
    QVBoxLayout *verticalLayout_2;
    QSpinBox *idBox;
    QCheckBox *pollBox;
    QCheckBox *keyboardControlBox;
    QCheckBox *autopilotBox;
    QCheckBox *updateRouteFromMapBox;
    QCheckBox *vescToolTcpBox;
    QPushButton *clearRouteButton;
    QPushButton *setClockButton;
    QPushButton *setClockPiButton;
    QPushButton *rebootPiButton;
    QPushButton *shutdownPiButton;
    QPushButton *restartCarClientButton;
    QSpacerItem *verticalSpacer;

    void setupUi(QWidget *CarInterface)
    {
        if (CarInterface->objectName().isEmpty())
            CarInterface->setObjectName("CarInterface");
        CarInterface->resize(870, 557);
        horizontalLayout_5 = new QHBoxLayout(CarInterface);
        horizontalLayout_5->setObjectName("horizontalLayout_5");
        verticalLayout = new QVBoxLayout();
        verticalLayout->setObjectName("verticalLayout");
        tabWidget = new QTabWidget(CarInterface);
        tabWidget->setObjectName("tabWidget");
        tabWidget->setTabPosition(QTabWidget::TabPosition::South);
        tabWidget->setTabShape(QTabWidget::TabShape::Triangular);
        tabWidget->setTabsClosable(false);
        vehicleStateTab = new QWidget();
        vehicleStateTab->setObjectName("vehicleStateTab");
        horizontalLayout = new QHBoxLayout(vehicleStateTab);
        horizontalLayout->setObjectName("horizontalLayout");
        vehicleStateLeftLayout = new QVBoxLayout();
        vehicleStateLeftLayout->setObjectName("vehicleStateLeftLayout");
        orientationSpacer = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        vehicleStateLeftLayout->addItem(orientationSpacer);

        rollBar = new QProgressBar(vehicleStateTab);
        rollBar->setObjectName("rollBar");
        rollBar->setMinimum(-180);
        rollBar->setMaximum(180);
        rollBar->setValue(0);

        vehicleStateLeftLayout->addWidget(rollBar);

        pitchBar = new QProgressBar(vehicleStateTab);
        pitchBar->setObjectName("pitchBar");
        pitchBar->setMinimum(-180);
        pitchBar->setMaximum(180);
        pitchBar->setValue(0);

        vehicleStateLeftLayout->addWidget(pitchBar);

        yawBar = new QProgressBar(vehicleStateTab);
        yawBar->setObjectName("yawBar");
        yawBar->setMinimum(-180);
        yawBar->setMaximum(180);
        yawBar->setValue(0);

        vehicleStateLeftLayout->addWidget(yawBar);

        speedBar = new QProgressBar(vehicleStateTab);
        speedBar->setObjectName("speedBar");
        speedBar->setMaximum(120);
        speedBar->setValue(0);

        vehicleStateLeftLayout->addWidget(speedBar);


        horizontalLayout->addLayout(vehicleStateLeftLayout);

        gridLayout = new QGridLayout();
        gridLayout->setObjectName("gridLayout");
        label00 = new QLabel(vehicleStateTab);
        label00->setObjectName("label00");

        gridLayout->addWidget(label00, 0, 0, 1, 1);

        label01 = new QLabel(vehicleStateTab);
        label01->setObjectName("label01");

        gridLayout->addWidget(label01, 0, 1, 1, 1);

        label10 = new QLabel(vehicleStateTab);
        label10->setObjectName("label10");

        gridLayout->addWidget(label10, 1, 0, 1, 1);

        label11 = new QLabel(vehicleStateTab);
        label11->setObjectName("label11");

        gridLayout->addWidget(label11, 1, 1, 1, 1);

        label20 = new QLabel(vehicleStateTab);
        label20->setObjectName("label20");

        gridLayout->addWidget(label20, 2, 0, 1, 1);

        label21 = new QLabel(vehicleStateTab);
        label21->setObjectName("label21");

        gridLayout->addWidget(label21, 2, 1, 1, 1);

        label30 = new QLabel(vehicleStateTab);
        label30->setObjectName("label30");

        gridLayout->addWidget(label30, 3, 0, 1, 1);

        label31 = new QLabel(vehicleStateTab);
        label31->setObjectName("label31");

        gridLayout->addWidget(label31, 3, 1, 1, 1);

        gridBottomSpacer = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        gridLayout->addItem(gridBottomSpacer, 4, 0, 1, 2);


        horizontalLayout->addLayout(gridLayout);

        QIcon icon;
        icon.addFile(QString::fromUtf8(":/models/Icons/Orthogonal View-96.png"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        tabWidget->addTab(vehicleStateTab, icon, QString());
        tab = new QWidget();
        tab->setObjectName("tab");
        label_log1 = new QLabel(tab);
        label_log1->setObjectName("label_log1");
        label_log1->setGeometry(QRect(20, 10, 67, 17));
        tabWidget->addTab(tab, QString());
        tab_2 = new QWidget();
        tab_2->setObjectName("tab_2");
        verticalLayout_4 = new QVBoxLayout(tab_2);
        verticalLayout_4->setObjectName("verticalLayout_4");
        imuPlot = new ImuPlot(tab_2);
        imuPlot->setObjectName("imuPlot");

        verticalLayout_4->addWidget(imuPlot);

        QIcon icon1;
        icon1.addFile(QString::fromUtf8(":/models/Icons/Gyroscope-96.png"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        tabWidget->addTab(tab_2, icon1, QString());
        tab_3 = new QWidget();
        tab_3->setObjectName("tab_3");
        verticalLayout_10 = new QVBoxLayout(tab_3);
        verticalLayout_10->setObjectName("verticalLayout_10");
        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName("horizontalLayout_3");
        terminalBrowser = new QTextBrowser(tab_3);
        terminalBrowser->setObjectName("terminalBrowser");
        QFont font;
        font.setFamilies({QString::fromUtf8("Monospace")});
        terminalBrowser->setFont(font);

        horizontalLayout_3->addWidget(terminalBrowser);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(2);
        verticalLayout_3->setObjectName("verticalLayout_3");
        label_8 = new QLabel(tab_3);
        label_8->setObjectName("label_8");
        QFont font1;
        font1.setBold(true);
        label_8->setFont(font1);
        label_8->setAlignment(Qt::AlignmentFlag::AlignCenter);

        verticalLayout_3->addWidget(label_8);

        zeroGyroButton = new QPushButton(tab_3);
        zeroGyroButton->setObjectName("zeroGyroButton");

        verticalLayout_3->addWidget(zeroGyroButton);

        label = new QLabel(tab_3);
        label->setObjectName("label");
        label->setFont(font1);
        label->setAlignment(Qt::AlignmentFlag::AlignCenter);

        verticalLayout_3->addWidget(label);

        ubxVersionButton = new QPushButton(tab_3);
        ubxVersionButton->setObjectName("ubxVersionButton");

        verticalLayout_3->addWidget(ubxVersionButton);

        ubxNavSatButton = new QPushButton(tab_3);
        ubxNavSatButton->setObjectName("ubxNavSatButton");

        verticalLayout_3->addWidget(ubxNavSatButton);

        ubxSolButton = new QPushButton(tab_3);
        ubxSolButton->setObjectName("ubxSolButton");

        verticalLayout_3->addWidget(ubxSolButton);

        ubxRelPosNedButton = new QPushButton(tab_3);
        ubxRelPosNedButton->setObjectName("ubxRelPosNedButton");

        verticalLayout_3->addWidget(ubxRelPosNedButton);

        ubxCfgGnssButton = new QPushButton(tab_3);
        ubxCfgGnssButton->setObjectName("ubxCfgGnssButton");

        verticalLayout_3->addWidget(ubxCfgGnssButton);

        label_5 = new QLabel(tab_3);
        label_5->setObjectName("label_5");
        label_5->setFont(font1);
        label_5->setAlignment(Qt::AlignmentFlag::AlignCenter);

        verticalLayout_3->addWidget(label_5);

        uwbListAnchorsButton = new QPushButton(tab_3);
        uwbListAnchorsButton->setObjectName("uwbListAnchorsButton");

        verticalLayout_3->addWidget(uwbListAnchorsButton);

        uwbResetPosButton = new QPushButton(tab_3);
        uwbResetPosButton->setObjectName("uwbResetPosButton");

        verticalLayout_3->addWidget(uwbResetPosButton);

        uwbUptimeButton = new QPushButton(tab_3);
        uwbUptimeButton->setObjectName("uwbUptimeButton");

        verticalLayout_3->addWidget(uwbUptimeButton);

        uwbRebootButton = new QPushButton(tab_3);
        uwbRebootButton->setObjectName("uwbRebootButton");

        verticalLayout_3->addWidget(uwbRebootButton);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        verticalLayout_3->addItem(verticalSpacer_2);


        horizontalLayout_3->addLayout(verticalLayout_3);


        verticalLayout_10->addLayout(horizontalLayout_3);

        horizontalLayout_12 = new QHBoxLayout();
        horizontalLayout_12->setObjectName("horizontalLayout_12");
        terminalEdit = new HistoryLineEdit(tab_3);
        terminalEdit->setObjectName("terminalEdit");

        horizontalLayout_12->addWidget(terminalEdit);

        terminalSendButton = new QPushButton(tab_3);
        terminalSendButton->setObjectName("terminalSendButton");
        QIcon icon2;
        icon2.addFile(QString::fromUtf8(":/models/Icons/Send File-96.png"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        terminalSendButton->setIcon(icon2);

        horizontalLayout_12->addWidget(terminalSendButton);

        terminalClearButton = new QPushButton(tab_3);
        terminalClearButton->setObjectName("terminalClearButton");
        QIcon icon3;
        icon3.addFile(QString::fromUtf8(":/models/Icons/Delete2-96.png"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        terminalClearButton->setIcon(icon3);

        horizontalLayout_12->addWidget(terminalClearButton);


        verticalLayout_10->addLayout(horizontalLayout_12);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName("horizontalLayout_2");
        label_4 = new QLabel(tab_3);
        label_4->setObjectName("label_4");

        horizontalLayout_2->addWidget(label_4);

        terminalEditVesc = new HistoryLineEdit(tab_3);
        terminalEditVesc->setObjectName("terminalEditVesc");

        horizontalLayout_2->addWidget(terminalEditVesc);

        terminalSendVescButton = new QPushButton(tab_3);
        terminalSendVescButton->setObjectName("terminalSendVescButton");
        terminalSendVescButton->setIcon(icon2);

        horizontalLayout_2->addWidget(terminalSendVescButton);


        verticalLayout_10->addLayout(horizontalLayout_2);

        QIcon icon4;
        icon4.addFile(QString::fromUtf8(":/models/Icons/Console-96.png"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        tabWidget->addTab(tab_3, icon4, QString());
        tab_4 = new QWidget();
        tab_4->setObjectName("tab_4");
        verticalLayout_9 = new QVBoxLayout(tab_4);
        verticalLayout_9->setObjectName("verticalLayout_9");
        groupBox_3 = new QGroupBox(tab_4);
        groupBox_3->setObjectName("groupBox_3");
        verticalLayout_7 = new QVBoxLayout(groupBox_3);
        verticalLayout_7->setObjectName("verticalLayout_7");
        gridLayout1 = new QGridLayout();
        gridLayout1->setObjectName("gridLayout1");
        servoDirectNumber = new QLCDNumber(groupBox_3);
        servoDirectNumber->setObjectName("servoDirectNumber");
        servoDirectNumber->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);
        servoDirectNumber->setProperty("value", QVariant(0.500000000000000));

        gridLayout1->addWidget(servoDirectNumber, 0, 2, 1, 1);

        label_3 = new QLabel(groupBox_3);
        label_3->setObjectName("label_3");

        gridLayout1->addWidget(label_3, 1, 0, 1, 1);

        servoMappedSlider = new QSlider(groupBox_3);
        servoMappedSlider->setObjectName("servoMappedSlider");
        servoMappedSlider->setMinimum(-1000);
        servoMappedSlider->setMaximum(1000);
        servoMappedSlider->setSingleStep(10);
        servoMappedSlider->setValue(0);
        servoMappedSlider->setOrientation(Qt::Orientation::Horizontal);

        gridLayout1->addWidget(servoMappedSlider, 1, 1, 1, 1);

        label_2 = new QLabel(groupBox_3);
        label_2->setObjectName("label_2");

        gridLayout1->addWidget(label_2, 0, 0, 1, 1);

        servoDirectSlider = new QSlider(groupBox_3);
        servoDirectSlider->setObjectName("servoDirectSlider");
        servoDirectSlider->setMinimum(0);
        servoDirectSlider->setMaximum(1000);
        servoDirectSlider->setSingleStep(10);
        servoDirectSlider->setValue(500);
        servoDirectSlider->setOrientation(Qt::Orientation::Horizontal);

        gridLayout1->addWidget(servoDirectSlider, 0, 1, 1, 1);

        servoMappedNumber = new QLCDNumber(groupBox_3);
        servoMappedNumber->setObjectName("servoMappedNumber");
        servoMappedNumber->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);
        servoMappedNumber->setProperty("value", QVariant(0.000000000000000));

        gridLayout1->addWidget(servoMappedNumber, 1, 2, 1, 1);

        label_9 = new QLabel(groupBox_3);
        label_9->setObjectName("label_9");

        gridLayout1->addWidget(label_9, 2, 0, 1, 1);

        ioBoardPwmSlider = new QSlider(groupBox_3);
        ioBoardPwmSlider->setObjectName("ioBoardPwmSlider");
        ioBoardPwmSlider->setMaximum(1000);
        ioBoardPwmSlider->setSingleStep(10);
        ioBoardPwmSlider->setValue(500);
        ioBoardPwmSlider->setOrientation(Qt::Orientation::Horizontal);

        gridLayout1->addWidget(ioBoardPwmSlider, 2, 1, 1, 1);

        ioBoardPwmNumber = new QLCDNumber(groupBox_3);
        ioBoardPwmNumber->setObjectName("ioBoardPwmNumber");
        ioBoardPwmNumber->setSegmentStyle(QLCDNumber::SegmentStyle::Flat);

        gridLayout1->addWidget(ioBoardPwmNumber, 2, 2, 1, 1);


        verticalLayout_7->addLayout(gridLayout1);


        verticalLayout_9->addWidget(groupBox_3);

        groupBox_2 = new QGroupBox(tab_4);
        groupBox_2->setObjectName("groupBox_2");
        QSizePolicy sizePolicy(QSizePolicy::Policy::Preferred, QSizePolicy::Policy::MinimumExpanding);
        sizePolicy.setHorizontalStretch(0);
        sizePolicy.setVerticalStretch(0);
        sizePolicy.setHeightForWidth(groupBox_2->sizePolicy().hasHeightForWidth());
        groupBox_2->setSizePolicy(sizePolicy);
        gridLayout_4 = new QGridLayout(groupBox_2);
        gridLayout_4->setObjectName("gridLayout_4");
        magCal = new MagCal(groupBox_2);
        magCal->setObjectName("magCal");

        gridLayout_4->addWidget(magCal, 0, 0, 1, 1);


        verticalLayout_9->addWidget(groupBox_2);

        QIcon icon5;
        icon5.addFile(QString::fromUtf8(":/models/Icons/Magnet-96.png"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        tabWidget->addTab(tab_4, icon5, QString());
        tab_6 = new QWidget();
        tab_6->setObjectName("tab_6");
        verticalLayout_8 = new QVBoxLayout(tab_6);
        verticalLayout_8->setObjectName("verticalLayout_8");
        nmeaWidget = new NmeaWidget(tab_6);
        nmeaWidget->setObjectName("nmeaWidget");

        verticalLayout_8->addWidget(nmeaWidget);

        QIcon icon6;
        icon6.addFile(QString::fromUtf8(":/models/Icons/Satellite-96.png"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        tabWidget->addTab(tab_6, icon6, QString());
        tab_5 = new QWidget();
        tab_5->setObjectName("tab_5");
        verticalLayout_6 = new QVBoxLayout(tab_5);
        verticalLayout_6->setObjectName("verticalLayout_6");
        groupBox_4 = new QGroupBox(tab_5);
        groupBox_4->setObjectName("groupBox_4");
        gridLayout_3 = new QGridLayout(groupBox_4);
        gridLayout_3->setSpacing(3);
        gridLayout_3->setObjectName("gridLayout_3");
        gridLayout_3->setContentsMargins(-1, 6, 6, 6);
        confUseUwbPosBox = new QCheckBox(groupBox_4);
        confUseUwbPosBox->setObjectName("confUseUwbPosBox");

        gridLayout_3->addWidget(confUseUwbPosBox, 6, 2, 1, 1);

        confYawImuGainBox = new QDoubleSpinBox(groupBox_4);
        confYawImuGainBox->setObjectName("confYawImuGainBox");
        confYawImuGainBox->setDecimals(5);
        confYawImuGainBox->setMaximum(1000.000000000000000);
        confYawImuGainBox->setSingleStep(0.010000000000000);

        gridLayout_3->addWidget(confYawImuGainBox, 2, 2, 1, 1);

        confClampImuYawBox = new QCheckBox(groupBox_4);
        confClampImuYawBox->setObjectName("confClampImuYawBox");

        gridLayout_3->addWidget(confClampImuYawBox, 7, 2, 1, 1);

        confMiscDisableMotorBox = new QCheckBox(groupBox_4);
        confMiscDisableMotorBox->setObjectName("confMiscDisableMotorBox");

        gridLayout_3->addWidget(confMiscDisableMotorBox, 7, 0, 1, 1);

        confMotorPoleBox = new QDoubleSpinBox(groupBox_4);
        confMotorPoleBox->setObjectName("confMotorPoleBox");
        confMotorPoleBox->setDecimals(0);
        confMotorPoleBox->setMinimum(2.000000000000000);
        confMotorPoleBox->setMaximum(1000.000000000000000);
        confMotorPoleBox->setSingleStep(2.000000000000000);

        gridLayout_3->addWidget(confMotorPoleBox, 0, 2, 1, 1);

        confMiscSimulateMotorBox = new QCheckBox(groupBox_4);
        confMiscSimulateMotorBox->setObjectName("confMiscSimulateMotorBox");

        gridLayout_3->addWidget(confMiscSimulateMotorBox, 7, 1, 1, 1);

        confServoDGainBox = new QDoubleSpinBox(groupBox_4);
        confServoDGainBox->setObjectName("confServoDGainBox");

        gridLayout_3->addWidget(confServoDGainBox, 3, 2, 1, 1);

        confSteeringRampBox = new QDoubleSpinBox(groupBox_4);
        confSteeringRampBox->setObjectName("confSteeringRampBox");
        confSteeringRampBox->setDecimals(3);
        confSteeringRampBox->setMaximum(1000.000000000000000);
        confSteeringRampBox->setSingleStep(0.100000000000000);

        gridLayout_3->addWidget(confSteeringRampBox, 1, 1, 1, 1);

        confSensorCentreBox = new QDoubleSpinBox(groupBox_4);
        confSensorCentreBox->setObjectName("confSensorCentreBox");
        confSensorCentreBox->setDecimals(0);
        confSensorCentreBox->setMinimum(0.000000000000000);
        confSensorCentreBox->setMaximum(1024.000000000000000);

        gridLayout_3->addWidget(confSensorCentreBox, 4, 0, 1, 1);

        confWheelDiamBox = new QDoubleSpinBox(groupBox_4);
        confWheelDiamBox->setObjectName("confWheelDiamBox");
        confWheelDiamBox->setDecimals(3);
        confWheelDiamBox->setMaximum(1000.000000000000000);
        confWheelDiamBox->setSingleStep(0.100000000000000);

        gridLayout_3->addWidget(confWheelDiamBox, 0, 1, 1, 1);

        confOdometryYawBox = new QCheckBox(groupBox_4);
        confOdometryYawBox->setObjectName("confOdometryYawBox");

        gridLayout_3->addWidget(confOdometryYawBox, 6, 0, 1, 2);

        confDegreeIntervalBox = new QDoubleSpinBox(groupBox_4);
        confDegreeIntervalBox->setObjectName("confDegreeIntervalBox");

        gridLayout_3->addWidget(confDegreeIntervalBox, 4, 2, 1, 1);

        confServoRangeBox = new QDoubleSpinBox(groupBox_4);
        confServoRangeBox->setObjectName("confServoRangeBox");
        confServoRangeBox->setDecimals(4);
        confServoRangeBox->setMinimum(-1.000000000000000);
        confServoRangeBox->setMaximum(1.000000000000000);
        confServoRangeBox->setSingleStep(0.010000000000000);

        gridLayout_3->addWidget(confServoRangeBox, 2, 1, 1, 1);

        confTurnRadBox = new QDoubleSpinBox(groupBox_4);
        confTurnRadBox->setObjectName("confTurnRadBox");
        confTurnRadBox->setDecimals(3);
        confTurnRadBox->setMaximum(1000.000000000000000);
        confTurnRadBox->setSingleStep(0.100000000000000);

        gridLayout_3->addWidget(confTurnRadBox, 1, 0, 1, 1);

        confGearRatioBox = new QDoubleSpinBox(groupBox_4);
        confGearRatioBox->setObjectName("confGearRatioBox");
        confGearRatioBox->setDecimals(3);
        confGearRatioBox->setMinimum(-1000.000000000000000);
        confGearRatioBox->setMaximum(1000.000000000000000);
        confGearRatioBox->setSingleStep(0.100000000000000);

        gridLayout_3->addWidget(confGearRatioBox, 0, 0, 1, 1);

        confAxisDistanceBox = new QDoubleSpinBox(groupBox_4);
        confAxisDistanceBox->setObjectName("confAxisDistanceBox");
        confAxisDistanceBox->setDecimals(3);
        confAxisDistanceBox->setMaximum(1000.000000000000000);
        confAxisDistanceBox->setSingleStep(0.100000000000000);

        gridLayout_3->addWidget(confAxisDistanceBox, 1, 2, 1, 1);

        confServoCenterBox = new QDoubleSpinBox(groupBox_4);
        confServoCenterBox->setObjectName("confServoCenterBox");
        confServoCenterBox->setDecimals(4);
        confServoCenterBox->setMaximum(1.000000000000000);
        confServoCenterBox->setSingleStep(0.010000000000000);

        gridLayout_3->addWidget(confServoCenterBox, 2, 0, 1, 1);

        confServoIGainBox = new QDoubleSpinBox(groupBox_4);
        confServoIGainBox->setObjectName("confServoIGainBox");

        gridLayout_3->addWidget(confServoIGainBox, 3, 1, 1, 1);

        confServoPGainBox = new QDoubleSpinBox(groupBox_4);
        confServoPGainBox->setObjectName("confServoPGainBox");

        gridLayout_3->addWidget(confServoPGainBox, 3, 0, 1, 1);

        confSensorIntervalBox = new QDoubleSpinBox(groupBox_4);
        confSensorIntervalBox->setObjectName("confSensorIntervalBox");
        confSensorIntervalBox->setDecimals(0);
        confSensorIntervalBox->setMinimum(0.000000000000000);
        confSensorIntervalBox->setMaximum(1024.000000000000000);

        gridLayout_3->addWidget(confSensorIntervalBox, 4, 1, 1, 1);

        confDeadBandBox = new QDoubleSpinBox(groupBox_4);
        confDeadBandBox->setObjectName("confDeadBandBox");
        confDeadBandBox->setDecimals(3);

        gridLayout_3->addWidget(confDeadBandBox, 5, 0, 1, 1);

        confTimeLimitBox = new QDoubleSpinBox(groupBox_4);
        confTimeLimitBox->setObjectName("confTimeLimitBox");

        gridLayout_3->addWidget(confTimeLimitBox, 5, 1, 1, 1);


        verticalLayout_6->addWidget(groupBox_4);

        confCommonWidget = new ConfCommonWidget(tab_5);
        confCommonWidget->setObjectName("confCommonWidget");
        QSizePolicy sizePolicy1(QSizePolicy::Policy::Preferred, QSizePolicy::Policy::Expanding);
        sizePolicy1.setHorizontalStretch(0);
        sizePolicy1.setVerticalStretch(0);
        sizePolicy1.setHeightForWidth(confCommonWidget->sizePolicy().hasHeightForWidth());
        confCommonWidget->setSizePolicy(sizePolicy1);

        verticalLayout_6->addWidget(confCommonWidget);

        horizontalLayout_7 = new QHBoxLayout();
        horizontalLayout_7->setObjectName("horizontalLayout_7");
        horizontalSpacer_3 = new QSpacerItem(40, 20, QSizePolicy::Policy::Expanding, QSizePolicy::Policy::Minimum);

        horizontalLayout_7->addItem(horizontalSpacer_3);

        confReadButton = new QPushButton(tab_5);
        confReadButton->setObjectName("confReadButton");
        QIcon icon7;
        icon7.addFile(QString::fromUtf8(":/models/Icons/Upload-96.png"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        confReadButton->setIcon(icon7);

        horizontalLayout_7->addWidget(confReadButton);

        confReadDefaultButton = new QPushButton(tab_5);
        confReadDefaultButton->setObjectName("confReadDefaultButton");
        QIcon icon8;
        icon8.addFile(QString::fromUtf8(":/models/Icons/HDD-96.png"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        confReadDefaultButton->setIcon(icon8);

        horizontalLayout_7->addWidget(confReadDefaultButton);

        confWriteButton = new QPushButton(tab_5);
        confWriteButton->setObjectName("confWriteButton");
        QIcon icon9;
        icon9.addFile(QString::fromUtf8(":/models/Icons/Download-96.png"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        confWriteButton->setIcon(icon9);

        horizontalLayout_7->addWidget(confWriteButton);


        verticalLayout_6->addLayout(horizontalLayout_7);

        QIcon icon10;
        icon10.addFile(QString::fromUtf8(":/models/Icons/Settings-96.png"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        tabWidget->addTab(tab_5, icon10, QString());
        tab_7 = new QWidget();
        tab_7->setObjectName("tab_7");
        QIcon icon11(QIcon::fromTheme("network-wired"));
        tabWidget->addTab(tab_7, icon11, QString());

        verticalLayout->addWidget(tabWidget);

        mcFaultLabel = new QLabel(CarInterface);
        mcFaultLabel->setObjectName("mcFaultLabel");
        mcFaultLabel->setAlignment(Qt::AlignmentFlag::AlignCenter);

        verticalLayout->addWidget(mcFaultLabel);

        horizontalLayout1 = new QHBoxLayout();
        horizontalLayout1->setObjectName("horizontalLayout1");
        fwLabel = new QLabel(CarInterface);
        fwLabel->setObjectName("fwLabel");

        horizontalLayout1->addWidget(fwLabel);

        batteryBar = new QProgressBar(CarInterface);
        batteryBar->setObjectName("batteryBar");
        batteryBar->setValue(0);

        horizontalLayout1->addWidget(batteryBar);

        tempFetBar = new QProgressBar(CarInterface);
        tempFetBar->setObjectName("tempFetBar");
        tempFetBar->setMaximum(110);
        tempFetBar->setValue(24);

        horizontalLayout1->addWidget(tempFetBar);

        clockLabel = new QLabel(CarInterface);
        clockLabel->setObjectName("clockLabel");

        horizontalLayout1->addWidget(clockLabel);


        verticalLayout->addLayout(horizontalLayout1);


        horizontalLayout_5->addLayout(verticalLayout);

        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setObjectName("verticalLayout_2");
        idBox = new QSpinBox(CarInterface);
        idBox->setObjectName("idBox");
        idBox->setMaximum(255);

        verticalLayout_2->addWidget(idBox);

        pollBox = new QCheckBox(CarInterface);
        pollBox->setObjectName("pollBox");

        verticalLayout_2->addWidget(pollBox);

        keyboardControlBox = new QCheckBox(CarInterface);
        keyboardControlBox->setObjectName("keyboardControlBox");

        verticalLayout_2->addWidget(keyboardControlBox);

        autopilotBox = new QCheckBox(CarInterface);
        autopilotBox->setObjectName("autopilotBox");

        verticalLayout_2->addWidget(autopilotBox);

        updateRouteFromMapBox = new QCheckBox(CarInterface);
        updateRouteFromMapBox->setObjectName("updateRouteFromMapBox");

        verticalLayout_2->addWidget(updateRouteFromMapBox);

        vescToolTcpBox = new QCheckBox(CarInterface);
        vescToolTcpBox->setObjectName("vescToolTcpBox");
        vescToolTcpBox->setEnabled(true);

        verticalLayout_2->addWidget(vescToolTcpBox);

        clearRouteButton = new QPushButton(CarInterface);
        clearRouteButton->setObjectName("clearRouteButton");
        clearRouteButton->setIcon(icon3);

        verticalLayout_2->addWidget(clearRouteButton);

        setClockButton = new QPushButton(CarInterface);
        setClockButton->setObjectName("setClockButton");
        QIcon icon12;
        icon12.addFile(QString::fromUtf8(":/models/Icons/Clock-96.png"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        setClockButton->setIcon(icon12);

        verticalLayout_2->addWidget(setClockButton);

        setClockPiButton = new QPushButton(CarInterface);
        setClockPiButton->setObjectName("setClockPiButton");
        setClockPiButton->setIcon(icon12);

        verticalLayout_2->addWidget(setClockPiButton);

        rebootPiButton = new QPushButton(CarInterface);
        rebootPiButton->setObjectName("rebootPiButton");
        QIcon icon13;
        icon13.addFile(QString::fromUtf8(":/models/Icons/Restart-96.png"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        rebootPiButton->setIcon(icon13);

        verticalLayout_2->addWidget(rebootPiButton);

        shutdownPiButton = new QPushButton(CarInterface);
        shutdownPiButton->setObjectName("shutdownPiButton");
        QIcon icon14;
        icon14.addFile(QString::fromUtf8(":/models/Icons/Shutdown-96.png"), QSize(), QIcon::Mode::Normal, QIcon::State::Off);
        shutdownPiButton->setIcon(icon14);

        verticalLayout_2->addWidget(shutdownPiButton);

        restartCarClientButton = new QPushButton(CarInterface);
        restartCarClientButton->setObjectName("restartCarClientButton");

        verticalLayout_2->addWidget(restartCarClientButton);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Policy::Minimum, QSizePolicy::Policy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);


        horizontalLayout_5->addLayout(verticalLayout_2);


        retranslateUi(CarInterface);
        QObject::connect(terminalEditVesc, &HistoryLineEdit::returnPressed, terminalSendVescButton, qOverload<>(&QPushButton::click));
        QObject::connect(terminalEdit, &HistoryLineEdit::returnPressed, terminalSendButton, qOverload<>(&QPushButton::click));

        tabWidget->setCurrentIndex(7);


        QMetaObject::connectSlotsByName(CarInterface);
    } // setupUi

    void retranslateUi(QWidget *CarInterface)
    {
        CarInterface->setWindowTitle(QCoreApplication::translate("CarInterface", "Form", nullptr));
        rollBar->setFormat(QCoreApplication::translate("CarInterface", "Roll: %v\302\260", nullptr));
        pitchBar->setFormat(QCoreApplication::translate("CarInterface", "Pitch %v\302\260", nullptr));
        yawBar->setFormat(QCoreApplication::translate("CarInterface", "Yaw: %v\302\260", nullptr));
        speedBar->setFormat(QCoreApplication::translate("CarInterface", "Speed: %v km/h", nullptr));
        label00->setText(QCoreApplication::translate("CarInterface", "Row 0, Col 0", nullptr));
        label01->setText(QCoreApplication::translate("CarInterface", "Row 0, Col 1", nullptr));
        label10->setText(QCoreApplication::translate("CarInterface", "Row 1, Col 0", nullptr));
        label11->setText(QCoreApplication::translate("CarInterface", "Row 1, Col 1", nullptr));
        label20->setText(QCoreApplication::translate("CarInterface", "Row 2, Col 0", nullptr));
        label21->setText(QCoreApplication::translate("CarInterface", "Row 2, Col 1", nullptr));
        label30->setText(QCoreApplication::translate("CarInterface", "Row 3, Col 0", nullptr));
        label31->setText(QCoreApplication::translate("CarInterface", "Row 3, Col 1", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(vehicleStateTab), QString());
#if QT_CONFIG(tooltip)
        tabWidget->setTabToolTip(tabWidget->indexOf(vehicleStateTab), QCoreApplication::translate("CarInterface", "Orientation", nullptr));
#endif // QT_CONFIG(tooltip)
        label_log1->setText(QCoreApplication::translate("CarInterface", "TextLabel", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab), QCoreApplication::translate("CarInterface", "Log", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_2), QString());
#if QT_CONFIG(tooltip)
        tabWidget->setTabToolTip(tabWidget->indexOf(tab_2), QCoreApplication::translate("CarInterface", "IMU realtime", nullptr));
#endif // QT_CONFIG(tooltip)
        label_8->setText(QCoreApplication::translate("CarInterface", "Car", nullptr));
#if QT_CONFIG(tooltip)
        zeroGyroButton->setToolTip(QCoreApplication::translate("CarInterface", "Zero gyro on car. Before doing this, make sure that the car is standing still.", nullptr));
#endif // QT_CONFIG(tooltip)
        zeroGyroButton->setText(QCoreApplication::translate("CarInterface", "Zero Gyro", nullptr));
        label->setText(QCoreApplication::translate("CarInterface", "Ublox", nullptr));
        ubxVersionButton->setText(QCoreApplication::translate("CarInterface", "Version", nullptr));
        ubxNavSatButton->setText(QCoreApplication::translate("CarInterface", "NAV_SAT", nullptr));
        ubxSolButton->setText(QCoreApplication::translate("CarInterface", "SOL", nullptr));
        ubxRelPosNedButton->setText(QCoreApplication::translate("CarInterface", "RELPOSNED", nullptr));
        ubxCfgGnssButton->setText(QCoreApplication::translate("CarInterface", "CFG_GNSS", nullptr));
        label_5->setText(QCoreApplication::translate("CarInterface", "UWB", nullptr));
#if QT_CONFIG(tooltip)
        uwbListAnchorsButton->setToolTip(QCoreApplication::translate("CarInterface", "List UWB anchors.", nullptr));
#endif // QT_CONFIG(tooltip)
        uwbListAnchorsButton->setText(QCoreApplication::translate("CarInterface", "List Anchors", nullptr));
#if QT_CONFIG(tooltip)
        uwbResetPosButton->setToolTip(QCoreApplication::translate("CarInterface", "Set UWB position to RTK position.", nullptr));
#endif // QT_CONFIG(tooltip)
        uwbResetPosButton->setText(QCoreApplication::translate("CarInterface", "Reset Pos", nullptr));
#if QT_CONFIG(tooltip)
        uwbUptimeButton->setToolTip(QCoreApplication::translate("CarInterface", "Print how long the UWB module has been running.", nullptr));
#endif // QT_CONFIG(tooltip)
        uwbUptimeButton->setText(QCoreApplication::translate("CarInterface", "Uptime", nullptr));
#if QT_CONFIG(tooltip)
        uwbRebootButton->setToolTip(QCoreApplication::translate("CarInterface", "Reboot the UWB module.", nullptr));
#endif // QT_CONFIG(tooltip)
        uwbRebootButton->setText(QCoreApplication::translate("CarInterface", "Reboot", nullptr));
#if QT_CONFIG(tooltip)
        terminalSendButton->setToolTip(QCoreApplication::translate("CarInterface", "Send command to car", nullptr));
#endif // QT_CONFIG(tooltip)
        terminalSendButton->setText(QString());
#if QT_CONFIG(tooltip)
        terminalClearButton->setToolTip(QCoreApplication::translate("CarInterface", "Clear terminal", nullptr));
#endif // QT_CONFIG(tooltip)
        terminalClearButton->setText(QString());
        label_4->setText(QCoreApplication::translate("CarInterface", "VESC", nullptr));
        terminalSendVescButton->setText(QString());
        tabWidget->setTabText(tabWidget->indexOf(tab_3), QString());
#if QT_CONFIG(tooltip)
        tabWidget->setTabToolTip(tabWidget->indexOf(tab_3), QCoreApplication::translate("CarInterface", "Terminal", nullptr));
#endif // QT_CONFIG(tooltip)
        groupBox_3->setTitle(QCoreApplication::translate("CarInterface", "Servo", nullptr));
        label_3->setText(QCoreApplication::translate("CarInterface", "Mapped steering", nullptr));
        label_2->setText(QCoreApplication::translate("CarInterface", "Direct Steering", nullptr));
        label_9->setText(QCoreApplication::translate("CarInterface", "IO Board PWM", nullptr));
        groupBox_2->setTitle(QCoreApplication::translate("CarInterface", "Magnetometer", nullptr));
        tabWidget->setTabText(tabWidget->indexOf(tab_4), QString());
#if QT_CONFIG(tooltip)
        tabWidget->setTabToolTip(tabWidget->indexOf(tab_4), QCoreApplication::translate("CarInterface", "Calibration", nullptr));
#endif // QT_CONFIG(tooltip)
        tabWidget->setTabText(tabWidget->indexOf(tab_6), QString());
#if QT_CONFIG(tooltip)
        tabWidget->setTabToolTip(tabWidget->indexOf(tab_6), QCoreApplication::translate("CarInterface", "GPS", nullptr));
#endif // QT_CONFIG(tooltip)
        groupBox_4->setTitle(QCoreApplication::translate("CarInterface", "Car Settings", nullptr));
        confUseUwbPosBox->setText(QCoreApplication::translate("CarInterface", "Use UWB Position", nullptr));
#if QT_CONFIG(tooltip)
        confYawImuGainBox->setToolTip(QCoreApplication::translate("CarInterface", "IMU gain vs odometry gain. Higer values give the IMU more gain.", nullptr));
#endif // QT_CONFIG(tooltip)
        confYawImuGainBox->setPrefix(QCoreApplication::translate("CarInterface", "Yaw IMU Gain: ", nullptr));
#if QT_CONFIG(tooltip)
        confClampImuYawBox->setToolTip(QCoreApplication::translate("CarInterface", "Clamp IMU yaw when car is stationary", nullptr));
#endif // QT_CONFIG(tooltip)
        confClampImuYawBox->setText(QCoreApplication::translate("CarInterface", "Clamp IMU yaw stat", nullptr));
        confMiscDisableMotorBox->setText(QCoreApplication::translate("CarInterface", "Disable Motor", nullptr));
        confMotorPoleBox->setPrefix(QCoreApplication::translate("CarInterface", "Motor Poles: ", nullptr));
        confMiscSimulateMotorBox->setText(QCoreApplication::translate("CarInterface", "Simulate Motor", nullptr));
        confServoDGainBox->setPrefix(QCoreApplication::translate("CarInterface", "Servo D Gain:", nullptr));
        confSteeringRampBox->setPrefix(QCoreApplication::translate("CarInterface", "Steering Ramp: ", nullptr));
        confSensorCentreBox->setPrefix(QCoreApplication::translate("CarInterface", "Sensor value centre: ", nullptr));
        confSensorCentreBox->setSuffix(QString());
        confWheelDiamBox->setPrefix(QCoreApplication::translate("CarInterface", "Wheel Diameter: ", nullptr));
        confWheelDiamBox->setSuffix(QCoreApplication::translate("CarInterface", " m", nullptr));
        confOdometryYawBox->setText(QCoreApplication::translate("CarInterface", "Use Odometry for YAW correction", nullptr));
        confDegreeIntervalBox->setPrefix(QCoreApplication::translate("CarInterface", "Interval degrees: ", nullptr));
        confDegreeIntervalBox->setSuffix(QCoreApplication::translate("CarInterface", "\302\260", nullptr));
        confServoRangeBox->setPrefix(QCoreApplication::translate("CarInterface", "Servo Range: ", nullptr));
        confTurnRadBox->setPrefix(QCoreApplication::translate("CarInterface", "Turn Radius: ", nullptr));
        confTurnRadBox->setSuffix(QCoreApplication::translate("CarInterface", " m", nullptr));
        confGearRatioBox->setPrefix(QCoreApplication::translate("CarInterface", "Gear Ratio: ", nullptr));
        confAxisDistanceBox->setPrefix(QCoreApplication::translate("CarInterface", "Axis Distance: ", nullptr));
        confAxisDistanceBox->setSuffix(QCoreApplication::translate("CarInterface", " m", nullptr));
        confServoCenterBox->setPrefix(QCoreApplication::translate("CarInterface", "Servo Center: ", nullptr));
        confServoIGainBox->setPrefix(QCoreApplication::translate("CarInterface", "Servo I Gain:", nullptr));
        confServoPGainBox->setPrefix(QCoreApplication::translate("CarInterface", "Servo P Gain:", nullptr));
        confSensorIntervalBox->setPrefix(QCoreApplication::translate("CarInterface", "Interval sensor values: ", nullptr));
        confSensorIntervalBox->setSuffix(QString());
        confDeadBandBox->setPrefix(QCoreApplication::translate("CarInterface", "Dead band: ", nullptr));
        confTimeLimitBox->setPrefix(QCoreApplication::translate("CarInterface", "Heartbeat max time: ", nullptr));
        confTimeLimitBox->setSuffix(QCoreApplication::translate("CarInterface", " s", nullptr));
#if QT_CONFIG(tooltip)
        confReadButton->setToolTip(QCoreApplication::translate("CarInterface", "Read configuration from car", nullptr));
#endif // QT_CONFIG(tooltip)
        confReadButton->setText(QString());
#if QT_CONFIG(tooltip)
        confReadDefaultButton->setToolTip(QCoreApplication::translate("CarInterface", "Read default configuration from car", nullptr));
#endif // QT_CONFIG(tooltip)
        confReadDefaultButton->setText(QString());
#if QT_CONFIG(tooltip)
        confWriteButton->setToolTip(QCoreApplication::translate("CarInterface", "Write configuration to car", nullptr));
#endif // QT_CONFIG(tooltip)
        confWriteButton->setText(QString());
        tabWidget->setTabText(tabWidget->indexOf(tab_5), QString());
#if QT_CONFIG(tooltip)
        tabWidget->setTabToolTip(tabWidget->indexOf(tab_5), QCoreApplication::translate("CarInterface", "Configuration", nullptr));
#endif // QT_CONFIG(tooltip)
        tabWidget->setTabText(tabWidget->indexOf(tab_7), QCoreApplication::translate("CarInterface", "Motor", nullptr));
        mcFaultLabel->setText(QCoreApplication::translate("CarInterface", "Fault code...", nullptr));
        fwLabel->setText(QCoreApplication::translate("CarInterface", "FW x.y", nullptr));
        batteryBar->setFormat(QCoreApplication::translate("CarInterface", "Battery", nullptr));
        tempFetBar->setFormat(QCoreApplication::translate("CarInterface", "MOSFET Temp: %v\302\260C", nullptr));
        clockLabel->setText(QCoreApplication::translate("CarInterface", "00:00:00", nullptr));
        idBox->setPrefix(QCoreApplication::translate("CarInterface", "Car ID: ", nullptr));
        pollBox->setText(QCoreApplication::translate("CarInterface", "Poll Data", nullptr));
        keyboardControlBox->setText(QCoreApplication::translate("CarInterface", "Keyboard Control", nullptr));
        autopilotBox->setText(QCoreApplication::translate("CarInterface", "AutoPilot", nullptr));
#if QT_CONFIG(tooltip)
        updateRouteFromMapBox->setToolTip(QCoreApplication::translate("CarInterface", "Upload route points as soon as they are created on the map", nullptr));
#endif // QT_CONFIG(tooltip)
        updateRouteFromMapBox->setText(QCoreApplication::translate("CarInterface", "Route from map", nullptr));
        vescToolTcpBox->setText(QCoreApplication::translate("CarInterface", "VESC Tool TCP", nullptr));
#if QT_CONFIG(tooltip)
        clearRouteButton->setToolTip(QCoreApplication::translate("CarInterface", "Clear stored route", nullptr));
#endif // QT_CONFIG(tooltip)
        clearRouteButton->setText(QString());
#if QT_CONFIG(tooltip)
        setClockButton->setToolTip(QCoreApplication::translate("CarInterface", "Set clock on STM32 board", nullptr));
#endif // QT_CONFIG(tooltip)
        setClockButton->setText(QCoreApplication::translate("CarInterface", "STM", nullptr));
#if QT_CONFIG(tooltip)
        setClockPiButton->setToolTip(QCoreApplication::translate("CarInterface", "Set clock on Raspberry Pi", nullptr));
#endif // QT_CONFIG(tooltip)
        setClockPiButton->setText(QCoreApplication::translate("CarInterface", "Pi", nullptr));
        rebootPiButton->setText(QCoreApplication::translate("CarInterface", "Reboot PI", nullptr));
        shutdownPiButton->setText(QCoreApplication::translate("CarInterface", "Shutdown PI", nullptr));
        restartCarClientButton->setText(QCoreApplication::translate("CarInterface", "Restart Car Client", nullptr));
    } // retranslateUi

};

namespace Ui {
    class CarInterface: public Ui_CarInterface {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CARINTERFACE_H
