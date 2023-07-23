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

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "ui_rtcmwidget.h"
#include <QSerialPortInfo>
#include <QDebug>
#include <cmath>
#include <QMessageBox>
#include <QFileDialog>
#include <QHostInfo>
#include <QInputDialog>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>
#include <QStringList>
#include <QElapsedTimer>
#include <QNetworkInterface>
#include <QGamepad>
#include <QLoggingCategory>
#include <QtSql>

#include "utility.h"
#include "routemagic.h"
#include "wireguard.h"
#include "attributes_masks.h"
#include "datatypes.h"

namespace {
void stepTowards(double &value, double goal, double step) {
    if (value < goal) {
        if ((value + step) < goal) {
            value += step;
        } else {
            value = goal;
        }
    } else if (value > goal) {
        if ((value - step) > goal) {
            value -= step;
        } else {
            value = goal;
        }
    }
}

void deadband(double &value, double tres, double max) {
    if (fabs(value) < tres) {
        value = 0.0;
    } else {
        double k = max / (max - tres);
        if (value > 0.0) {
            value = k * value + max * (1.0 - k);
        } else {
            value = -(k * -value + max * (1.0 - k));
        }

    }
}
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    mVersion = "0.8";
    mSupportedFirmwares.append(qMakePair(12, 3));
    mSupportedFirmwares.append(qMakePair(20, 1));

    qRegisterMetaType<LocPoint>("LocPoint");

    mTimer = new QTimer(this);
    mTimer->start(ui->pollIntervalBox->value());
    mHeartbeatTimer = new QTimer(this);
    mHeartbeatTimer->start(mHeartbeatMS);
    mStatusLabel = new QLabel(this);
    ui->statusBar->addPermanentWidget(mStatusLabel);
    mStatusInfoTime = 0;
    mPacketInterface = new PacketInterface(this);
    mSerialPort = new QSerialPort(this);
    mThrottle = 0.0;
    mSteering = 0.0;

    qDebug() << "Start";

#ifdef HAS_JOYSTICK

    auto gamepads = QGamepadManager::instance()->connectedGamepads();
    if (gamepads.isEmpty()) {
        qDebug() << "Did not find any connected gamepads";
//        return;
    }

    mJoystick = new QGamepad(*gamepads.begin(), this);

    connect(mJoystick, SIGNAL(buttonChanged(int,bool)),
            this, SLOT(jsButtonChanged(int,bool)));

    connect(mJoystick, &QGamepad::axisLeftXChanged, this, [](double value){
        });
        connect(mJoystick, &QGamepad::axisLeftYChanged, this, [](double value){
        });
        connect(mJoystick, &QGamepad::axisRightXChanged, this, [](double value){
        });
        connect(mJoystick, &QGamepad::axisRightYChanged, this, [](double value){
        });


        static MainWindow *mThis=this;          // Just to be able to get the lambdas to work
        connect(mJoystick, &QGamepad::buttonL1Changed, this, [](bool pressed){
            qDebug() << "Button L1" << pressed;
            mThis->jsButtonChanged(4, pressed);
        });
        connect(mJoystick, &QGamepad::buttonR1Changed, this, [](bool pressed){
            qDebug() << "Button R1" << pressed;
            mThis->jsButtonChanged(5, pressed);
        });
        connect(mJoystick, &QGamepad::buttonL2Changed, this, [](double value){
            qDebug() << "Button L2: " << value;
            mThis->jsButtonChanged(6, value>0);
        });

        connect(mJoystick, &QGamepad::buttonR2Changed, this, [](double value){
            qDebug() << "Button R2: " << value;
            mThis->jsButtonChanged(7, value>0);
        });
    #endif

    mPing = new Ping(this);
    mNmea = new NmeaServer(this);
    mUdpSocket = new QUdpSocket(this);
    mTcpClientMulti = new TcpClientMulti(this);
    mUdpSocket->setSocketOption(QAbstractSocket::LowDelayOption, true);

    mIntersectionTest = new IntersectionTest(this);
    mIntersectionTest->setCars(&mCars);
    mIntersectionTest->setMap(ui->mapWidget);
    mIntersectionTest->setPacketInterface(mPacketInterface);
    connect(ui->nComWidget, SIGNAL(dataRx(ncom_data)),
            mIntersectionTest, SLOT(nComRx(ncom_data)));


#ifdef HAS_LIME_SDR
    mGpsSim = new GpsSim(this);
    mGpsSim->setMap(ui->mapWidget);
#endif

    mKeyUp = false;
    mKeyDown = false;
    mKeyLeft = false;
    mKeyRight = false;

    ui->mapWidget->setRoutePointSpeed(ui->mapRouteSpeedBox->value() / 3.6);
    ui->networkLoggerWidget->setMap(ui->mapWidget);
    ui->networkInterface->setMap(ui->mapWidget);
//    ui->networkInterface->setMap(ui->mapWidgetFarm);
    ui->networkInterface->setPacketInterface(mPacketInterface);
    ui->networkInterface->setCars(&mCars);
    ui->moteWidget->setPacketInterface(mPacketInterface);
    ui->nComWidget->setMap(ui->mapWidget);
//    ui->nComWidget->setMap(ui->mapWidgetFarm);

//    ui->baseStationWidget->setMap(ui->mapWidget);
//    ui->baseStationWidget->setMap(ui->mapWidgetFarm);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    connect(mSerialPort, SIGNAL(readyRead()),
            this, SLOT(serialDataAvailable()));
    connect(mSerialPort, SIGNAL(error(QSerialPort::SerialPortError)),
            this, SLOT(serialPortError(QSerialPort::SerialPortError)));
    connect(mHeartbeatTimer, SIGNAL(timeout()), this, SLOT(sendHeartbeat()));
    connect(mPacketInterface, SIGNAL(dataToSend(QByteArray&)),
            this, SLOT(packetDataToSend(QByteArray&)));
    connect(mPacketInterface, SIGNAL(stateReceived(quint8,CAR_STATE)),
            this, SLOT(stateReceived(quint8,CAR_STATE)));
    connect(mPacketInterface, SIGNAL(mrStateReceived(quint8,MULTIROTOR_STATE)),
            this, SLOT(mrStateReceived(quint8,MULTIROTOR_STATE)));
    connect(ui->mapWidget, SIGNAL(posSet(quint8,LocPoint)),
            this, SLOT(mapPosSet(quint8,LocPoint)));
    connect(mPacketInterface, SIGNAL(ackReceived(quint8,CMD_PACKET,QString)),
            this, SLOT(ackReceived(quint8,CMD_PACKET,QString)));
    connect(ui->rtcmWidget, SIGNAL(rtcmReceived(QByteArray)),
            this, SLOT(rtcmReceived(QByteArray)));
    connect(ui->rtcmWidget, SIGNAL(refPosGet()), this, SLOT(rtcmRefPosGet()));
    connect(mPing, SIGNAL(pingRx(int,QString)), this, SLOT(pingRx(int,QString)));
    connect(mPing, SIGNAL(pingError(QString,QString)), this, SLOT(pingError(QString,QString)));
    connect(mPacketInterface, SIGNAL(enuRefReceived(quint8,double,double,double)),
            this, SLOT(enuRx(quint8,double,double,double)));
    connect(mNmea, SIGNAL(clientGgaRx(int,NmeaServer::nmea_gga_info_t)),
            this, SLOT(nmeaGgaRx(int,NmeaServer::nmea_gga_info_t)));
    connect(ui->mapWidget, SIGNAL(routePointAdded(LocPoint)),
            this, SLOT(routePointAdded(LocPoint)));
    connect(ui->mapWidget, SIGNAL(infoTraceChanged(int)),
            this, SLOT(infoTraceChanged(int)));

    connect(ui->actionAboutQt, SIGNAL(triggered(bool)),
            qApp, SLOT(aboutQt()));

    connect(mTcpClientMulti, &TcpClientMulti::packetRx, [this](QByteArray data) {
        mPacketInterface->processPacket((unsigned char*)data.data(), data.size());
    });

    connect(mTcpClientMulti, &TcpClientMulti::stateChanged, [this](QString msg, bool isError) {
        showStatusInfo(msg, !isError);



        if (isError) {
            qWarning() << "TCP Error:" << msg;
            QMessageBox::warning(this, "TCP Error", msg);
        }
    });

    on_mapCameraWidthBox_valueChanged(ui->mapCameraWidthBox->value());
    on_mapCameraOpacityBox_valueChanged(ui->mapCameraOpacityBox->value());
    if (WireGuard::isWireGuardInstalled()) {
        ui->wgStatusLabel->setText("Status: Ready");
        ui->wgGroupBox->setEnabled(true);
    } else {
        ui->wgStatusLabel->setText("Status: Not installed");
        ui->wgGroupBox->setEnabled(false);
    }

#ifdef HAS_JOYSTICK
    // Connect micronav joystick by default
    bool connectJs = false;

    {
         QGamepad js;
            if (js.isConnected()) {
                    connectJs = true;
        }
    }

    if (connectJs) {
        on_jsConnectButton_clicked();
    }
#endif


#ifdef HAS_SIM_SCEN
    mSimScen = new PageSimScen;
    ui->mainTabWidget->addTab(mSimScen, QIcon(":/models/Icons/Sedan-96.png"), "");
    ui->mainTabWidget->setTabToolTip(ui->mainTabWidget->count() - 1,
                                     "Simulation Scenarios");
#endif


    //Initialize
    on_WgConnectPushButton_clicked();
//    on_tcpConnectButton_clicked();
//    on_carAddButton_clicked();

    if (!QSqlDatabase::drivers().contains("QSQLITE"))
        QMessageBox::critical(
                    this,
                    "Unable to load database",
                    "This demo needs the SQLITE driver"
                    );

    // Initialize the database:
    QSqlError err = initDb();
    if (err.type() != QSqlError::NoError) {
        showError(err);
        return;
    }


    // Create the data model:
    modelFarm = new QSqlRelationalTableModel(ui->farmTable);
//    model->setEditStrategy(QSqlTableModel::OnManualSubmit);
    modelFarm->setEditStrategy(QSqlTableModel::OnFieldChange);
    modelFarm->setTable("locations");

    // Set the localized header captions:
 //   modelFarm->setHeaderData(locationIdx, Qt::Horizontal, tr("Location"));
    modelFarm->setHeaderData(modelFarm->fieldIndex("name"),
                         Qt::Horizontal, tr("Name"));
    modelFarm->setHeaderData(modelFarm->fieldIndex("longitude"),
                         Qt::Horizontal, tr("Longitude"));
    modelFarm->setHeaderData(modelFarm->fieldIndex("latitude"),
                         Qt::Horizontal, tr("Latitude"));
    modelFarm->setHeaderData(modelFarm->fieldIndex("ip"),
                         Qt::Horizontal, tr("ip"));
    modelFarm->setHeaderData(modelFarm->fieldIndex("port"),
                         Qt::Horizontal, tr("port"));
    modelFarm->setHeaderData(modelFarm->fieldIndex("NTRIP"),
                         Qt::Horizontal, tr("NTRIP"));

    // Populate the model:
    if (!modelFarm->select()) {
        showError(modelFarm->lastError());
        return;
    }

    // Set the model and hide the ID column:
    ui->farmTable->setModel(modelFarm);
    //ui.locationTable->setItemDelegate(new BookDelegate(ui.locationTable));
    ui->farmTable->setColumnHidden(modelFarm->fieldIndex("id"), true);
    ui->farmTable->setSelectionMode(QAbstractItemView::SingleSelection);

    QDataWidgetMapper *mapperFarm = new QDataWidgetMapper(this);
    mapperFarm->setModel(modelFarm);
    mapperFarm->addMapping(ui->locationnameEdit, modelFarm->fieldIndex("name"));

    mapperFarm->addMapping(this->findChild<QLineEdit*>("ntripServerEdit"), modelFarm->fieldIndex("ip"));
    mapperFarm->addMapping(this->findChild<QSpinBox*>("ntripPortBox"), modelFarm->fieldIndex("port"));
    mapperFarm->addMapping(this->findChild<QLineEdit*>("ntripUserEdit"), modelFarm->fieldIndex("user"));
    mapperFarm->addMapping(this->findChild<QCheckBox*>("ntripBox"), modelFarm->fieldIndex("NTRIP"));
    mapperFarm->addMapping(this->findChild<QLineEdit*>("ntripPasswordEdit"), modelFarm->fieldIndex("password"));
    mapperFarm->addMapping(this->findChild<QLineEdit*>("ntripStreamEdit"), modelFarm->fieldIndex("stream"));
    mapperFarm->addMapping(this->findChild<QDoubleSpinBox*>("refSendLatBox"), modelFarm->fieldIndex("latitude"));
    mapperFarm->addMapping(this->findChild<QDoubleSpinBox*>("refSendLonBox"), modelFarm->fieldIndex("longitude"));

//    mapperFarm->addMapping(ui->streamFarmBasestationEdit, modelFarm->fieldIndex("stream"));

    connect(ui->farmTable->selectionModel(),
            &QItemSelectionModel::currentRowChanged,
            mapperFarm
            ,
            &QDataWidgetMapper::setCurrentModelIndex
            );

    ui->farmTable->setCurrentIndex(modelFarm->index(0, 0));


    // Create the data model:
    modelField = new QSqlRelationalTableModel(ui->fieldTable);
    modelField->setEditStrategy(QSqlTableModel::OnManualSubmit);
//    modelField->setEditStrategy(QSqlTableModel::OnFieldChange);
    modelField->setTable("fields");

    // Remember the indexes of the columns:
    locationIdx = modelField->fieldIndex("location");

    // Set the relations to the other database tables:
    modelField->setRelation(locationIdx, QSqlRelation("locations", "id", "name"));

    // Set the localized header captions:
    modelField->setHeaderData(locationIdx, Qt::Horizontal, tr("Location"));
    modelField->setHeaderData(modelField->fieldIndex("name"),
                         Qt::Horizontal, tr("Field name"));
    // Populate the model:
    if (!modelField->select()) {
        showError(modelField->lastError());
        return;
    }

    // Set the model and hide the ID column:
    ui->fieldTable->setModel(modelField);
    ui->fieldTable->setColumnHidden(modelField->fieldIndex("id"), true);
    ui->fieldTable->setSelectionMode(QAbstractItemView::SingleSelection);

    QTableView* tableShort=ui->fieldTable;
    QSqlRelationalTableModel *modelShort=this->modelField;

    MapWidget *mapShort=ui->mapWidgetFields;
    QLabel *areaLabel=ui->label_area_ha;


    QObject::connect(ui->fieldTable->selectionModel(), &QItemSelectionModel::selectionChanged,
                     [tableShort, modelShort,mapShort,areaLabel](const QItemSelection& selected, const QItemSelection&)
                     {
                         QModelIndexList selectedIndexes = selected.indexes();
                         if (selectedIndexes.isEmpty()) {
                             qDebug() << "No selection";
                             return;
                         }

                         int row = selectedIndexes.first().row();
                         qDebug() << "Selected Row: " << row;

                         // Retrieve the data of the selected row if needed
                         QString name = modelShort->data(modelShort->index(row, 1)).toString();
                         QString xmlText = modelShort->data(modelShort->index(row, 4)).toString();
                         qDebug() << "Name: " << name << ", xml: " << xmlText;
                         mapShort->clearRoute();
                         QXmlStreamReader xmlData(xmlText);
                         bool routes_found=utility::loadXMLRoute(&xmlData, mapShort);
                         QList<LocPoint> route=mapShort->getRoute();
                         double area=RouteMagic::getArea(route);
                         areaLabel->setText(QString::number(area));
                     });




    // Initialize the Author combo box:
    ui->locationEdit->setModel(modelField->relationModel(locationIdx));
    ui->locationEdit->setModelColumn(
                modelField->relationModel(locationIdx)->fieldIndex("name"));


    QDataWidgetMapper *mapperField = new QDataWidgetMapper(this);
    mapperField->setModel(modelField);
    mapperField->addMapping(ui->fieldnameEdit, modelField->fieldIndex("name"));
    mapperField->addMapping(ui->locationEdit, locationIdx);
//    mapperField->addMapping(ui->locationEdit, locationIdx);
//    mapperFarm->addMapping(this->findChild<QLineEdit*>("ntripPasswordEdit"), modelFarm->fieldIndex("password"));

    connect(ui->fieldTable->selectionModel(),
            &QItemSelectionModel::currentRowChanged,
            mapperField,
            &QDataWidgetMapper::setCurrentModelIndex
            );

    ui->fieldTable->setCurrentIndex(modelField->index(0, 0));

    // Initialize the Author combo box:
    ui->locationEdit->setModel(modelField->relationModel(locationIdx));
    ui->locationEdit->setModelColumn(
                modelField->relationModel(locationIdx)->fieldIndex("name"));

    // Create the data model:
    modelVehicle = new QSqlRelationalTableModel(ui->vehicleTable);
//    model->setEditStrategy(QSqlTableModel::OnManualSubmit);
    modelVehicle->setEditStrategy(QSqlTableModel::OnFieldChange);
    modelVehicle->setTable("vehicles");

    // Set the localized header captions:
 //   modelFarm->setHeaderData(locationIdx, Qt::Horizontal, tr("Location"));
    modelVehicle->setHeaderData(modelVehicle->fieldIndex("name"),
                         Qt::Horizontal, tr("Name"));
    modelVehicle->setHeaderData(modelVehicle->fieldIndex("ip"),
                         Qt::Horizontal, tr("IP"));
    modelVehicle->setHeaderData(modelVehicle->fieldIndex("port"),
                         Qt::Horizontal, tr("port"));
    modelVehicle->setHeaderData(modelVehicle->fieldIndex("steering_type"),
                         Qt::Horizontal, tr("Steering type"));
    modelVehicle->setHeaderData(modelVehicle->fieldIndex("length"),
                         Qt::Horizontal, tr("Length"));

    // Populate the model:
    if (!modelVehicle->select()) {
        showError(modelVehicle->lastError());
        return;
    }

    // Set the model and hide the ID column:
    ui->vehicleTable->setModel(modelVehicle);
    //ui.locationTable->setItemDelegate(new BookDelegate(ui.locationTable));
    ui->vehicleTable->setColumnHidden(modelVehicle->fieldIndex("id"), true);
    ui->vehicleTable->setSelectionMode(QAbstractItemView::ExtendedSelection);

    QDataWidgetMapper *mapperVehicle = new QDataWidgetMapper(this);
    mapperVehicle->setModel(modelVehicle);
    mapperVehicle->addMapping(ui->namevehicleEdit, modelVehicle->fieldIndex("name"));
    mapperVehicle->addMapping(ui->ipvehicleEdit, modelVehicle->fieldIndex("ip"));
    mapperVehicle->addMapping(ui->portvehicleEdit, modelVehicle->fieldIndex("port"));
    mapperVehicle->addMapping(ui->lengthvehicleEdit, modelVehicle->fieldIndex("length"));
    mapperVehicle->addMapping(ui->widthvehicleEdit, modelVehicle->fieldIndex("width"));

    connect(ui->vehicleTable->selectionModel(),
            &QItemSelectionModel::currentRowChanged,
            mapperVehicle
            ,
            &QDataWidgetMapper::setCurrentModelIndex
            );


    QTableView* tableShortVeh=ui->vehicleTable;
    QSqlRelationalTableModel *modelShortVeh=this->modelVehicle;
/*
    QObject::connect(ui->vehicleTable->selectionModel(), &QItemSelectionModel::selectionChanged,
                     [tableShortVeh, modelShortVeh](const QItemSelection& selected, const QItemSelection&)
                     {
                         QModelIndexList selectedIndexes = selected.indexes();
                         if (selectedIndexes.isEmpty()) {
                             qDebug() << "No selection";
                             return;
                         }

                         // Using an iterator
                         int oldrow=-1;
                         qDebug() << "Using an iterator:";
                         for (QList<QModelIndex>::iterator it = selectedIndexes.begin(); it != selectedIndexes.end(); ++it) {
                             QModelIndex currentrow = *it;
                             int row = currentrow.row();
                             if (row!=oldrow)
                             {
                                 int column = currentrow.column();
                                 qDebug() << "Selected Row: " << row;
                                 qDebug() << "Selected Column: " << column;

                                 // Retrieve the data of the selected row if needed
                                 QString name = modelShortVeh->data(modelShortVeh->index(row, 0)).toString();
                                 QString ip = modelShortVeh->data(modelShortVeh->index(row, 1)).toString();
                                 QString port = modelShortVeh->data(modelShortVeh->index(row, 2)).toString();
                                 qDebug() << "Name: " << name << ", ip: " << ip << ", port: " << port;
                                 oldrow=row;
                             };
                         }

                   });
*/

    ui->vehicleTable->setCurrentIndex(modelVehicle->index(0, 0));

    connect(ui->pushButton_field, &QPushButton::released, this, &MainWindow::handleFieldButton);
    connect(ui->pushButton_location, &QPushButton::released, this, &MainWindow::handleLocationButton);

    ui->fieldTable->installEventFilter(this);
    ui->farmTable->installEventFilter(this);
    ui->vehicleTable->installEventFilter(this);

//    QWidget* pWidget= ui->mainTabWidget->findChild<QWidget *>("tabSettings")->findChild<QTabWidget *>("tabWidgetSettings")->findChild<QWidget *>("tab_19");
//    pWidget->close();

    QTabWidget* pWidget= ui->mainTabWidget->findChild<QWidget *>("tabSettings")->findChild<QTabWidget *>("tabWidgetSettings");
    pWidget->removeTab(3);
    ui->mainTabWidget->removeTab(7);
    ui->mainTabWidget->removeTab(6);
    ui->mainTabWidget->removeTab(5);
    ui->mainTabWidget->removeTab(4);

    //    ui->tab_19->close();
//    ui->tabWidgetSettings->removeTab(4);
//  ui->mainTabWidget->removeTab(5);

    qApp->installEventFilter(this);
}

MainWindow::~MainWindow()
{
    // Remove all vehicles before this window is destroyed to not get segfaults
    // in their destructors.
    while (mCars.size() > 0 || mCopters.size() > 0) {
        QWidget *w = ui->carsWidget->currentWidget();

        if (dynamic_cast<CarInterface*>(w) != NULL) {
            CarInterface *car = (CarInterface*)w;

            ui->carsWidget->removeTab(ui->carsWidget->currentIndex());
            mCars.removeOne(car);
            delete car;
        } else if (dynamic_cast<CopterInterface*>(w) != NULL) {
            CopterInterface *copter = (CopterInterface*)w;

            ui->carsWidget->removeTab(ui->carsWidget->currentIndex());
            mCopters.removeOne(copter);
            delete copter;
        }
    }

    delete ui;
}

bool MainWindow::eventFilter(QObject *object, QEvent *e)
{
    Q_UNUSED(object);

    // Emergency stop on escape
    if (e->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(e);
        if (keyEvent->key() == Qt::Key_Escape) {
            on_stopButton_clicked();
            return true;
        }
    }

    // Handle F10 here as it won't be detected from the camera window otherwise.
    if (e->type() == QEvent::KeyPress) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(e);
        if (keyEvent->key() == Qt::Key_F10) {
            on_actionToggleCameraFullscreen_triggered();
            return true;
        }
    }

/*
    if (e->type() == QEvent::KeyPress || e->type() == QEvent::KeyRelease) {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(e);
        bool isPress = e->type() == QEvent::KeyPress;

        switch(keyEvent->key()) {
        case Qt::Key_Up:
        case Qt::Key_Down:
        case Qt::Key_Left:
        case Qt::Key_Right:
            break;

        default:
            return false;
        }

        switch(keyEvent->key()) {
        case Qt::Key_Up: mKeyUp = isPress; break;
        case Qt::Key_Down: mKeyDown = isPress; break;
        case Qt::Key_Left: mKeyLeft = isPress; break;
        case Qt::Key_Right: mKeyRight = isPress; break;

        default:
            break;
        }

        // Return true to not pass the key event on
        return true;
    }
*/

    if ( object == ui->fieldTable &&  (e->type() == QEvent::HoverLeave )  ) {
        qDebug() << "OUT";
    }
    if ( object->objectName() == "fieldTable")
    {
        switch (e->type())
        {
            case QEvent::KeyPress:
                {
                    QKeyEvent *keyEvent = static_cast<QKeyEvent *>(e);
//                    qDebug("Ate key press (field) %d", keyEvent->key());
                    switch ( keyEvent->key() )
                    {
                        case Qt::Key_Up:
                            qDebug() << "UP";
                            break;
                        case Qt::Key_Down:
                            qDebug() << "DOWN";
                            break;
                        case Qt::Key_Left:
                            qDebug() << "LEFT";
                            break;
                        case Qt::Key_Right:
                            qDebug() << "RIGHT";
                            break;
                    case Qt::Key_Delete:
                        qDebug() << "DELETE";
       /*                 QModelIndexList selection = object->selectionModel()->selectedRows();

                        // Multiple rows can be selected
                        for(int i=0; i< selection.count(); i++)
                        {
                            QModelIndex index = selection.at(i);
                            qDebug() << index.row();
                        }
                        */
                        break;
                    }
                    return true;
                }
            default:
                {
                    //         qDebug() << object;
                    //         qDebug() << event;
                     return QObject::eventFilter(object, e);
                }
        }
    }
    if ( object->objectName() == "locationTable")
    {
        switch (e->type())
        {
            case QEvent::KeyPress:
                {
                    QKeyEvent *keyEvent = static_cast<QKeyEvent *>(e);
                    qDebug("Ate key press (location) %d", keyEvent->key());
                    return true;
                }
            default:
                {
                    //         qDebug() << object;
                    //         qDebug() << e;
                     return QObject::eventFilter(object, e);
                }
        }
    }

    /*
    ////////////////////////////////////
    switch ( e->key() )
    {
        case Qt::Key_Up:
            qDebug() << "UP";
            break;
        case Qt::Key_Down:
            qDebug() << "DOWN";
            break;
        case Qt::Key_Left:
            qDebug() << "LEFT";
            break;
        case Qt::Key_Right:
            qDebug() << "RIGHT";
            break;
    default:
            qDebug() << e->key() << Qt::endl;
            break;
    }
/////////////////
    */
    // false means it should be send to target also. as in , we dont remove it.
    // if you return true , you will take the event and widget never sees it so be carefull with that.

#ifdef HAS_JOYSTICK
    if (mJoystick->isConnected()) {
        return false;
    }
#endif

    if (ui->throttleOffButton->isChecked()) {
        return false;
    }


    return false;
}

void MainWindow::addCar(int id, QString name, bool pollData)
{
    CarInterface *car = new CarInterface(this);
    mCars.append(car);
    car->setID(id);
    ui->carsWidget->addTab(car, name);
    car->setMap(ui->mapWidget);
    car->setPacketInterface(mPacketInterface);
    car->setPollData(pollData);
    connect(car, SIGNAL(showStatusInfo(QString,bool)), this, SLOT(showStatusInfo(QString,bool)));
}

void MainWindow::connectJoystick(QString dev)
{
}

void MainWindow::addTcpConnection(QString ip, int port)
{
    mTcpClientMulti->addConnection(ip, port);
}

void MainWindow::setNetworkTcpEnabled(bool enabled, int port)
{
    ui->networkInterface->setTcpEnabled(enabled, port);
}

void MainWindow::setNetworkUdpEnabled(bool enabled, int port)
{
    ui->networkInterface->setUdpEnabled(enabled, port);
}

MapWidget *MainWindow::map()
{
    return ui->mapWidget;
}

void MainWindow::serialDataAvailable()
{
    while (mSerialPort->bytesAvailable() > 0) {
        QByteArray data = mSerialPort->readAll();
        mPacketInterface->processData(data);
    }
}

void MainWindow::serialPortError(QSerialPort::SerialPortError error)
{
    QString message;
    switch (error) {
    case QSerialPort::NoError:
        break;

    default:
        message = "Serial port error: " + mSerialPort->errorString();
        break;
    }

    if(!message.isEmpty()) {
        showStatusInfo(message, false);

        if(mSerialPort->isOpen()) {
            mSerialPort->close();
        }
    }
}

void MainWindow::timerSlot()
{
#ifdef HAS_JOYSTICK
    // Nothing to qDebug, as the function is called frequently

        double js_mr_thr = 0.0;
        double js_mr_roll = 0.0;
        double js_mr_pitch = 0.0;
        double js_mr_yaw = 0.0;

        // Update throttle and steering from keys.
        if (mJoystick->isConnected()) {

                mThrottle = -mJoystick->axisLeftY();
                deadband(mThrottle,0.1, 1.0);
                mSteering = mJoystick->axisRightX();
                // qDebug () << "Throttle: " << mThrottle << ", Steering: " << mSteering;

                js_mr_thr = -mJoystick->axisLeftY();
                js_mr_roll = mJoystick->axisRightX();
                js_mr_pitch = 0; // mJoystick->getAxis(4) / 32768.0; // GL - not sure which controller this corresponds to
                js_mr_yaw = 0; //mJoystick->getAxis(0) / 32768.0;   // GL - not sure which controller this corresponds to
                utility::truncate_number(&js_mr_thr, 0.0, 1.0);
                utility::truncate_number_abs(&js_mr_roll, 1.0);
                utility::truncate_number_abs(&js_mr_pitch, 1.0);
                utility::truncate_number_abs(&js_mr_yaw, 1.0);
//#ifdef DEBUG_FUNCTIONS
                if (mThrottle !=0 || mSteering !=0)
                {
                    qDebug() << QDateTime::currentDateTime().toString() << " - JOYSTICK (timerslot), mThrottle: " << mThrottle << ", mSteering: " << mSteering;
                }
  //  #endif
            //mSteering /= 2.0;
        } else {

            //qDebug () << "NOT CONNECTED!!!!!!!!!!";
            float throttleGain=0.03;
            float steeringGain=0.08;
            if (mKeyUp) {
                stepTowards(mThrottle, 1.0, throttleGain);
            } else if (mKeyDown) {
                stepTowards(mThrottle, -1.0, throttleGain);
            } else {
                stepTowards(mThrottle, 0.0, throttleGain);
            }

            if (mKeyRight) {
                stepTowards(mSteering, 1.0, steeringGain);
            } else if (mKeyLeft) {
                stepTowards(mSteering, -1.0, steeringGain);
            } else {
                stepTowards(mSteering, 0.0, steeringGain);
            }
        }
        ui->throttleBar->setValue(mThrottle * 100.0);
        ui->steeringBar->setValue(mSteering * 100.0);
#endif

    // Notify about key events
    for(QList<CarInterface*>::Iterator it_car = mCars.begin();it_car < mCars.end();it_car++) {
        CarInterface *car = *it_car;
        car->setControlValues(mThrottle, mSteering, ui->throttleMaxBox->value(), ui->throttleCurrentButton->isChecked());
    }
#ifdef HAS_JOYSTICK
    // Notify about joystick events
    for(QList<CopterInterface*>::Iterator it_copter = mCopters.begin();it_copter < mCopters.end();it_copter++) {
        CopterInterface *copter = *it_copter;
        copter->setControlValues(js_mr_thr, js_mr_roll, js_mr_pitch, js_mr_yaw);
    }
#endif
    // Update status label
    if (mStatusInfoTime) {
        mStatusInfoTime--;
        if (!mStatusInfoTime) {
            mStatusLabel->setStyleSheet(qApp->styleSheet());
        }
    } else {
        if (mSerialPort->isOpen() || mPacketInterface->isUdpConnected() || mTcpClientMulti->isAnyConnected()) {
            mStatusLabel->setText("Connected");
        } else {
            mStatusLabel->setText("Not connected");
        }
    }

    // Poll data (one vehicle per timeslot)
    static int next_car = 0;
    int ind = 0;
    int largest = 0;
    bool polled = false;

    for(QList<CarInterface*>::Iterator it_car = mCars.begin();it_car < mCars.end();it_car++) {
        CarInterface *car = *it_car;
        if ((mSerialPort->isOpen() || mPacketInterface->isUdpConnected() || mTcpClientMulti->isAnyConnected()) &&
                car->pollData() && ind >= next_car && !polled) {
            mPacketInterface->getState(car->getId());
            next_car = ind + 1;
            polled = true;
        }

        if (car->pollData() && ind > largest) {
            largest = ind;
        }

        ind++;
    }

    for(QList<CopterInterface*>::Iterator it_copter = mCopters.begin();it_copter < mCopters.end();it_copter++) {
        CopterInterface *copter = *it_copter;
        if ((mSerialPort->isOpen() || mPacketInterface->isUdpConnected() || mTcpClientMulti->isAnyConnected()) &&
                copter->pollData() && ind >= next_car && !polled) {
            mPacketInterface->getMrState(copter->getId());
            next_car = ind + 1;
            polled = true;
        }

        if (copter->pollData() && ind > largest) {
            largest = ind;
        }

        ind++;
    }

    if (next_car > largest) {
        next_car = 0;
    }

    // Update map settings
    if (ui->mapFollowBox->isChecked()) {
        ui->mapWidget->setFollowCar(ui->mapCarBox->value());
    } else {
        ui->mapWidget->setFollowCar(-1);
    }
    if (ui->mapTraceBox->isChecked()) {
        ui->mapWidget->setTraceCar(ui->mapCarBox->value());
    } else {
        ui->mapWidget->setTraceCar(-1);
    }
    ui->mapWidget->setSelectedCar(ui->mapCarBox->value());

    // Joystick connected
#ifdef HAS_JOYSTICK
    static bool jsWasconn = false;
    if (mJoystick->isConnected() != jsWasconn) {
        jsWasconn = mJoystick->isConnected();

        if (jsWasconn) {
            ui->jsConnectedLabel->setText("Connected");
        } else {
            ui->jsConnectedLabel->setText("Not connected");
            // STOP STOP STOP
            on_stopButton_clicked();
        }
    }
#endif

    // Update nmea stream connected label
    static bool wasNmeaStreamConnected = false;
    if (wasNmeaStreamConnected != mNmea->isClientTcpConnected()) {
        wasNmeaStreamConnected = mNmea->isClientTcpConnected();

        if (wasNmeaStreamConnected) {
            ui->mapStreamNmeaConnectedLabel->setText("Connected");
        } else {
            ui->mapStreamNmeaConnectedLabel->setText("Not connected");
        }
    }
}

void MainWindow::sendHeartbeat()
{
    for(QList<CarInterface*>::Iterator it_car = mCars.begin();it_car < mCars.end();it_car++)
        if ((*it_car)->getFirmwareVersion().first >= 20)
            mPacketInterface->sendHeartbeat((*it_car)->getId());

    for(QList<CopterInterface*>::Iterator it_copter = mCopters.begin();it_copter < mCopters.end();it_copter++)
        if ((*it_copter)->getFirmwareVersion().first >= 20)
            mPacketInterface->sendHeartbeat((*it_copter)->getId());
}

void MainWindow::showStatusInfo(QString info, bool isGood)
{
    if (mStatusLabel->text() == info) {
        mStatusInfoTime = 80;
        return;
    }

    if (isGood) {
        mStatusLabel->setStyleSheet("QLabel { background-color : lightgreen; color : black; }");
    } else {
        mStatusLabel->setStyleSheet("QLabel { background-color : red; color : black; }");
    }

    mStatusInfoTime = 80;
    mStatusLabel->setText(info);
}

void MainWindow::packetDataToSend(QByteArray &data)
{
    if (mSerialPort->isOpen()) {
        mSerialPort->write(data);
    }

    mTcpClientMulti->sendAll(data);
}

void MainWindow::stateReceived(quint8 id, CAR_STATE state)
{

    if (!mSupportedFirmwares.contains(qMakePair(state.fw_major, state.fw_minor))) {
        on_disconnectButton_clicked();
        QMessageBox::warning(this, "Unsupported Firmware",
                             "This version of RControlStation is not compatible with the "
                             "firmware of the connected car. Update RControlStation, the car "
                             "firmware or both.");
    } else {
        for(QList<CarInterface*>::Iterator it_car = mCars.begin();it_car < mCars.end();it_car++) {
            CarInterface *car = *it_car;
            if (car->getId() == id) {
                car->setStateData(state);
            }
        }
    }
}

void MainWindow::mrStateReceived(quint8 id, MULTIROTOR_STATE state)
{
    for(QList<CopterInterface*>::Iterator it_copter = mCopters.begin();it_copter < mCopters.end();it_copter++) {
        CopterInterface *copter = *it_copter;
        if (copter->getId() == id) {
            copter->setStateData(state);
        }
    }
}

void MainWindow::mapPosSet(quint8 id, LocPoint pos)
{
    mPacketInterface->setPos(id, pos.getX(), pos.getY(), pos.getYaw() * 180.0 / M_PI);
}

void MainWindow::ackReceived(quint8 id, CMD_PACKET cmd, QString msg)
{
    (void)cmd;
    QString str;
    str.sprintf("Vehicle %d ack: ", id);
    str += msg;
    showStatusInfo(str, true);
}

void MainWindow::rtcmReceived(QByteArray data)
{
    mPacketInterface->sendRtcmUsb(ID_ALL, data);

    if (ui->mapEnuBaseBox->isChecked()) {
        rtcm3_init_state(&mRtcmState);
        mRtcmState.decode_all = true;

        for(char b: data) {
            int res = rtcm3_input_data(b, &mRtcmState);
            if (res == 1005 || res == 1006) {
                ui->mapWidget->setEnuRef(mRtcmState.pos.lat, mRtcmState.pos.lon, mRtcmState.pos.height);
                ui->mapWidgetFarm->setEnuRef(mRtcmState.pos.lat, mRtcmState.pos.lon, mRtcmState.pos.height);
            }
        }
    }
}

void MainWindow::rtcmRefPosGet()
{
    QMessageBox::warning(this, "Reference Position",
                         "Not implemented yet");
}

void MainWindow::pingRx(int time, QString msg)
{
    QString str;
    str.sprintf("ping response time: %.3f ms", (double)time / 1000.0);
    QMessageBox::information(this, "Ping " + msg, str);
}

void MainWindow::pingError(QString msg, QString error)
{
    QMessageBox::warning(this, "Error ping " + msg, error);
}

void MainWindow::enuRx(quint8 id, double lat, double lon, double height)
{
    (void)id;
    ui->mapWidget->setEnuRef(lat, lon, height);
}

void MainWindow::nmeaGgaRx(int fields, NmeaServer::nmea_gga_info_t gga)
{
    if (fields >= 5) {
        if (gga.fix_type == 4 || gga.fix_type == 5 || gga.fix_type == 2 ||
                (gga.fix_type == 1 && !ui->mapStreamNmeaRtkOnlyBox->isChecked())) {
            double i_llh[3];

            if (ui->mapStreamNmeaZeroEnuBox->isChecked()) {
                i_llh[0] = gga.lat;
                i_llh[1] = gga.lon;
                i_llh[2] = gga.height;
                ui->mapWidget->setEnuRef(i_llh[0], i_llh[1], i_llh[2]);
                ui->mapStreamNmeaZeroEnuBox->setChecked(false);
            } else {
                ui->mapWidget->getEnuRef(i_llh);
            }

            double llh[3];
            double xyz[3];

            llh[0] = gga.lat;
            llh[1] = gga.lon;
            llh[2] = gga.height;
            utility::llhToEnu(i_llh, llh, xyz);

            LocPoint p;
            p.setXY(xyz[0], xyz[1]);
            QString info;

            QString fix_t = "Unknown";
            if (gga.fix_type == 4) {
                fix_t = "RTK fix";
                p.setColor(Qt::green);
            } else if (gga.fix_type == 5) {
                fix_t = "RTK float";
                p.setColor(Qt::yellow);
            } else if (gga.fix_type == 1) {
                fix_t = "Single";
                p.setColor(Qt::red);
            }

            info.sprintf("Fix type: %s\n"
                         "Sats    : %d\n"
                         "Height  : %.2f\n"
                         "Age     : %.2f",
                         fix_t.toLocal8Bit().data(),
                         gga.n_sat,
                         gga.height,
                         gga.diff_age);

            p.setInfo(info);
            ui->mapWidget->addInfoPoint(p);

            if (ui->mapStreamNmeaFollowBox->isChecked()) {
                ui->mapWidget->moveView(p.getX(), p.getY());
            }

            // Optionally stream the data over UDP
            if (ui->mapStreamNmeaForwardUdpBox->isChecked()) {
                QString hostString = ui->mapStreamNmeaForwardUdpHostEdit->text();
                QHostAddress host;

                host.setAddress(hostString);

                // In case setting the address failed try DNS lookup. Notice
                // that the lookup is stored in a static QHostInfo as long as
                // the host line does not change. This is to avoid some delay.
                if (host.isNull()) {
                    static QString hostStringBefore;
                    static QHostInfo hostBefore;

                    QList<QHostAddress> addresses = hostBefore.addresses();

                    // Make a new lookup if the address has changed or the old one is invalid.
                    if (hostString != hostStringBefore || addresses.isEmpty()) {
                        hostBefore = QHostInfo::fromName(hostString);
                        hostStringBefore = hostString;
                    }

                    if (!addresses.isEmpty()) {
                        host.setAddress(addresses.first().toString());
                    }
                }

                if (!host.isNull()) {
                    static int seq = 0;
                    QByteArray datagram;
                    QTextStream out(&datagram);
                    QString str;

                    utility::llhToXyz(llh[0], llh[1], llh[2],
                            &xyz[0], &xyz[1], &xyz[2]);

                    out << str.sprintf("%d\n", seq);          // Seq
                    out << str.sprintf("%05f\n", xyz[0]);     // X
                    out << str.sprintf("%05f\n", xyz[1]);     // Y
                    out << str.sprintf("%05f\n", xyz[2]);     // Height
                    out << str.sprintf("%05f\n", gga.t_tow);  // GPS time of week
                    out << str.sprintf("%d\n", 2);            // Vehicle ID
                    out.flush();

                    mUdpSocket->writeDatagram(datagram,
                                              host,
                                              ui->mapStreamNmeaForwardUdpPortBox->value());

                    seq++;
                } else {
                    QMessageBox::warning(this,
                                         tr("Host not found"),
                                         tr("Could not find %1").arg(hostString));
                    ui->mapStreamNmeaForwardUdpBox->setChecked(false);
                }
            }
        }
    }
}

void MainWindow::routePointAdded(LocPoint pos)
{
    (void)pos;
    QTime t = ui->mapRouteTimeEdit->time();
    t = t.addMSecs(ui->mapRouteAddTimeEdit->time().msecsSinceStartOfDay());
    ui->mapRouteTimeEdit->setTime(t);
}

void MainWindow::infoTraceChanged(int traceNow)
{
    ui->mapInfoTraceBox->setValue(traceNow);
}

void MainWindow::jsButtonChanged(int button, bool pressed)
{
        qDebug() << "JS BT:" << button << pressed;

    #ifdef HAS_JOYSTICK
        if (1) {
            // 5: Front Up
            // 7: Front Down
            // 4: Rear up
            // 6: Rear down
            // 1: Extra out
            // 3: Extra in

            if (button == 5 || button == 7 || button == 1 ||
                    button == 3 || button == 4 || button == 6) {
                for(QList<CarInterface*>::Iterator it_car = mCars.begin();it_car < mCars.end();it_car++) {
                    CarInterface *car = *it_car;
                    if (car->getCtrlKb()) {
                        if (button == 5 || button == 7) {
                            if (pressed) {
                                if (button == 5) {
                                    qDebug() << "Hydraulic, front up";
                                    mPacketInterface->hydraulicMove(car->getId(), HYDRAULIC_POS_FRONT, HYDRAULIC_MOVE_UP);
                                } else {
                                    qDebug() << "Hydraulic, front down";
                                    mPacketInterface->hydraulicMove(car->getId(), HYDRAULIC_POS_FRONT, HYDRAULIC_MOVE_DOWN);
                                }
                            } else {
                                mPacketInterface->hydraulicMove(car->getId(), HYDRAULIC_POS_FRONT, HYDRAULIC_MOVE_STOP);
                            }
                        } else if (button == 4 || button == 6) {
                            if (pressed) {
                                if (button == 4) {
                                    mPacketInterface->hydraulicMove(car->getId(), HYDRAULIC_POS_REAR, HYDRAULIC_MOVE_UP);
                                } else {
                                    mPacketInterface->hydraulicMove(car->getId(), HYDRAULIC_POS_REAR, HYDRAULIC_MOVE_DOWN);
                                }
                            } else {
                                mPacketInterface->hydraulicMove(car->getId(), HYDRAULIC_POS_REAR, HYDRAULIC_MOVE_STOP);
                            }
                        } else if (button == 1 || button == 3) {
                            if (pressed) {
                                if (button == 1) {
                                    mPacketInterface->hydraulicMove(car->getId(), HYDRAULIC_POS_EXTRA, HYDRAULIC_MOVE_OUT);
                                } else {
                                    mPacketInterface->hydraulicMove(car->getId(), HYDRAULIC_POS_EXTRA, HYDRAULIC_MOVE_IN);
                                }
                            } else {
                                mPacketInterface->hydraulicMove(car->getId(), HYDRAULIC_POS_EXTRA, HYDRAULIC_MOVE_STOP);
                            }
                        }
                    }
                }
            }
        }
    #endif
}

void MainWindow::on_disconnectButton_clicked()
{
    if (mSerialPort->isOpen()) {
        mSerialPort->close();
    }

    if (mPacketInterface->isUdpConnected()) {
        mPacketInterface->stopUdpConnection();
    }

    mTcpClientMulti->disconnectAll();
}

void MainWindow::on_mapRemoveTraceButton_clicked()
{
    ui->mapWidget->clearTrace();
}

void MainWindow::on_MapRemovePixmapsButton_clicked()
{
    ui->mapWidget->clearPerspectivePixmaps();
}


void MainWindow::on_tcpConnectButton_clicked()
{
    mTcpClientMulti->disconnectAll();

    QModelIndexList selectedIndexes = ui->vehicleTable->selectionModel()->selectedIndexes();
    for (const QModelIndex& index : selectedIndexes) {
        QString data = index.data().toString();
        qDebug() << "Selected Data:" << data;
    }
    if (selectedIndexes.isEmpty()) {
        qDebug() << "No selection";
        return;
    }

    // Using an iterator
    int oldrow=-1;
    qDebug() << "Button pressed";
    for (QList<QModelIndex>::iterator it = selectedIndexes.begin(); it != selectedIndexes.end(); ++it) {
        QModelIndex currentrow = *it;
        int row = currentrow.row();
        if (row!=oldrow)
        {
/*            int column = currentrow.column();
            qDebug() << "Selected Row: " << row;
            qDebug() << "Selected Column: " << column;
*/
            // Retrieve the data of the selected row if needed
            QString name = this->modelVehicle->data(this->modelVehicle->index(row, 0)).toString();
            QString ip = this->modelVehicle->data(this->modelVehicle->index(row, 1)).toString();
            QString port = this->modelVehicle->data(this->modelVehicle->index(row, 2)).toString();

            if (port.isEmpty())
            {
                QMessageBox msgBox;
                msgBox.setText(ip);
                msgBox.exec();
                mTcpClientMulti->addConnection(ip,8300);
            } else
            {
                QMessageBox msgBox;
                msgBox.setText(ip + ':' + port);
                msgBox.exec();
                mTcpClientMulti->addConnection(ip,port.toInt());
            };
            addCar(mCars.size() + mCopters.size(),name);
            qDebug() << "Name: " << name << ", ip: " << ip << ", port: " << port;
            oldrow=row;

        };
    }


/*
    QStringList conns = ui->tcpConnEdit->toPlainText().split("\n");

    for (QString c: conns) {
        QStringList ipPort = c.split(":");

        if (ipPort.size() == 1) {
            QMessageBox msgBox;
            msgBox.setText(ipPort.at(0));
            msgBox.exec();
            mTcpClientMulti->addConnection(ipPort.at(0),
                                           8300);
            on_carAddButton_clicked();
        } else if (ipPort.size() == 2) {
            QMessageBox msgBox;
            msgBox.setText(ipPort.at(0));
            msgBox.exec();
            mTcpClientMulti->addConnection(ipPort.at(0),
                                           ipPort.at(1).toInt());
            on_carAddButton_clicked();
        }
    } */

}

void MainWindow::on_tcpPingButton_clicked()
{
    QModelIndexList selectedIndexes = ui->vehicleTable->selectionModel()->selectedIndexes();
    for (const QModelIndex& index : selectedIndexes) {
        QString data = index.data().toString();
        qDebug() << "Selected Data:" << data;
    }
    if (selectedIndexes.isEmpty()) {
        qDebug() << "No selection";
        return;
    }

    // Using an iterator
    int oldrow=-1;
    qDebug() << "Button pressed";
    for (QList<QModelIndex>::iterator it = selectedIndexes.begin(); it != selectedIndexes.end(); ++it) {
        QModelIndex currentrow = *it;
        int row = currentrow.row();
        if (row!=oldrow)
        {
            // Retrieve the data of the selected row if needed
            QString ip = this->modelVehicle->data(this->modelVehicle->index(row, 1)).toString();

            mPing->pingHost(ip, 64, "TCP Host");
            oldrow=row;

        };
    }
/*
    QStringList conns = ui->tcpConnEdit->toPlainText().split("\n");

    for (QString c: conns) {
        QStringList ipPort = c.split(":");

        if (ipPort.size() == 2) {
            mPing->pingHost(ipPort.at(0), 64, "TCP Host");
            break;
        }
    }
*/
}

void MainWindow::on_mapZeroButton_clicked()
{
    ui->mapWidget->setXOffset(0);
    ui->mapWidget->setYOffset(0);
}

void MainWindow::on_mapRemoveRouteButton_clicked()
{
    ui->mapWidget->clearRoute();
}

void MainWindow::on_mapRouteSpeedBox_valueChanged(double arg1)
{
    ui->mapWidget->setRoutePointSpeed(arg1 / 3.6);
}

void MainWindow::on_jsConnectButton_clicked()
{
    connectJoystick(ui->jsPortEdit->text());
}

void MainWindow::on_jsDisconnectButton_clicked()
{
}

void MainWindow::on_mapAntialiasBox_toggled(bool checked)
{
    ui->mapWidget->setAntialiasDrawings(checked);
}

void MainWindow::on_carsWidget_tabCloseRequested(int index)
{
    QWidget *w = ui->carsWidget->widget(index);
    ui->carsWidget->removeTab(index);

    if (dynamic_cast<CarInterface*>(w) != NULL) {
        CarInterface *car = (CarInterface*)w;
        mCars.removeOne(car);
        delete car;
    } else if (dynamic_cast<CopterInterface*>(w) != NULL) {
        CopterInterface *copter = (CopterInterface*)w;
        mCopters.removeOne(copter);
        delete copter;
    }
}

void MainWindow::on_genCircButton_clicked()
{
    double rad = ui->genCircRadBox->value();
    double speed = ui->mapRouteSpeedBox->value() / 3.6;
    double ang_ofs = M_PI;
    double cx = 0.0;
    double cy = 0.0;
    int points = ui->genCircPointsBox->value();
    int type = ui->genCircCenterBox->currentIndex();

    if (type == 1 || type == 2) {
        CarInfo *car = ui->mapWidget->getCarInfo(ui->mapCarBox->value());
        if (car) {
            LocPoint p = car->getLocation();
            double ang = p.getYaw();

            cx = p.getX();
            cy = p.getY();

            if (ui->genCircCenterBox->currentIndex() == 1) {
                cx += rad * sin(ang);
                cy += rad * cos(ang);
                ang_ofs = ang + M_PI;
            }
        }
    } else if (type == 3) {
        cx = ui->genCircXBox->value();
        cy = ui->genCircYBox->value();
    } else if (type == 4) {
        QList<LocPoint> r = ui->mapWidget->getRoute();
        int samples = 0;
        cx = 0.0;
        cy = 0.0;

        for (LocPoint lp: r) {
            cx += lp.getX();
            cy += lp.getY();
            samples++;
        }

        if (samples > 0) {
            cx /= (double)samples;
            cy /= (double)samples;
        }
    }

    QList<LocPoint> route;

    for (int i = 1;i <= points;i++) {
        int ind = i;

        if (rad < 0.0) {
            ind = points - i;
        }

        double ang = -((double)ind * 2.0 * M_PI) / (double)points + ang_ofs;

        double px = sin(ang) * rad;
        double py = cos(ang) * rad;

        // Move up
        px += cx;
        py += cy;

        bool res = true;
        LocPoint pos;
        pos.setXY(px, py);
        pos.setSpeed(speed);
        route.append(pos);

        QList<LocPoint> points;
        points.append(pos);

        for (int i = 0;i < mCars.size();i++) {
            if (mCars[i]->updateRouteFromMap()) {
                res = mPacketInterface->setRoutePoints(mCars[i]->getId(), points);
            }
        }

        if (!res) {
            QMessageBox::warning(this, "Generate Cirlce",
                                 "No ack from car when uploading point.");
            break;
        }
    }

    if (ui->genCircAppendCurrentBox->isChecked()) {
        for (LocPoint p: route) {
            ui->mapWidget->addRoutePoint(p.getX(), p.getY(), p.getSpeed(), p.getTime());
        }
    } else {
        ui->mapWidget->addRoute(route);
    }
}

void MainWindow::on_mapSetAbsYawButton_clicked()
{
    CarInfo *car = ui->mapWidget->getCarInfo(ui->mapCarBox->value());
    if (car) {
        if (mSerialPort->isOpen() || mPacketInterface->isUdpConnected() || mTcpClientMulti->isAnyConnected()) {
            ui->mapSetAbsYawButton->setEnabled(false);
            ui->mapAbsYawSlider->setEnabled(false);
            bool ok = mPacketInterface->setYawOffsetAck(car->getId(), (double)ui->mapAbsYawSlider->value());
            ui->mapSetAbsYawButton->setEnabled(true);
            ui->mapAbsYawSlider->setEnabled(true);

            if (!ok) {
                qDebug() << "No pos ack received";
            }
        }
    }
}

void MainWindow::on_mapAbsYawSlider_valueChanged(int value)
{
    (void)value;
    CarInfo *car = ui->mapWidget->getCarInfo(ui->mapCarBox->value());
    if (car) {
        mPacketInterface->setYawOffset(car->getId(), (double)ui->mapAbsYawSlider->value());
    }
}

void MainWindow::on_mapAbsYawSlider_sliderReleased()
{
    on_mapSetAbsYawButton_clicked();
}

void MainWindow::on_stopButton_clicked()
{
    for (int i = 0;i < mCars.size();i++) {
        mCars[i]->emergencyStop();
    }

    mPacketInterface->setRcControlCurrentBrake(255, 40.0, 0.0);
    mPacketInterface->setRcControlCurrentBrake(255, 40.0, 0.0);
    mPacketInterface->setRcControlCurrentBrake(255, 40.0, 0.0);
    mPacketInterface->setRcControlCurrentBrake(255, 40.0, 0.0);
}

void MainWindow::on_mapUploadRouteButton_clicked()
{
    if (!mSerialPort->isOpen() && !mPacketInterface->isUdpConnected() && !mTcpClientMulti->isAnyConnected()) {
        QMessageBox::warning(this, "Upload route",
                             "Serial port not connected.");
        return;
    }

    QList<LocPoint> route = ui->mapWidget->getRoute();
    int len = route.size();
    int car = ui->mapCarBox->value();
    bool ok = true;

    if (len <= 0) {
        QMessageBox::warning(this, "Upload route",
                             "No route on map.");
        return;
    }

    ui->mapUploadRouteButton->setEnabled(false);

    // Stop car
    for (int i = 0;i < mCars.size();i++) {
        if (mCars[i]->getId() == car) {
            ok = mCars[i]->setAp(false);
            break;
        }
    }

    // Clear previous route
    if (ok) {
        ok = mPacketInterface->clearRoute(car);
    }

    QElapsedTimer timer;
    timer.start();

    if (ok) {
        int ind = 0;
        for (ind = 0;ind < len;ind += 5) {
            QList<LocPoint> tmpList;
            for (int j = ind;j < (ind + 5);j++) {
                if (j < len) {
                    tmpList.append(route.at(j));
                }
            }

            ok = mPacketInterface->setRoutePoints(car, tmpList);

            if (!ok) {
                break;
            }

            if (timer.elapsed() >= 20) {
                timer.restart();
                ui->mapUploadRouteProgressBar->setValue((100 * (ind + 5)) / len);
            }
        }
    }

    if (!ok) {
        QMessageBox::warning(this, "Upload route",
                             "No response when uploading route.");
    } else {
        ui->mapUploadRouteProgressBar->setValue(100);
    }

    ui->mapUploadRouteButton->setEnabled(true);
}

void MainWindow::on_mapGetRouteButton_clicked()
{
    if (!mSerialPort->isOpen() && !mPacketInterface->isUdpConnected() && !mTcpClientMulti->isAnyConnected()) {
        QMessageBox::warning(this, "Get route",
                             "Car not connected.");
        return;
    }

    ui->mapGetRouteButton->setEnabled(false);

    QList<LocPoint> route;
    int routeLen = 0;
    bool ok = mPacketInterface->getRoutePart(ui->mapCarBox->value(), route.size(), 10, route, routeLen);

    QElapsedTimer timer;
    timer.start();

    while (route.size() < routeLen && ok) {
        ok = mPacketInterface->getRoutePart(ui->mapCarBox->value(), route.size(), 10, route, routeLen);
        if (timer.elapsed() >= 20) {
            timer.restart();
            ui->mapUploadRouteProgressBar->setValue((100 * route.size()) / routeLen);
        }
    }

    while (route.size() > routeLen) {
        route.removeLast();
    }

    ui->mapGetRouteButton->setEnabled(true);

    if (ok) {
        if (route.size() > 0) {
            ui->mapWidget->setRoute(route);
            ui->mapUploadRouteProgressBar->setValue(100);
            showStatusInfo("GetRoute OK", true);
        } else {
            showStatusInfo("GetRoute OK, but route empty", true);
        }
    } else {
        showStatusInfo("GetRoute failed", false);
        QMessageBox::warning(this, "Get route",
                             "Could not get route from car.");
    }
}

void MainWindow::on_mapApButton_clicked()
{
    for (int i = 0;i < mCars.size();i++) {
        if (mCars[i]->getId() == ui->mapCarBox->value()) {
            mCars[i]->setCtrlAp();
        }
    }
    ui->throttleOffButton->setChecked(true);
}

void MainWindow::on_mapKbButton_clicked()
{
    for (int i = 0;i < mCars.size();i++) {
        if (mCars[i]->getId() == ui->mapCarBox->value()) {
            mCars[i]->setCtrlKb();
        }
    }
    ui->throttleDutyButton->setChecked(true);
}

void MainWindow::on_mapOffButton_clicked()
{
    for (int i = 0;i < mCars.size();i++) {
        if (mCars[i]->getId() == ui->mapCarBox->value()) {
            mCars[i]->emergencyStop();
        }
    }
}

void MainWindow::on_mapUpdateSpeedButton_clicked()
{
    QList<LocPoint> route = ui->mapWidget->getRoute();
    qint32 timeAcc = 0;

    for (int i = 0;i < route.size();i++) {
        double speed = ui->mapRouteSpeedBox->value() / 3.6;
        route[i].setSpeed(speed);

        if (i == 0) {
            route[i].setTime(0);
        } else {
            double dist = route[i].getDistanceTo(route[i - 1]);
            timeAcc += (dist / speed) * 1000.0;
            route[i].setTime(timeAcc);
        }
    }

    ui->mapWidget->setRoute(route);
}

void MainWindow::on_mapOpenStreetMapBox_toggled(bool checked)
{
    ui->mapWidget->setDrawOpenStreetmap(checked);
    ui->mapWidget->update();
}

void MainWindow::on_mapAntialiasOsmBox_toggled(bool checked)
{
    ui->mapWidget->setAntialiasOsm(checked);
}

void MainWindow::on_mapOsmResSlider_valueChanged(int value)
{
    ui->mapWidget->setOsmRes((double)value / 100.0);
}

void MainWindow::on_mapChooseNmeaButton_clicked()
{
    QString path;
    path = QFileDialog::getOpenFileName(this, tr("Choose log file to open"));
    if (path.isNull()) {
        return;
    }

    ui->mapImportNmeaEdit->setText(path);
}

void MainWindow::on_mapImportNmeaButton_clicked()
{
    QFile file;
    file.setFileName(ui->mapImportNmeaEdit->text());
    bool mapUpdated = false;

    if (file.exists()) {
        bool ok = file.open(QIODevice::ReadOnly | QIODevice::Text);

        if (ok) {
            QTextStream in(&file);

            double i_llh[3];
            bool i_llh_set = false;

            while(!in.atEnd()) {
                QString line = in.readLine();

                NmeaServer::nmea_gga_info_t gga;
                int res = NmeaServer::decodeNmeaGGA(line.toLocal8Bit(), gga);

                if (res > 5) {
                    if (!i_llh_set) {
                        if (ui->mapImportNmeaZeroEnuBox->isChecked()) {
                            i_llh[0] = gga.lat;
                            i_llh[1] = gga.lon;
                            i_llh[2] = gga.height;
                            ui->mapWidget->setEnuRef(i_llh[0], i_llh[1], i_llh[2]);
                        } else {
                            ui->mapWidget->getEnuRef(i_llh);
                        }

                        i_llh_set = true;
                    }

                    double llh[3];
                    double xyz[3];

                    llh[0] = gga.lat;
                    llh[1] = gga.lon;
                    llh[2] = gga.height;
                    utility::llhToEnu(i_llh, llh, xyz);

                    LocPoint p;
                    p.setXY(xyz[0], xyz[1]);
                    QString info;

                    QString fix_t = "Unknown";
                    if (gga.fix_type == 4) {
                        fix_t = "RTK fix";
                        p.setColor(Qt::green);
                    } else if (gga.fix_type == 5) {
                        fix_t = "RTK float";
                        p.setColor(Qt::yellow);
                    } else if (gga.fix_type == 1) {
                        fix_t = "Single";
                        p.setColor(Qt::red);
                    }

                    info.sprintf("Fix type: %s\n"
                                 "Sats    : %d\n"
                                 "Height  : %.2f",
                                 fix_t.toLocal8Bit().data(),
                                 gga.n_sat,
                                 gga.height);

                    p.setInfo(info);

                    if (!mapUpdated) {
                        mapUpdated = true;
                        ui->mapWidget->setNextEmptyOrCreateNewInfoTrace();
                    }

                    ui->mapWidget->addInfoPoint(p);
                }
            }
        } else {
            QMessageBox::warning(this, "Open Error", "Could not open " + file.fileName());
        }

    } else {
        QMessageBox::warning(this, "Open Error", "Please select a valid log file");
    }
}

void MainWindow::on_mapRemoveInfoAllButton_clicked()
{
    ui->mapWidget->clearAllInfoTraces();
}

void MainWindow::on_traceInfoMinZoomBox_valueChanged(double arg1)
{
    ui->mapWidget->setInfoTraceTextZoom(arg1);
}

void MainWindow::on_removeRouteExtraButton_clicked()
{
    on_mapRemoveRouteButton_clicked();
}

void MainWindow::on_mapOsmClearCacheButton_clicked()
{
    ui->mapWidget->osmClient()->clearCache();
    ui->mapWidget->update();
}

void MainWindow::on_mapOsmServerOsmButton_toggled(bool checked)
{
    if (checked) {
                ui->mapWidget->osmClient()->setTileServerUrl("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile");
    	//        ui->mapWidget->osmClient()->setTileServerUrl("http://tile.openstreetmap.org");
    }
}

void MainWindow::on_mapOsmServerHiResButton_toggled(bool checked)
{
    if (checked) {
                ui->mapWidget->osmClient()->setTileServerUrl("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile");
    	//        ui->mapWidget->osmClient()->setTileServerUrl("http://c.osm.rrze.fau.de/osmhd"); // Also https
    }
}

void MainWindow::on_mapOsmServerVedderButton_toggled(bool checked)
{
    if (checked) {
        ui->mapWidget->osmClient()->setTileServerUrl("http://tiles.vedder.se/osm_tiles");
    }
}

void MainWindow::on_mapOsmServerVedderHdButton_toggled(bool checked)
{
    if (checked) {
        ui->mapWidget->osmClient()->setTileServerUrl("http://tiles.vedder.se/osm_tiles_hd");
    }
}

void MainWindow::on_mapOsmMaxZoomBox_valueChanged(int arg1)
{
    ui->mapWidget->setOsmMaxZoomLevel(arg1);
}

void MainWindow::on_mapDrawGridBox_toggled(bool checked)
{
    ui->mapWidget->setDrawGrid(checked);
}

void MainWindow::on_mapGetEnuButton_clicked()
{
    mPacketInterface->getEnuRef(ui->mapCarBox->value());
}

void MainWindow::on_mapSetEnuButton_clicked()
{
    double llh[3];
    ui->mapWidget->getEnuRef(llh);
    mPacketInterface->setEnuRef(ui->mapCarBox->value(), llh);
}

void MainWindow::on_mapOsmStatsBox_toggled(bool checked)
{
    ui->mapWidget->setDrawOsmStats(checked);
}

void MainWindow::on_removeTraceExtraButton_clicked()
{
    ui->mapWidget->clearTrace();
}

void MainWindow::on_mapEditHelpButton_clicked()
{
    QMessageBox::information(this, tr("Keyboard shortcuts"),
                             tr("<b>CTRL + Left click:</b> Move selected car<br>"
                                "<b>CTRL + Right click:</b> Update route point or anchor settings<br>"
                                "<b>Shift + Left click:</b> Add route point or anchor<br>"
                                "<b>Shift + Left drag:</b> Move route point or anchor<br>"
                                "<b>Shift + right click:</b> Delete route point or anchor<br>"
                                "<b>CTRL + SHIFT + Left click:</b> Zero map ENU coordinates<br>"));
}

void MainWindow::on_mapStreamNmeaConnectButton_clicked()
{
    mNmea->connectClientTcp(ui->mapStreamNmeaServerEdit->text(),
                            ui->mapStreamNmeaPortBox->value());
}

void MainWindow::on_mapStreamNmeaDisconnectButton_clicked()
{
    mNmea->disconnectClientTcp();
}

void MainWindow::on_mapStreamNmeaClearTraceButton_clicked()
{
    ui->mapWidget->clearInfoTrace();
}

void MainWindow::on_mapRouteBox_valueChanged(int arg1)
{
    ui->mapWidget->setRouteNow(arg1);
}

void MainWindow::on_mapRemoveRouteAllButton_clicked()
{
    ui->mapWidget->clearAllRoutes();
}

void MainWindow::on_mapUpdateTimeButton_clicked()
{
    bool ok;
    int res = QInputDialog::getInt(this,
                                   tr("Set new route start time"),
                                   tr("Seconds from now"), 30, 0, 60000, 1, &ok);

    if (ok) {
        QList<LocPoint> route = ui->mapWidget->getRoute();
        QDateTime date = QDateTime::currentDateTime();
        QTime current = QTime::currentTime().addSecs(-date.offsetFromUtc());
        qint32 now = current.msecsSinceStartOfDay() + res * 1000;
        qint32 start_diff = 0;

        for (int i = 0;i < route.size();i++) {
            if (i == 0) {
                start_diff = now - route[i].getTime();
            }

            route[i].setTime(route[i].getTime() + start_diff);
        }

        ui->mapWidget->setRoute(route);
    }
}

void MainWindow::on_mapRouteTimeEdit_timeChanged(const QTime &time)
{
    ui->mapWidget->setRoutePointTime(time.msecsSinceStartOfDay());
}

void MainWindow::on_mapTraceMinSpaceCarBox_valueChanged(double arg1)
{
    ui->mapWidget->setTraceMinSpaceCar(arg1 / 1000.0);
}

void MainWindow::on_mapTraceMinSpaceGpsBox_valueChanged(double arg1)
{
    ui->mapWidget->setTraceMinSpaceGps(arg1 / 1000.0);
}

void MainWindow::on_mapInfoTraceBox_valueChanged(int arg1)
{
    ui->mapWidget->setInfoTraceNow(arg1);
}

void MainWindow::on_removeInfoTraceExtraButton_clicked()
{
    ui->mapWidget->clearInfoTrace();
}

void MainWindow::on_pollIntervalBox_valueChanged(int arg1)
{
    mTimer->setInterval(arg1);
}

void MainWindow::on_actionAbout_triggered()
{
    QMessageBox::about(this, "RControlStation",
                       tr("<b>RControlStation %1</b><br>"
                          "&copy; Benjamin Vedder 2016 - 2017<br>"
                          "<a href=\"mailto:benjamin@vedder.se\">benjamin@vedder.se</a><br>").
                       arg(mVersion));
}

void MainWindow::on_actionAboutLibrariesUsed_triggered()
{

    QMessageBox::about(this, "Libraries Used",
                       tr("<b>Icons<br>"
                          "<a href=\"https://icons8.com/\">https://icons8.com/</a><br><br>"
                          "<b>Plotting<br>"
                          "<a href=\"http://qcustomplot.com/\">http://qcustomplot.com/</a><br><br>"
                          "<b>Linear Algebra<br>"
                          "<a href=\"http://eigen.tuxfamily.org\">http://eigen.tuxfamily.org</a>"));
}

void MainWindow::on_actionExit_triggered()
{
    qApp->exit();
}

void MainWindow::on_actionSaveRoutes_triggered()
{
    saveRoutes(false);
}

void MainWindow::on_actionSaveRouteswithIDs_triggered()
{
    saveRoutes(true);
}

void MainWindow::on_actionLoadRoutes_triggered()
{
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load Routes"), "",
                                                    tr("Xml files (*.xml)"));

    if (!filename.isEmpty()) {
        int res = utility::loadRoutes(filename, ui->mapWidget);

        if (res >= 0) {
            showStatusInfo("Loaded routes", true);
        } else if (res == -1) {
            QMessageBox::critical(this, "Load Routes",
                                  "Could not open\n" + filename + "\nfor reading");
        } else if (res == -2) {
            QMessageBox::critical(this, "Load Routes",
                                  "routes tag not found in " + filename);
        } else {
            QMessageBox::critical(this, "Load Routes", "unknown error");
        }
    }
}

void MainWindow::on_actionTestIntersection_triggered()
{
    mIntersectionTest->show();
}

void MainWindow::on_actionSaveSelectedRouteAsDriveFile_triggered()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save Drive File"), "",
                                                    tr("Csv files (*.csv)"));

    // Cancel pressed
    if (filename.isEmpty()) {
        return;
    }

    if (!filename.toLower().endsWith(".csv")) {
        filename.append(".csv");
    }

    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        QMessageBox::critical(this, "Save Drive File",
                              "Could not open\n" + filename + "\nfor writing");
        showStatusInfo("Could not save drive file", false);
        return;
    }

    QFileInfo fileInfo(file);


    QTextStream stream(&file);
    stream.setCodec("UTF-8");

    QList<LocPoint> route = ui->mapWidget->getRoute();

    QString trajName = fileInfo.fileName();
    trajName.chop(4);
    stream << "TRAJECTORY;" << trajName <<
              ";0.1;" << route.size() << ";\n";

    for (LocPoint p: route) {
        // LINE;TIME(s);X(m);Y(m);Z(m);HEAD(rad, 0=right, ccw);VEL(m/s);ACCEL(m/s/s);CURVATURE(rad/m);MODE(0);ENDLINE;

        stream << "LINE;";
        stream << (double)p.getTime() / 1000.0 << ";";
        stream << p.getX() << ";" << p.getY() << ";" << "0.0;";
        stream << "0.0;";
        stream << p.getSpeed() << ";";
        stream << "0.0;";
        stream << "0.0;";
        stream << "0;";
        stream << "ENDLINE;\n";
    }

    stream << "ENDTRAJECTORY;";
    file.close();
    showStatusInfo("Saved drive file", true);
}

void MainWindow::on_actionLoadDriveFile_triggered()
{
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load Drive File"), "",
                                                    tr("Csv files (*.csv)"));

    if (!filename.isEmpty()) {
        QFile file(filename);
        if (!file.open(QIODevice::ReadOnly)) {
            QMessageBox::critical(this, "Load Drive File",
                                  "Could not open\n" + filename + "\nfor reading");
            return;
        }

        QTextStream stream(&file);

        QList<LocPoint> route;

        while (!stream.atEnd()) {
            QString line = stream.readLine();
            if (line.toUpper().startsWith("LINE;")) {
                QStringList tokens = line.split(";");

                // LINE;TIME(s);X(m);Y(m);Z(m);HEAD(rad, 0=right, ccw);VEL(m/s);ACCEL(m/s/s);CURVATURE(rad/m);MODE(0);ENDLINE;

                LocPoint p;
                p.setTime(tokens.at(1).toDouble() * 1000.0);
                p.setX(tokens.at(2).toDouble());
                p.setY(tokens.at(3).toDouble());
                p.setSpeed(tokens.at(6).toDouble());

                route.append(p);
            }
        }

        if (route.size() == 0) {
            file.close();
            QMessageBox::critical(this, "Load Drive File",
                                  "Drive file empty or could not be parsed.");
            return;
        }

        // Reduce route density
        QList<LocPoint> routeReduced;
        LocPoint pointLast = route.first();
        routeReduced.append(route.first());

        for (LocPoint p: route) {
            if (p.getDistanceTo(pointLast) >= 1.0) {
                routeReduced.append(p);
                pointLast = p;
            }
        }

        if (route.last().getDistanceTo(pointLast) > 0.01) {
            routeReduced.append(route.last());
        }

        ui->mapWidget->addRoute(routeReduced);

        file.close();
        showStatusInfo("Loaded drive file", true);
    }
}

void MainWindow::on_mapSaveAsPdfButton_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save Map Image"), "",
                                                    tr("Pdf files (*.pdf)"));

    // Cancel pressed
    if (filename.isEmpty()) {
        return;
    }

    if (!filename.toLower().endsWith(".pdf")) {
        filename.append(".pdf");
    }

    ui->mapWidget->printPdf(filename,
                            ui->mapSaveWBox->value(),
                            ui->mapSaveHBox->value());

    mLastImgFileName = filename;
}

void MainWindow::on_mapSaveAsPngButton_clicked()
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save Map Image"), "",
                                                    tr("png files (*.png)"));

    // Cancel pressed
    if (filename.isEmpty()) {
        return;
    }

    if (!filename.toLower().endsWith(".png")) {
        filename.append(".png");
    }

    ui->mapWidget->printPng(filename,
                            ui->mapSaveWBox->value(),
                            ui->mapSaveHBox->value());

    mLastImgFileName = filename;
}

void MainWindow::on_mapSaveRetakeButton_clicked()
{
    if (mLastImgFileName.toLower().endsWith(".pdf")) {
        ui->mapWidget->printPdf(mLastImgFileName,
                                ui->mapSaveWBox->value(),
                                ui->mapSaveHBox->value());
    } else if (mLastImgFileName.toLower().endsWith(".png")) {
        ui->mapWidget->printPng(mLastImgFileName,
                                ui->mapSaveWBox->value(),
                                ui->mapSaveHBox->value());
    } else {
        QMessageBox::critical(this, "Retake Image",
                              "No image has been taken yet, so a retake "
                              "is not possible.");
    }
}

void MainWindow::on_modeRouteButton_toggled(bool checked)
{
    ui->mapWidget->setAnchorMode(!checked);
}

void MainWindow::on_uploadAnchorButton_clicked()
{
    QVector<UWB_ANCHOR> anchors;
    for (LocPoint p: ui->mapWidget->getAnchors()) {
        UWB_ANCHOR a;
        a.dist_last = 0.0;
        a.height = p.getHeight();
        a.id = p.getId();
        a.px = p.getX();
        a.py = p.getY();
        anchors.append(a);
    }

    if (anchors.size() == 0) {
        return;
    }

    ui->uploadAnchorButton->setEnabled(false);

    bool ok = true;
    int car = ui->mapCarBox->value();

    ok = mPacketInterface->clearUwbAnchors(car);

    if (ok) {
        ui->anchorUploadProgressBar->setValue(0);
        for (int i = 0;i < anchors.size();i++) {
            ok = mPacketInterface->addUwbAnchor(car, anchors.at(i));
            if (!ok) {
                break;
            }
        }
    }

    if (!ok) {
        QMessageBox::warning(this, "Upload anchors",
                             "No response when uploading anchors.");
    } else {
        ui->anchorUploadProgressBar->setValue(100);
    }

    ui->uploadAnchorButton->setEnabled(true);
}

void MainWindow::on_anchorIdBox_valueChanged(int arg1)
{
    ui->mapWidget->setAnchorId(arg1);
}

void MainWindow::on_anchorHeightBox_valueChanged(double arg1)
{
    ui->mapWidget->setAnchorHeight(arg1);
}

void MainWindow::on_removeAnchorsButton_clicked()
{
    ui->mapWidget->clearAnchors();
}

void MainWindow::saveRoutes(bool withId)
{
    QString filename = QFileDialog::getSaveFileName(this,
                                                    tr("Save Routes"), "",
                                                    tr("Xml files (*.xml)"));

    // Cancel pressed
    if (filename.isEmpty()) {
        return;
    }

    if (!filename.toLower().endsWith(".xml")) {
        filename.append(".xml");
    }

    QFile file(filename);
    if (!file.open(QIODevice::WriteOnly)) {
        QMessageBox::critical(this, "Save Routes",
                              "Could not open\n" + filename + "\nfor writing");
        showStatusInfo("Could not save routes", false);
        return;
    }


    QXmlStreamWriter stream(&file);
    stream.setCodec("UTF-8");
    stream.setAutoFormatting(true);
    stream.writeStartDocument();

    stream.writeStartElement("routes");

    QList<LocPoint> anchors = ui->mapWidget->getAnchors();
    QList<QList<LocPoint> > routes = ui->mapWidget->getRoutes();

    if (!anchors.isEmpty()) {
        stream.writeStartElement("anchors");
        for (LocPoint p: anchors) {
            stream.writeStartElement("anchor");
            stream.writeTextElement("x", QString::number(p.getX()));
            stream.writeTextElement("y", QString::number(p.getY()));
            stream.writeTextElement("height", QString::number(p.getHeight()));
            stream.writeTextElement("id", QString::number(p.getId()));
            stream.writeEndElement();
        }
        stream.writeEndElement();
    }

    for (int i = 0;i < routes.size();i++) {
        if (!routes.at(i).isEmpty()) {
            stream.writeStartElement("route");

            if (withId) {
                stream.writeTextElement("id", QString::number(i));
            }

            for (const LocPoint p: routes.at(i)) {
                stream.writeStartElement("point");
                stream.writeTextElement("x", QString::number(p.getX()));
                stream.writeTextElement("y", QString::number(p.getY()));
                stream.writeTextElement("speed", QString::number(p.getSpeed()));
                stream.writeTextElement("time", QString::number(p.getTime()));
                stream.writeTextElement("attributes", QString::number(p.getAttributes()));
                stream.writeEndElement();
            }
            stream.writeEndElement();
        }
    }

    stream.writeEndDocument();
    file.close();
    showStatusInfo("Saved routes", true);
}

void MainWindow::on_mapDrawRouteTextBox_toggled(bool checked)
{
    ui->mapWidget->setDrawRouteText(checked);
}

void MainWindow::on_actionGPSSimulator_triggered()
{
#ifdef HAS_LIME_SDR
    mGpsSim->show();
#else
    QMessageBox::warning(this, "GPS Simulator",
                         "This version of RControlStation is not built with LIME SDR support, which "
                         "is required for the GPS simulator.");
#endif
}

void MainWindow::on_mapDrawUwbTraceBox_toggled(bool checked)
{
    ui->mapWidget->setDrawUwbTrace(checked);
}

void MainWindow::on_actionToggleFullscreen_triggered()
{
    if (isFullScreen()) {
        showNormal();
    } else {
        showFullScreen();
    }
}

void MainWindow::on_mapCameraWidthBox_valueChanged(double arg1)
{
    ui->mapWidget->setCameraImageWidth(arg1);
}

void MainWindow::on_mapCameraOpacityBox_valueChanged(double arg1)
{
    ui->mapWidget->setCameraImageOpacity(arg1);
}

void MainWindow::on_actionToggleCameraFullscreen_triggered()
{
    if (mCars.size() == 1) {
        mCars[0]->toggleCameraFullscreen();
    } else {
        for (int i = 0;i < mCars.size();i++) {
            if (mCars[i]->getId() == ui->mapCarBox->value()) {
                mCars[i]->toggleCameraFullscreen();
            }
        }
    }
}

void MainWindow::on_tabWidget_currentChanged(int index)
{
    // Focus on map widget when changing tab to it
    if (index == 1) {
        ui->mapWidget->setFocus();
    }
}

void MainWindow::on_routeZeroButton_clicked()
{
    ui->mapWidget->zoomInOnRoute(ui->mapRouteBox->value(), 0.1);
}

void MainWindow::on_routeZeroAllButton_clicked()
{
    ui->mapWidget->zoomInOnRoute(-1, 0.1);
}

void MainWindow::on_mapRoutePosAttrBox_currentIndexChanged(int index)
{
    quint32 attr = ui->mapWidget->getRoutePointAttributes();
    attr &= ~ATTR_POSITIONING_MASK;
    attr |= index;
    ui->mapWidget->setRoutePointAttributes(attr);
}

void MainWindow::on_clearAnchorButton_clicked()
{
    mPacketInterface->clearUwbAnchors(ui->mapCarBox->value());
}

void MainWindow::on_setBoundsRoutePushButton_clicked()
{
    int r = ui->mapWidget->getRouteNow();
    ui->boundsRouteSpinBox->setValue(r);
}

void MainWindow::on_boundsFillPushButton_clicked()
{
    QList<LocPoint> bounds = ui->mapWidget->getRoute(ui->boundsRouteSpinBox->value());

    double spacing = ui->boundsFillSpacingSpinBox->value();
    if (spacing < 0.5) return;

    QList<LocPoint> route;
    if (ui->generateFrameCheckBox->isChecked())
        route = RouteMagic::fillConvexPolygonWithFramedZigZag(bounds, spacing, ui->boundsFillKeepTurnsInBoundsCheckBox->isChecked(), ui->boundsFillSpeedSpinBox->value()/3.6,
                                                              ui->boundsFillSpeedInTurnsSpinBox->value()/3.6, ui->stepsForTurningSpinBox->value(), ui->visitEverySpinBox->value(),
                                                              ui->lowerToolsCheckBox->isChecked() ? ATTR_HYDRAULIC_FRONT_DOWN : 0, ui->raiseToolsCheckBox->isChecked() ? ATTR_HYDRAULIC_FRONT_UP : 0,
                                                              ui->lowerToolsDistanceSpinBox->value()*2, ui->raiseToolsDistanceSpinBox->value()*2);
                                                              // attribute changes at half distance
    else
        route = RouteMagic::fillConvexPolygonWithZigZag(bounds, spacing, ui->boundsFillKeepTurnsInBoundsCheckBox->isChecked(), ui->boundsFillSpeedSpinBox->value()/3.6,
                                                        ui->boundsFillSpeedInTurnsSpinBox->value()/3.6, ui->stepsForTurningSpinBox->value(), ui->visitEverySpinBox->value(),
                                                        ui->lowerToolsCheckBox->isChecked() ? ATTR_HYDRAULIC_FRONT_DOWN : 0, ui->raiseToolsCheckBox->isChecked() ? ATTR_HYDRAULIC_FRONT_UP : 0,
                                                        ui->lowerToolsDistanceSpinBox->value()*2, ui->raiseToolsDistanceSpinBox->value()*2);

    ui->mapWidget->addRoute(route);
    int r = ui->mapWidget->getRoutes().size()-1;
    ui->mapWidget->setRouteNow(r);
    ui->mapRouteBox->setValue(r);
    ui->mapWidget->repaint();

}

void MainWindow::on_lowerToolsCheckBox_stateChanged(int arg1)
{
    ui->lowerToolsDistanceSpinBox->setEnabled(arg1 != 0);
}

void MainWindow::on_raiseToolsCheckBox_stateChanged(int arg1)
{
    ui->raiseToolsDistanceSpinBox->setEnabled(arg1 != 0);
}

void MainWindow::on_WgSettingsPushButton_clicked()
{
    if (!mWireGuard)
        mWireGuard.reset(new WireGuard(this));
    mWireGuard->show();
}

void MainWindow::on_WgConnectPushButton_clicked()
{
    system("pkexec wg-quick up wg_sdvp");
    QList<QNetworkInterface> interfaces = QNetworkInterface::allInterfaces();
    if (std::find_if(interfaces.begin(), interfaces.end(),
                     [](QNetworkInterface currInterface){return currInterface.name() == "wg_sdvp";}) != interfaces.end())
        ui->wgStatusLabel->setText("Status: Interface up");
    else
        ui->wgStatusLabel->setText("Status: Config. error");
}

void MainWindow::on_WgDisconnectPushButton_clicked()

{
    system("pkexec wg-quick down wg_sdvp");
    QList<QNetworkInterface> interfaces = QNetworkInterface::allInterfaces();
    if (std::find_if(interfaces.begin(), interfaces.end(),
                     [](QNetworkInterface currInterface){return currInterface.name() == "wg_sdvp";}) != interfaces.end())
        ui->wgStatusLabel->setText("Status: Interface up");
    else
        ui->wgStatusLabel->setText("Status: Interface down");
}

void MainWindow::on_AutopilotConfigurePushButton_clicked()
{
//    ui->mainTabWidget->setCurrentIndex(ui->mainTabWidget->indexOf(ui->tab));
    ui->carsWidget->setCurrentIndex(ui->mapCarBox->value());

    QWidget *tmp = ui->carsWidget->widget(ui->mapCarBox->value());
    if (tmp) {
        CarInterface *car = dynamic_cast<CarInterface*>(tmp);
        car->showAutoPilotConfiguration();
    }
}

void MainWindow::on_AutopilotStartPushButton_clicked()
{
    ui->throttleOffButton->setChecked(true);
    QWidget *tmp = ui->carsWidget->widget(ui->mapCarBox->value());
    if (tmp) {
        CarInterface *car = dynamic_cast<CarInterface*>(tmp);
        if (ui->radioButton_followRoute->isChecked())
            car->setApMode(AP_MODE_FOLLOW_ROUTE);

        else if (ui->radioButton_followMe->isChecked())
            car->setApMode(AP_MODE_FOLLOW_ME);

        car->setAp(true, false);
    }
}

void MainWindow::on_AutopilotStopPushButton_clicked()
{
    QWidget *tmp = ui->carsWidget->widget(ui->mapCarBox->value());
    if (tmp) {
        CarInterface *car = dynamic_cast<CarInterface*>(tmp);
        car->setAp(false, true);
    }
}

void MainWindow::on_AutopilotRestartPushButton_clicked()
{
    ui->throttleOffButton->setChecked(true);
    QWidget *tmp = ui->carsWidget->widget(ui->mapCarBox->value());
    if (tmp) {
        CarInterface *car = dynamic_cast<CarInterface*>(tmp);
        if (ui->radioButton_followRoute->isChecked())
            car->setApMode(AP_MODE_FOLLOW_ROUTE);

        else if (ui->radioButton_followMe->isChecked())
            car->setApMode(AP_MODE_FOLLOW_ME);

        car->setAp(true, true);
    }
}

void MainWindow::on_AutopilotPausePushButton_clicked()
{
    QWidget *tmp = ui->carsWidget->widget(ui->mapCarBox->value());
    if (tmp) {
        CarInterface *car = dynamic_cast<CarInterface*>(tmp);
        car->setAp(false, false);
    }
}








/*
BookWindow::BookWindow()
{
    ui.setupUi(this);

}
*/
/*
bool BookWindow::eventFilter(QObject *object, QEvent *event)
{
    if ( object == ui.fieldTable &&  (event->type() == QEvent::HoverLeave )  ) {
        qDebug() << "OUT";
    }
    if ( object->objectName() == "fieldTable")
    {
        switch (event->type())
        {
            case QEvent::KeyPress:
                {
                    QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
//                    qDebug("Ate key press (field) %d", keyEvent->key());
                    switch ( keyEvent->key() )
                    {
                        case Qt::Key_Up:
                            qDebug() << "UP";
                            break;
                        case Qt::Key_Down:
                            qDebug() << "DOWN";
                            break;
                        case Qt::Key_Left:
                            qDebug() << "LEFT";
                            break;
                        case Qt::Key_Right:
                            qDebug() << "RIGHT";
                            break;
                    case Qt::Key_Delete:
                        qDebug() << "DELETE";
                        break;
                    }
                    return true;
                }
            default:
                {
                    //         qDebug() << object;
                    //         qDebug() << event;
                     return QObject::eventFilter(object, event);
                }
        }
    }
    if ( object->objectName() == "locationTable")
    {
        switch (event->type())
        {
            case QEvent::KeyPress:
                {
                    QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);
                    qDebug("Ate key press (location) %d", keyEvent->key());
                    return true;
                }
            default:
                {
                    //         qDebug() << object;
                    //         qDebug() << event;
                     return QObject::eventFilter(object, event);
                }
        }
    }

    ////////////////////////////////////
    switch ( event->key() )
    {
        case Qt::Key_Up:
            qDebug() << "UP";
            break;
        case Qt::Key_Down:
            qDebug() << "DOWN";
            break;
        case Qt::Key_Left:
            qDebug() << "LEFT";
            break;
        case Qt::Key_Right:
            qDebug() << "RIGHT";
            break;
    default:
            qDebug() << event->key() << endl;
            break;
/////////////////

    // false means it should be send to target also. as in , we dont remove it.
    // if you return true , you will take the event and widget never sees it so be carefull with that.
    return false;
}

*/
void MainWindow::showError(const QSqlError &err)
{
    QMessageBox::critical(this, "Unable to initialize Database",
                "Error initializing database: " + err.text());
}

void MainWindow::handleFieldButton()
{
    QMessageBox msgBox;
    msgBox.setText("Button handled: " + ui->fieldnameEdit->text()+', '+QString::number(ui->locationEdit->currentIndex()));
    msgBox.exec();
    const auto INSERT_FIELD_SQL = QLatin1String(R"(
        insert into fields(title, year, location, rating)
                          values(?, ?, ?, ?)
        )");
    QSqlQuery q;
    if (q.prepare(INSERT_FIELD_SQL))
    {
        addField(q, ui->fieldnameEdit->text(), ui->locationEdit->currentIndex());
    };
}

void MainWindow::handleLocationButton()
{
    QMessageBox msgBox;
    msgBox.setText("Button handled: " + ui->fieldnameEdit->text()+', '+QString::number(ui->locationEdit->currentIndex()));
    msgBox.exec();
    const auto INSERT_LOCATION_SQL = QLatin1String(R"(
        insert into fields(namw, longitude,latitude)
                          values(?, ?, ?)
        )");
    QSqlQuery q;
    if (q.prepare(INSERT_LOCATION_SQL))
    {
        addField(q, ui->fieldnameEdit->text(), ui->locationEdit->currentIndex());
    };
}


/*
void BookWindow::keyPressEvent( QKeyEvent *k )
{
    switch ( k->key() )
    {
        case Qt::Key_Up:
            qDebug() << "UP";
            break;
        case Qt::Key_Down:
            qDebug() << "DOWN";
            break;
        case Qt::Key_Left:
            qDebug() << "LEFT";
            break;
        case Qt::Key_Right:
            qDebug() << "RIGHT";
            break;
    default:
            qDebug() << k->key() << endl;
            break;
    }
}
*/

void addField(QSqlQuery &q, const QString &title, const QVariant &locationId)
{
    q.addBindValue(title);
    q.addBindValue(locationId);
    q.exec();
}

QVariant addLocation(QSqlQuery &q, const QString &name, QDate birthdate)
{
    q.addBindValue(name);
    q.addBindValue(birthdate);
    q.exec();
    return q.lastInsertId();
}

QSqlError initDb()
{
    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");
//    db.setDatabaseName(":memory:");
    db.setDatabaseName("test.db");
    if (!db.open())
        return db.lastError();
    return QSqlError();
}






