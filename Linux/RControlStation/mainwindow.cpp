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
#include <QCheckBox>

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

#include <QtCharts>

using namespace QtCharts;

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QListView>
#include <QStringListModel>


#include "utility.h"
#include "routemagic.h"
#include "wireguard.h"
#include "attributes_masks.h"
#include "datatypes.h"
#include "checkboxdelegate.h"

#include "ui_mainwindow.h"
// #include "rtcmwidget.h"

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


    QMessageBox msgStart;
    msgStart.setText("Before you start make sure that: 1) The entire electric system on the vehicle is turned on, 2) That the emergency stop is turned on");
    msgStart.exec();

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

    bLogLoaded=false;
    stepsize=1;




    checkboxdelegate=new CheckBoxDelegate();

    mPing = new Ping(this);
    mNmea = new NmeaServer(this);
    mUdpSocket = new QUdpSocket(this);
    mTcpClientMulti = new TcpClientMulti(this);
    mUdpSocket->setSocketOption(QAbstractSocket::LowDelayOption, true);

    mIntersectionTest = new IntersectionTest(this);
    mIntersectionTest->setCars(&mCars);
    mIntersectionTest->setMap(ui->mapLiveWidget);
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

    ui->mapLiveWidget->setRoutePointSpeed(ui->mapRouteSpeedBox->value() / 3.6);
    ui->networkLoggerWidget->setMap(ui->mapLiveWidget);
    ui->networkInterface->setMap(ui->mapLiveWidget);
    ui->networkInterface->setPacketInterface(mPacketInterface);
    ui->networkInterface->setCars(&mCars);
    ui->moteWidget->setPacketInterface(mPacketInterface);
    ui->nComWidget->setMap(ui->mapLiveWidget);

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
    connect(ui->mapLiveWidget, SIGNAL(posSet(quint8,LocPoint)),
            this, SLOT(mapPosSet(quint8,LocPoint)));
    connect(mPacketInterface, SIGNAL(ackReceived(quint8,CMD_PACKET,QString)),
            this, SLOT(ackReceived(quint8,CMD_PACKET,QString)));
    connect(this, SIGNAL(rtcmReceivedStep1(QByteArray)),
            this, SLOT(rtcmReceived(QByteArray)));
    connect(this, SIGNAL(refPosGet()), this, SLOT(rtcmRefPosGet()));
    connect(mPing, SIGNAL(pingRx(int,QString)), this, SLOT(pingRx(int,QString)));
    connect(mPing, SIGNAL(pingError(QString,QString)), this, SLOT(pingError(QString,QString)));
    connect(mPacketInterface, SIGNAL(enuRefReceived(quint8,double,double,double)),
            this, SLOT(enuRx(quint8,double,double,double)));
    connect(mNmea, SIGNAL(clientGgaRx(int,NmeaServer::nmea_gga_info_t)),
            this, SLOT(nmeaGgaRx(int,NmeaServer::nmea_gga_info_t)));
    connect(ui->mapLiveWidget, SIGNAL(routePointAdded(LocPoint)),
            this, SLOT(routePointAdded(LocPoint)));
    connect(ui->mapLiveWidget, SIGNAL(infoTraceChanged(int)),
            this, SLOT(infoTraceChanged(int)));

    connect(ui->actionAboutQt, SIGNAL(triggered(bool)),
            qApp, SLOT(aboutQt()));

    // Assuming that the signal mouseClickedInField in MapWidget is declared as a signal
    connect(ui->mapWidgetFields, SIGNAL(mouseClickedInField(int)), this, SLOT(onMouseClickedFieldSelected(int)));
    connect(ui->mapWidgetLog, SIGNAL(mouseClickedInField(int)), this, SLOT(onMouseClickedFieldSelected(int)));

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
    } else {
        ui->wgStatusLabel->setText("Status: Not installed");
    }

#ifdef HAS_JOYSTICK
    // Connect micronav joystick by default
    if (!connectJoystick())
    {
        QMessageBox msg;
        msg.setText("No joystick connected. Please connect one before you continue");
        msg.exec();
        connectJoystick();
    }
#endif

#ifdef HAS_SIM_SCEN
    mSimScen = new PageSimScen;
    ui->mainTabWidget->addTab(mSimScen, QIcon(":/models/Icons/Sedan-96.png"), "");
    ui->mainTabWidget->setTabToolTip(ui->mainTabWidget->count() - 1,
                                     "Simulation Scenarios");
#endif

    on_WgConnectPushButton_clicked();

    if (!QSqlDatabase::drivers().contains("QSQLITE"))
        QMessageBox::critical(
                    this,
                    "Unable to load database",
                    "This program needs the SQLITE driver"
                    );

    // Initialize the database:
    QSqlError err = initDb();
    if (err.type() != QSqlError::NoError) {
        showError(err);
        return;
    }

    modelFarm=setupFarmTable(ui->farmTable,"locations",true);
    modelFarmLog=setupFarmTable(ui->farmTable_Log,"locations",false);
    modelField=setupFieldTable(ui->fieldTable,"fields");
    modelFieldLog=setupFieldTable(ui->fieldTable_Log,"fields");
    modelPath=setupPathTable(ui->pathTable,"paths");
    modelPathLog=setupFieldTable(ui->pathTable_Log,"paths");
    modelTestLog=setupLogTable(ui->testTable_Log,"drivelogs");

    // Connect the signal from the first table view to a custom slot
    QObject::connect(ui->farmTable->selectionModel(), &QItemSelectionModel::currentChanged, this, &MainWindow::onSelectedFarm);
    QObject::connect(ui->farmTable_Log->selectionModel(), &QItemSelectionModel::currentChanged, this, &MainWindow::onSelectedFarmLog);
    QObject::connect(ui->fieldTable->selectionModel(), &QItemSelectionModel::currentChanged, this, &MainWindow::onSelectedField);
    QObject::connect(ui->fieldTable_Log->selectionModel(), &QItemSelectionModel::currentChanged, this, &MainWindow::onSelectedFieldLog);
    QObject::connect(ui->pathTable->selectionModel(), &QItemSelectionModel::currentChanged, this, &MainWindow::onSelectedPath);
    QObject::connect(ui->pathTable_Log->selectionModel(), &QItemSelectionModel::currentChanged, this, &MainWindow::onSelectedPathLog);
    QObject::connect(ui->testTable_Log->selectionModel(), &QItemSelectionModel::currentChanged, this, &MainWindow::onSelectedLog);

  ui->fieldTable->installEventFilter(&filterFieldtable);

  MapWidget *mapFields=ui->mapWidgetFields;

  QObject::connect(&filterFieldtable, &FocusEventFilter::focusGained, [mapFields]() {
      qDebug() << "Focus gained Fields";
  });
 /* ui->pathTable->installEventFilter(&filterPathtable);

  QObject::connect(&filterPathtable, &FocusEventFilter::focusGained, [mapFields]() {
      mapFields->setBorderFocus(false);
      mapFields->update();
      qDebug() << "Focus gained Paths";
  });
*/
//  modelVehicleTypes = new QSqlRelationalTableModel(ui->pathTable);
  // Assuming you have a QSqlDatabase connection named "db" and a QComboBox named "comboBox"
  QSqlQuery query("SELECT id, name FROM vehicle_types"); // Replace with your table name
  if (query.exec()) {
        while (query.next()) {
            int id = query.value(0).toInt();
            QString name = query.value(1).toString();
            qDebug() << "Vehicle: " << id << " (id), " << name << " (name)";
            ui->comboBoxVehicleType->addItem(name, id); // Add the name to the combo box with the associated id as userData
        }
  } else {
        // Handle query error
  }

    // Create the data model:
    modelVehicle = new QSqlRelationalTableModel(ui->vehicleTable);
    modelVehicle->setEditStrategy(QSqlTableModel::OnFieldChange);
    modelVehicle->setTable("vehicles");

    // Set the localized header captions:
    modelVehicle->setHeaderData(modelVehicle->fieldIndex("name"), Qt::Horizontal, tr("Name"));
    modelVehicle->setHeaderData(modelVehicle->fieldIndex("ip"), Qt::Horizontal, tr("IP"));
    modelVehicle->setHeaderData(modelVehicle->fieldIndex("port"), Qt::Horizontal, tr("port"));
    modelVehicle->setHeaderData(modelVehicle->fieldIndex("steering_type"), Qt::Horizontal, tr("Steering type"));
    modelVehicle->setHeaderData(modelVehicle->fieldIndex("length"), Qt::Horizontal, tr("Length"));

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

    ui->vehicleTable->setCurrentIndex(modelVehicle->index(0, 0));

    // Sample data to display in the QTableView
    QStringList data;
    int rowCount = modelVehicle->rowCount();

    // Access the data from the first column of the source model
    for (int row = 0; row < rowCount; ++row) {
        // Assuming the first column of the source model is the one you want to fetch
        QModelIndex index = modelVehicle->index(row, 0);
        QString value = modelVehicle->data(index).toString();
        data.append(value);
    }

    // Create a QStandardItemModel and set the data
    modelVehiclestatus=new QStandardItemModel(data.size(), 2);

    statustocolourDelegate=new CustomDelegate;
    ui->vehiclestatusTable->setItemDelegateForColumn(1, statustocolourDelegate);

    for (int row = 0; row < data.size(); ++row) {
        QStandardItem *item = new QStandardItem(data.at(row));
        modelVehiclestatus->setItem(row, 0, item);

        // Set the second column to "Not connected"
        QStandardItem *notConnectedItem = new QStandardItem("Not connected");
        modelVehiclestatus->setItem(row, 1, notConnectedItem);

        QModelIndex cellIndex = modelVehiclestatus->index(row, 1); // Row 1, Column 1 (second column)
        QColor colourred = Qt::red;
        modelVehiclestatus->setData(cellIndex, colourred, Qt::TextColorRole);
    }

    // Set the model on the QTableView
    ui->vehiclestatusTable->setModel(modelVehiclestatus);

    // Set headers for both rows and columns
    ui->vehiclestatusTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch); // Adjust the column width to the content
    ui->vehiclestatusTable->verticalHeader()->setVisible(false); // Hide vertical headers

    // Set horizontal header labels
    QStringList headerLabels;
    headerLabels << "Vehicle" << "Status";
    modelVehiclestatus->setHorizontalHeaderLabels(headerLabels);

    QModelIndex cellIndex = modelVehiclestatus->index(1, 1); // Row 1, Column 1 (second column)
    QColor coluorgreen = Qt::green;
    modelVehiclestatus->setData(cellIndex, coluorgreen, Qt::TextColorRole);

    connect(ui->pushButton_farm, &QPushButton::released, this, &MainWindow::handleAddFarmButton);
    connect(ui->pushButton_field, &QPushButton::released, this, &MainWindow::handleAddFieldButton);
    connect(ui->pushButton_ImportPath, &QPushButton::released, this, &MainWindow::handleImportPathButton);

    ui->farmTable->setFocus();
    ui->fieldTable->installEventFilter(this);
    ui->farmTable->installEventFilter(this);
    ui->vehicleTable->installEventFilter(this);

//    ui->farmTable->selectRow(0);

    if (ui->fieldTable->model()->rowCount()>0)
    {
        ui->fieldTable->selectRow(0);
    }
    if (ui->pathTable->model()->rowCount()>0)
    {
        ui->pathTable->selectRow(0);
    }

    ui->mapWidgetLog->update();
    ui->horizontalSliderStart->setValue(0);

    // Create a QStringListModel instance
    QStringListModel* modelChoseVariable = new QStringListModel();

    // Create a QStringList containing items
    QStringList items;
    items << "Speed" << "Angle";

    // Set the QStringList as the model's data
    modelChoseVariable->setStringList(items);

    // Create a QListView instance

    // Set the model to the QListView
    ui->listViewVariable->setModel(modelChoseVariable);

//    connect(ui->loadTrackButton, &QPushButton::clicked, this, &MainWindow::onLoadLogFile);

    scatterSeriesLog.setMarkerShape(QScatterSeries::MarkerShapeCircle);
    scatterSeriesLog.setMarkerSize(7.0);
    scatterSeriesLog.setColor(Qt::blue);

    chartLog.setTitle("Speed");
    chartLog.addSeries(&scatterSeriesLog);
    chartLog.createDefaultAxes(); // Add axes to the chart

    //    QChartView chartView(&chartLog);
    QChartView *chartView=new QChartView(&chartLog);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setParent(ui->horizontalFrame);

    // Connect the valueChanged signal to a slot function
    QObject::connect(ui->horizontalSliderStart, &QSlider::valueChanged, this, &MainWindow::onLogStartSliderChange);

    QObject::connect(ui->horizontalSliderEnd, &QSlider::valueChanged, this, &MainWindow::onLogEndSliderChange);

    // Connect the clicked signal to the handleItemSelectionChange slot
    QObject::connect(ui->listViewVariable, &QListView::clicked, this, &MainWindow::onLogVariableSelection);

    ui->mainTabWidget->removeTab(8);
    ui->mainTabWidget->removeTab(7);
    ui->mainTabWidget->removeTab(6);
    ui->mainTabWidget->removeTab(5);
    ui->mainTabWidget->removeTab(4);
    ui->controlWidget->removeTab(3);
    ui->controlWidget->removeTab(2);

    // START RTCM
    mRtcm = new RtcmClient(this);
    mTimerRtcm = new QTimer(this);
    mTimerRtcm->start(20);
    mTcpServerRtcm = new TcpBroadcast(this);

    connect(mRtcm, SIGNAL(rtcmReceivedStep1(QByteArray,int,bool)),
            this, SLOT(rtcmRx(QByteArray,int,bool)));
    connect(mRtcm, SIGNAL(refPosReceived(double,double,double,double)),
            this, SLOT(refPosRx(double,double,double,double)));
    connect(mTimerRtcm, SIGNAL(timeout()),
            this, SLOT(timerSlotRtcm()));

    on_gpsOnlyBox_toggled(ui->gpsOnlyBox->isChecked());

    // END RTCM

    qApp->installEventFilter(this);
}

QSqlRelationalTableModel* MainWindow::setupLogTable(QTableView* uiLogTable,QString sqlTablename)
{
    QSqlRelationalTableModel *model= new QSqlRelationalTableModel(uiLogTable);
    model->setEditStrategy(QSqlTableModel::OnFieldChange);
    model->setTable(sqlTablename);

    // Remember the indexes of the columns:
    int pathIdx = model->fieldIndex("path");

    // Set the relations to the other database tables:
    model->setRelation(pathIdx, QSqlRelation("paths", "iPath", "name"));

    // Set the localized header captions:
    model->setHeaderData(model->fieldIndex("filename"), Qt::Horizontal, tr("File name"));

    // Populate the model:
    if (!model->select()) {
        showError(model->lastError());
        return model;
    }

    // Set the model and hide the ID column:
    uiLogTable->setModel(model);
    uiLogTable->setColumnHidden(model->fieldIndex("id"), true);
    uiLogTable->setColumnHidden(model->fieldIndex("name"), true);
    uiLogTable->setSelectionMode(QAbstractItemView::SingleSelection);
    uiLogTable->setCurrentIndex(model->index(0, 0));
    uiLogTable->resizeColumnsToContents();
    uiLogTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    return model;
}

QSqlRelationalTableModel* MainWindow::setupPathTable(QTableView* uiPathTable,QString sqlTablename)
{
    QSqlRelationalTableModel *model= new QSqlRelationalTableModel(uiPathTable);
    model->setEditStrategy(QSqlTableModel::OnFieldChange);
    model->setTable(sqlTablename);

    // Remember the indexes of the columns:
    int fieldIdx = model->fieldIndex("field");

    // Set the relations to the other database tables:
    model->setRelation(fieldIdx, QSqlRelation("fields", "id", "name"));

    // Set the localized header captions:
    model->setHeaderData(model->fieldIndex("name"), Qt::Horizontal, tr("Name"));
//    model->setHeaderData(model->fieldIndex("xml"), Qt::Horizontal, tr("XML"));

    // Populate the model:
    if (!model->select()) {
        showError(model->lastError());
        return model;
    }

    // Set the model and hide the ID column:
    uiPathTable->setModel(model);
    uiPathTable->setColumnHidden(model->fieldIndex("iPath"), true);
    uiPathTable->setColumnHidden(model->fieldIndex("xml"), true);
    uiPathTable->setColumnHidden(model->fieldIndex("field"), true);
    uiPathTable->setColumnHidden(model->fieldIndex("fields_name_2"), true);
    uiPathTable->setSelectionMode(QAbstractItemView::SingleSelection);
    uiPathTable->setCurrentIndex(model->index(0, 0));
    uiPathTable->resizeColumnsToContents();
    uiPathTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    uiPathTable->horizontalHeader()->setVisible(false); // Hide vertical headers
    return model;
}

QSqlRelationalTableModel* MainWindow::setupFieldTable(QTableView* uiFieldTable,QString sqlTablename)
{
    // Create the data model:
    QSqlRelationalTableModel *model = new QSqlRelationalTableModel(uiFieldTable);
    model->setEditStrategy(QSqlTableModel::OnFieldChange);
    model->setTable(sqlTablename);

    // Remember the indexes of the columns:
    int locationIdx = model->fieldIndex("location");

    // Set the relations to the other database tables:
    model->setRelation(locationIdx, QSqlRelation("locations", "id", "name"));

    // Set the localized header captions:
    model->setHeaderData(locationIdx, Qt::Horizontal, tr("Location"));
    model->setHeaderData(model->fieldIndex("name"),  Qt::Horizontal, tr("Field name"));
    model->setHeaderData(model->fieldIndex("fenced"),  Qt::Horizontal, tr("Is fenced?"));
    model->setHeaderData(model->fieldIndex("boundaryXML"),  Qt::Horizontal, tr("boundary"));
    model->setHeaderData(model->fieldIndex("location"),  Qt::Horizontal, tr("Location"));
    model->setHeaderData(model->fieldIndex("id"),  Qt::Horizontal, tr("Id"));

    // Set the model and hide the ID column:
    uiFieldTable->setModel(model);
    uiFieldTable->setColumnHidden(model->fieldIndex("id"), true);
    uiFieldTable->setColumnHidden(model->fieldIndex("location"), true);
    uiFieldTable->setColumnHidden(model->fieldIndex("fenced"), true);
    uiFieldTable->setColumnHidden(model->fieldIndex("boundaryXML"), true);
    uiFieldTable->setSelectionMode(QAbstractItemView::SingleSelection);
    //  ui->fieldTable->setSelectionModel(new QItemSelectionModel( ui->fieldTable->model()));
    uiFieldTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    uiFieldTable->setItemDelegateForColumn(3, checkboxdelegate);
    uiFieldTable->resizeColumnsToContents();
    uiFieldTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    uiFieldTable->horizontalHeader()->setVisible(false); // Hide vertical headers
    return model;
}

QSqlRelationalTableModel* MainWindow::setupFarmTable(QTableView* uiFarmTable,QString sqlTablename,bool bShowAll)
{
    // Create the data model:
    QSqlRelationalTableModel* model = new QSqlRelationalTableModel(uiFarmTable);
    model->setEditStrategy(QSqlTableModel::OnFieldChange);
    model->setTable(sqlTablename);

    // Set the localized header captions:
    model->setHeaderData(model->fieldIndex("name"), Qt::Horizontal, tr("Name"));
    if (bShowAll)
    {
        model->setHeaderData(model->fieldIndex("longitude"), Qt::Horizontal, tr("Longitude"));
        model->setHeaderData(model->fieldIndex("latitude"), Qt::Horizontal, tr("Latitude"));
        model->setHeaderData(model->fieldIndex("ip"), Qt::Horizontal, tr("ip"));
        model->setHeaderData(model->fieldIndex("port"), Qt::Horizontal, tr("port"));
        model->setHeaderData(model->fieldIndex("NTRIP"), Qt::Horizontal, tr("NTRIP"));
    }

    // Populate the model:
    if (!model->select()) {
        showError(model->lastError());
        return model;
    }

    // Set the model and hide the ID column:
    uiFarmTable->setModel(model);
    //ui.locationTable->setItemDelegate(new BookDelegate(ui.locationTable));
    uiFarmTable->setColumnHidden(model->fieldIndex("id"), true);
    if (!bShowAll)
    {
        uiFarmTable->setColumnHidden(model->fieldIndex("longitude"), true);
        uiFarmTable->setColumnHidden(model->fieldIndex("latitude"), true);
        uiFarmTable->setColumnHidden(model->fieldIndex("ip"), true);
        uiFarmTable->setColumnHidden(model->fieldIndex("port"), true);
        uiFarmTable->setColumnHidden(model->fieldIndex("NTRIP"), true);
        uiFarmTable->setColumnHidden(model->fieldIndex("user"), true);
        uiFarmTable->setColumnHidden(model->fieldIndex("password"), true);
        uiFarmTable->setColumnHidden(model->fieldIndex("stream"), true);
        uiFarmTable->setColumnHidden(model->fieldIndex("autoconnect"), true);
    };
    uiFarmTable->setSelectionMode(QAbstractItemView::SingleSelection);
    uiFarmTable->setCurrentIndex(model->index(0, 0));

    if (bShowAll)
    {
        QDataWidgetMapper *mapperFarm = new QDataWidgetMapper(this);
        mapperFarm->setModel(model);
        mapperFarm->addMapping(this->findChild<QDoubleSpinBox*>("refSendLatBox"), model->fieldIndex("latitude"));
        mapperFarm->addMapping(this->findChild<QDoubleSpinBox*>("refSendLonBox"), model->fieldIndex("longitude"));
        connect(uiFarmTable->selectionModel(),&QItemSelectionModel::currentRowChanged,mapperFarm,&QDataWidgetMapper::setCurrentModelIndex);
    }    else
    {
        uiFarmTable->resizeColumnsToContents();
        uiFarmTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    }
    return model;
}


void MainWindow::onLogVariableSelection(const QModelIndex &index)
{
    onUpdateLogGraph();
}

void MainWindow::onLogStartSliderChange(int newValue)
{
    qDebug() << "LogStartSlider";
    ui->mapWidgetLog->setLogStart(newValue);
    emit onLogSliderChange();
}

void MainWindow::onLogEndSliderChange(int newValue)
{
    qDebug() << "LogEndSlider";
    ui->mapWidgetLog->setLogEnd(newValue);
    emit onLogSliderChange();
}

void MainWindow::onUpdateLogGraph()
{
    int iFirstelement=ui->mapWidgetLog->firstElement();
    int iLastelement=ui->mapWidgetLog->lastElement();
    QScatterSeries* scatterSeriesLog2=&scatterSeriesLog;
    scatterSeriesLog2->clear();

    QVector<QPointF> *datasetLog;

    QModelIndex selectedIndex = ui->listViewVariable->selectionModel()->currentIndex();
    if (selectedIndex.isValid()) {
        switch(selectedIndex.row())
        {
        case 0:
            datasetLog=&datasetSpeed;
            break;
        case 1:
            datasetLog=&datasetAngle;
            break;
        }
        float firsttime=datasetLog->at(iFirstelement).x();
        float lasttime=datasetLog->at(iLastelement).x();
        float maxvalue=-999;
        float minvalue=999;
        for (int i = iFirstelement; i < iLastelement; i+=stepsize) {
            float tmpy=datasetLog->at(i).y();
            float x=datasetX.at(i).y();
            float y=datasetY.at(i).y();
            qDebug() << "x:" << x;
            qDebug() << ", y:" << y;
            qDebug() << ", time:" << datasetLog->at(i).x() << ", value: " << tmpy;
            maxvalue = (tmpy>maxvalue) ? tmpy : maxvalue;
            minvalue = (tmpy<minvalue) ? tmpy : minvalue;
            //            if (isPointWithin(x, y, QList<LocPoint> route);)
            //            {
            scatterSeriesLog2->append(datasetLog->at(i));
            //            }
        }
        qDebug() << "First element:" << iFirstelement << ", time: " << firsttime;
        qDebug() << "Last element:" << iLastelement << ", time: " << lasttime;
        ui->minEdit->setText(QString::number(minvalue));
        ui->maxEdit->setText(QString::number(maxvalue));
        QChart* chartLog2=&chartLog;
        chartLog2->axes(Qt::Horizontal).first()->setRange(firsttime, lasttime);
        chartLog2->axes(Qt::Vertical).first()->setRange(minvalue, maxvalue*1.1);
        chartLog2->update();
    } else
    {
        QMessageBox msg;
        msg.setText("No variable chosen!");
        msg.exec();
    }
}

void MainWindow::onLogSliderChange()
{
    if (bLogLoaded)
    {
        int iElements=ui->mapWidgetLog->Elements();
        ui->labelElements->setText(QString::number(iElements));
        if (iElements>300)
        {
            stepsize=(int)(iElements/300);
        } else stepsize=1;

        onUpdateLogGraph();
    } else
    {
        QMessageBox msg;
        msg.setText("No log loaded!");
        msg.exec();
    };
}

void MainWindow::onSelectedFarmGeneral(QSqlRelationalTableModel *model,QSqlRelationalTableModel *modelFld,QSqlRelationalTableModel *modelPth,const QModelIndex& current, const QModelIndex& previous,bool bAct)
{
    qDebug() << "onSelectedFarm";
    int row = current.row();

    int id = model->data(model->index(row, 0)).toInt();

    double lon = model->data(model->index(row, 2)).toDouble();
    double lat = model->data(model->index(row, 3)).toDouble();

//    NetworkInterface::sendEnuRef(quint8 id, double lat, double lon, double height)
    MapWidget* activeMap;
    if (bAct)
    {
        activeMap=ui->mapWidgetFields;

        double llh[3]={lat,lon,0};
        mPacketInterface->setEnuRef(ui->mapCarBox->value(), llh);

        ui->mapLiveWidget->setEnuRef(lat,lon,0);
        ui->networkInterface->sendEnuRef(1,lat,lon,0);
    } else
    {
        activeMap=ui->mapWidgetLog;
    }
    activeMap->setEnuRef(lat,lon,0);
    activeMap->clearAllFields();
    activeMap->clearAllPaths();
    activeMap->clearTrace();
    // Get the selected value from the first table view
    QVariant selectedValue = id;

    // Construct a new query based on the selected value
    QString filter = QString("location = %1").arg(id);
    // Set the new query for the QSqlRelationalTableModel
    modelFld->setFilter(filter);
    modelFld->select();
    //To make sure the path table view is empty until a field has been selected
    QString filter2 = QString("field = %1").arg(0);
    modelPth->setFilter(filter2);
    modelPth->select();
    if (!bAct)
    {
        QString filter3 = QString("field = %1").arg(0);
        modelTestLog->setFilter(filter3);
        modelTestLog->select();
    }

    //mPacketInterface->setPos(id, pos.getX(), pos.getY(), pos.getYaw() * 180.0 / M_PI);


    // Execute the SQL query
    QString querystring= QString("SELECT * FROM fields WHERE location = %1").arg(selectedValue.toString());
    QSqlQuery query(querystring);

    // Loop through the query results
    while (query.next()) {
        // Access data for each record
        QString boundaryXML= query.value("boundaryXML").toString();
        QXmlStreamReader xmlData(boundaryXML);
        activeMap->loadXMLRoute(&xmlData,true);
    }
    if (activeMap->getFieldNum()>0)
    {
        std::array<double, 4> extremes_m=activeMap->findExtremeValuesFieldBorders();
        double fieldareawidth_m=extremes_m[2]-extremes_m[0];
        double fieldareaheight_m=extremes_m[3]-extremes_m[1];
        double offsetx_m=(extremes_m[2]+extremes_m[0])/2;
        double offsety_m=(extremes_m[3]+extremes_m[1])/2;
        double scalex=0.5/(fieldareawidth_m);
        double scaley=0.5/(fieldareaheight_m);

        if (bAct)
        {
        ui->mapLiveWidget->moveView(offsetx_m, offsety_m);
        ui->mapLiveWidget->setScaleFactor(std::min(scalex,scaley));
        } else
        {
        }
    activeMap->moveView(offsetx_m, offsety_m);

    activeMap->setScaleFactor(std::min(scalex,scaley));
    } else
    {
        if (bAct)
        {
        ui->mapLiveWidget->moveView(0, 0);
        // If no fields set a zoom matching a with of about 500 m -> scalefactor=0.5/500=0.001
        ui->mapLiveWidget->setScaleFactor(0.001);
        }
        activeMap->moveView(0, 0);
        // If no fields set a zoom matching a with of about 500 m -> scalefactor=0.5/500=0.001
        activeMap->setScaleFactor(0.001);
    }/*
            if (ui->pathTable->model()->rowCount()>0)
            {
                ui->pathTable->selectRow(0);
            }*/
//    mPacketInterface->setPos(0, 200, 300, 0);
    if (bAct)
    {
        on_ntripDisconnectButton_clicked();
        ntripConnect(row);
    }
    qDebug() << "onSelectedFarm END";
}

void MainWindow::onSelectedFarm(const QModelIndex& current, const QModelIndex& previous)
{
    onSelectedFarmGeneral(modelFarm,modelField,modelPath,current, previous,true);
}

void MainWindow::onSelectedFarmLog(const QModelIndex& current, const QModelIndex& previous)
{
    onSelectedFarmGeneral(modelFarmLog,modelFieldLog,modelPathLog,current, previous,false);
}

void MainWindow::onSelectedFieldGeneral(QSqlRelationalTableModel *model,QSqlRelationalTableModel *modelPth,const QModelIndex& current, const QModelIndex& previous, bool bAct)
{
    MapWidget* activeMap;
    if (bAct)
    {
        activeMap=ui->mapWidgetFields;
    } else
    {
        activeMap=ui->mapWidgetLog;
    }

    int row = current.row();

    // Retrieve the data of the selected row if needed
    int id = model->data(model->index(row, 0)).toInt();

    ui->mapWidgetFields->setFieldNow(row);

    //MapWidget *mapFields=;
    QComboBox *selectedRoute=ui->mapRouteBox;

    if (bAct)
    {
        QLabel *areaLabel=ui->label_area_ha;
        MapRoute border=activeMap->getField();
        double area=border.getArea();
        areaLabel->setText(QString::number(area));
        ui->mapLiveWidget->clearAllPaths(); // Drive widget
    };
    activeMap->clearAllPaths();

    // Construct a new query based on the selected value
    QString filter = QString("field = %1").arg(id);

    // Set the new query for the QSqlRelationalTableModel
    modelPth->setFilter(filter);
    modelPth->select();

    // Clear the existing items in the combobox
    selectedRoute->clear();

    // Iterate through the rows in the model and add items to the combobox
    for (int row = 0; row < modelPth->rowCount(); ++row) {
        // Assuming "id" is in column 0 and "name" is in column 1
        QVariant id = modelPth->data(modelPth->index(row, 0));
        QVariant name = modelPth->data(modelPth->index(row, 1));

        // Add the item to the combobox
        selectedRoute->addItem(name.toString(), id);
    }

    // Execute the SQL query
    QString querystring= QString("SELECT * FROM paths WHERE field = %1").arg(QString::number(id));
    QSqlQuery query(querystring);
    //     QMessageBox msg;
    while (query.next()) {
        // Access data for each record
        QString pathXML= query.value("xml").toString();
        QXmlStreamReader xmlData(pathXML);
        QXmlStreamReader xmlData2(pathXML);
        activeMap->loadXMLRoute(&xmlData,false);
        if (bAct)
        {
            ui->mapLiveWidget->loadXMLRoute(&xmlData2,false); // Drive-widget
        };
        //           msg.setText("Looping!");
        //           msg.exec();
    }
    if (bAct)
    {
        ui->mapLiveWidget->update(); // Drive-widget
    }
    activeMap->setBorderFocus(true);
    //       mapFields->setRouteNow();   // Make sure that no route is set automatically (in order to make it easier to edit)
}

void MainWindow::onSelectedFieldLog(const QModelIndex& current, const QModelIndex& previous)
{
    onSelectedFieldGeneral(modelFieldLog,modelPathLog,current, previous,false);
}

void MainWindow::onSelectedField(const QModelIndex& current, const QModelIndex& previous)
{
    onSelectedFieldGeneral(modelField,modelPath,current, previous,true);
};

void MainWindow::onSelectedPathGeneral(QSqlRelationalTableModel *model,const QModelIndex& current, const QModelIndex& previous, bool bAct)
{
 //   if (previous.row()!=-1)   // Something actually selected
 //   {
        ui->mapWidgetFields->setBorderFocus(false);
        ui->mapWidgetFields->update();
        ui->mapLiveWidget->setBorderFocus(false);
        ui->mapLiveWidget->update();

        int row = current.row();

        // Retrieve the data of the selected row if needed
        int id = model->data(model->index(row, 0)).toInt();
        if (bAct)
        {
        int id = model->data(model->index(row, 0)).toInt();
        QString filter = QString("path = %1").arg(id);
        modelTestLog->setFilter(filter);
        modelTestLog->select();
        }
//    }
}


void MainWindow::onSelectedPath(const QModelIndex& current, const QModelIndex& previous)
{
    onSelectedPathGeneral(modelPath,current, previous,false);
};

void MainWindow::onSelectedPathLog(const QModelIndex& current, const QModelIndex& previous)
{
    onSelectedPathGeneral(modelPathLog,current, previous,true);
};

void MainWindow::onSelectedLog(const QModelIndex& current, const QModelIndex& previous)
{
    int row = current.row();
    QString filename = modelTestLog->data(modelTestLog->index(row, 1)).toString();
    onLoadLogFile(filename);
}

void MainWindow::onLoadLogFile(QString filename)
{
    // Handle the file selection and update the line edit
//    filename = QFileDialog::getOpenFileName(this, "Open File", "", "CSV Files (*.csv);;All Files (*)");
    filename="/home/biophysics/Documents/logs/"+filename;
    QVector<LocPoint> mTrace;
    coords_polar vehicle_polar;
    coords_cartesian vehicle_enu;

    if (!filename.isEmpty()) {
        // Set the selected file path to the line edit
        ui->logfileEdit->setText(filename);

        // You can now use the selected file path for further processing
        qDebug() << "Selected file:" << filename;
        QFile file(filename);

        if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
            qDebug() << "Failed to open the file:" << file.errorString();
        }

        QTextStream in(&file);
        int a=0;
        int format=0;
        bool first=true;

        double lattmp;
        double latdgr;
        double lat;
        double lontmp;
        double londgr;
        double lon;
        double height;
        in.readLine();
        in.readLine();
        in.readLine();
        QString csvLine = in.readLine();
        qDebug() << "csvline: " << csvLine;
        QStringList substrings= csvLine.split(",");
        QString time1,time2;
        switch (substrings.size())
        {
        case 15:
            format=1;
            time1=substrings.at(1);
            break;
        case 24:
            format=2;
//            time1=substrings.at(0);
            time1=QString::number(substrings.at(1).toFloat()/100);
            break;
        }
        double llh[3]={0.0,0.0,0.0};
        double *llhp=llh;
        ui->mapWidgetLog->getEnuRef(llhp);
        coords_polar basestation_polar;
        basestation_polar.lat=llhp[0];
        basestation_polar.lon=llhp[1];
        basestation_polar.h=llhp[2];
        coords_matrix basestation_matrix=toOrientationMatrix(basestation_polar);
        coords_cartesian basestation_cartesian=toCartesian(basestation_polar);


        float time,lastTime,lastX,lastY,speed,distance,dt,dx,dy;
        while (!in.atEnd()) {
            substrings= csvLine.split(",");
  //          qDebug() << "Size:" << substrings.size();
  //          qDebug() << "Looping2";
            switch(format)
            {
            case 1:
                time=substrings.at(1).toFloat();
                lattmp=substrings.at(2).toDouble()/100;
                latdgr=std::floor(lattmp);
                lat=latdgr+(lattmp-latdgr)/0.6;
                lontmp=substrings.at(4).toDouble()/100;
                londgr=std::floor(lontmp);
                lon=londgr+(lontmp-londgr)/0.6;
                lontmp=substrings.at(4).toDouble()/100;
                height=substrings.at(9).toDouble();
                break;
            case 2:
                time=substrings.at(1).toFloat()/100;
                lat=substrings.at(19).toDouble();
                lon=substrings.at(20).toDouble();
                break;
            }
            time/=3600; // s->h
            vehicle_polar.lat=lat;
            vehicle_polar.lon=lon;
            vehicle_polar.h=height;
            vehicle_enu=toEnu(basestation_cartesian,basestation_matrix,vehicle_polar);
            LocPoint toAdd;
            toAdd.setX(vehicle_enu.x);
            toAdd.setY(vehicle_enu.y);

            if (first)
            {
                datasetSpeed.append(QPointF(time,0));
                datasetAngle.append(QPointF(time,0));
                datasetX.append(QPointF(time,vehicle_enu.x));
                datasetY.append(QPointF(time,vehicle_enu.y));
                mTrace.append(toAdd);
                first=false;
            } else
            {
                dx=vehicle_enu.x-lastX;
                dy=vehicle_enu.y-lastY;
                dt=time-lastTime;
                // Calculate the angle in radians using atan2
                double angle_rad = std::atan2(dy, dx);

                // Convert radians to degrees
                double angle_deg = angle_rad * (180.0 / M_PI);

                distance=sqrt(dx*dx+dy*dy);
                speed=distance*(0.001/3.6)/dt;
                if ((distance>0.00001) && (speed>0.00001))
                {
                    datasetX.append(QPointF(time,vehicle_enu.x));
                    datasetY.append(QPointF(time,vehicle_enu.y));
                    datasetAngle.append(QPointF(time,angle_deg));
                    datasetSpeed.append(QPointF(time,speed));
                    mTrace.append(toAdd);
                    qDebug() << "dx: " << dx << " dy: " << dy << " dt: " << dt << " speed: " << speed;
                }
            }
            lastX=vehicle_enu.x;
            lastY=vehicle_enu.y;
            lastTime=time;
//            qDebug() << "Lat.:" << lat << ", lon.: " << lon;

            // Process the line (e.g., split it by commas to get individual fields)
            csvLine = in.readLine();
            a++;
        }
        switch(format)
        {
        case 1:
            time2=substrings.at(1);
            break;
        case 2:
            time2=QString::number(substrings.at(1).toFloat()/100);
            break;
        }
        qDebug() << "time 1: " << time1 << ", time 2: " << time2;
        file.close();
        ui->mapWidgetLog->setTrace(mTrace);
        bLogLoaded=true;
        ui->horizontalSliderStart->setValue(0);
        ui->horizontalSliderEnd->setValue(99);
    } else {
        qDebug() << "No file selected!";
    }

}

MainWindow::~MainWindow()
{
    // Remove all vehicles before this window is destroyed to not get segfaults
    // in their destructors.
    while (mCars.size()) {
        QWidget *w = ui->carsWidget->currentWidget();

        if (dynamic_cast<CarInterface*>(w) != NULL) {
            CarInterface *car = (CarInterface*)w;

            ui->carsWidget->removeTab(ui->carsWidget->currentIndex());
            mCars.removeOne(car);
            delete car;
        }
    }

    delete ui;
}

bool MainWindow::eventFilter(QObject *object, QEvent *e)
{
    Q_UNUSED(object);
    if (e->type() == QEvent::KeyPress)
    {
        QKeyEvent *keyEvent = static_cast<QKeyEvent *>(e);

        // Emergency stop on escape
        if (keyEvent->key() == Qt::Key_Escape) {
            on_stopButton_clicked();
            return true;
        }
        // Handle F10 here as it won't be detected from the camera window otherwise.
        if (keyEvent->key() == Qt::Key_F10) {
            on_actionToggleCameraFullscreen_triggered();
            return true;
        }
        if (object == ui->farmTable)
          {
            qDebug() << "in farmtable";
            switch (keyEvent->key())
            {
                case Qt::Key_Delete:
                    QModelIndexList selectedRows = ui->farmTable->selectionModel()->selectedRows();
                    QSqlTableModel* model = qobject_cast<QSqlTableModel*>(ui->farmTable->model());
                    for (const QModelIndex& index : selectedRows) {
                        model->removeRow(index.row());
                    }
                    model->select();
                    return true; // Event is handled, don't propagate further
            }
        }
        if (object == ui->fieldTable)
          {
            qDebug() << "in fieldtable";
            QItemSelectionModel* selectionModel = ui->fieldTable->selectionModel();
            QModelIndexList selection = selectionModel->selectedRows();
            QSqlTableModel* model = qobject_cast<QSqlTableModel*>(ui->fieldTable->model());
            switch (keyEvent->key())
            {
                case Qt::Key_Delete:
                  // Multiple rows can be selected
                  for(int i=0; i< selection.count(); i++)
                  {
                      QModelIndex index = selection.at(i);
                      qDebug() << index.row();
                      QAbstractItemModel* connectedModel = ui->fieldTable->model();
                      QString fieldid = connectedModel->data(connectedModel->index(index.row(), 0)).toString();
                      deleteField(fieldid);
                                  //                     connectedModel->select();
                  }
                  if (model) {
                      model->select();
                  };
                  return true;
                break;
            case Qt::Key_Return:
                QByteArray byteArray;
                QXmlStreamWriter stream(&byteArray);
                ui->mapWidgetFields->saveXMLCurrentRoute(&stream);
                QString xmlString = QString::fromUtf8(byteArray);
                QModelIndexList selectedIndexes = selectionModel->selectedIndexes();
                if (selectedIndexes.isEmpty()) {
                    qDebug() << "No selection";
                }

                int row = selectedIndexes.first().row();
                qDebug() << "Selected Row: " << row;

                // Retrieve the data of the selected row if needed
                model->setData(model->index(row,4),xmlString);
                ui->fieldTable->show();
                return true;
              }
      }
        if (object == ui->pathTable)
        {
              qDebug() << "in pathdtable";
        }

    }


#ifdef HAS_JOYSTICK
    if (mJoystick->isConnected()) {
        return false;
    }
#endif

    if (ui->throttleOffButton->isChecked()) {
        return false;
    }

    return QObject::eventFilter(object, e);
//    return false;
}

void MainWindow::onMouseClickedFieldSelected(int field)
{
    qDebug() << "Field selected: " << QString::number(field) << ", time to act!";

    // Get the QSqlRelationalTableModel from the QTableView
    QSqlRelationalTableModel* model = qobject_cast<QSqlRelationalTableModel*>(this->ui->fieldTable->model());

    if (!model) {
        // The model is not a QSqlRelationalTableModel
        return;
    }

    // Find the row that matches the 'idValue' in the first column (assuming it's the primary key)
    int row = -1;
    for (int i = 0; i < model->rowCount(); ++i) {
        QModelIndex index = model->index(i, 0); // Assuming the 'id' is in column 0
        int id = model->data(index).toInt();

        if (id == field) {
            row = i;
            qDebug() << "table row: " << row;
            break;
        }
    }
    qDebug() << "Select in fieldTable";
    if (row != -1) {
        this->ui->fieldTable->selectRow(row);
    }

    qDebug() << "Acted enough!";

}


void MainWindow::addCar(int id, QString name, bool pollData)
{
    CarInterface *car = new CarInterface(this);
    mCars.append(car);
    car->setID(id);
    ui->carsWidget->addTab(car, name);
    car->setMap(ui->mapLiveWidget);
    car->setPacketInterface(mPacketInterface);
    car->setPollData(pollData);
    connect(car, SIGNAL(showStatusInfo(QString,bool)), this, SLOT(showStatusInfo(QString,bool)));
}

bool MainWindow::connectJoystick()
{
#ifdef HAS_JOYSTICK
    qDebug() << "in connectAAAAAAAAAAAAAAAAAAAAAAA";

    auto gamepads = QGamepadManager::instance()->connectedGamepads();
    if (gamepads.isEmpty()) {
        qDebug() << "Did not find any connected gamepads";
        mJoystick = new QGamepad();
        return false;
    } else
    {
    mJoystick = new QGamepad(*gamepads.begin(), this);

    connect(mJoystick, SIGNAL(buttonChanged(int,bool)),
            this, SLOT(jsButtonChanged(int,bool)));

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
    return true;
    };
    #endif

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
    return ui->mapLiveWidget;
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
                // qDebug() << "mThrottle" << mThrottle;
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
//                    qDebug() << QDateTime::currentDateTime().toString() << " - JOYSTICK (timerslot), mThrottle: " << mThrottle << ", mSteering: " << mSteering;
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

    // Update status label
    if (mStatusInfoTime) {
        mStatusInfoTime--;
    /*    if (!mStatusInfoTime) {
            mStatusLabel->setStyleSheet(qApp->styleSheet());
        }*/
    } else {
        if (mSerialPort->isOpen() || mPacketInterface->isUdpConnected() || mTcpClientMulti->isAnyConnected()) {
            mStatusLabel->setText("ConnectedXXX");
            mStatusLabel->setStyleSheet("color: green;");
        } else {
            mStatusLabel->setText("Not connectedZZZ");
            mStatusLabel->setStyleSheet("color: red;");
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

    if (next_car > largest) {
        next_car = 0;
    }

    // Update map settings
    if (ui->mapFollowBox->isChecked()) {
        ui->mapLiveWidget->setFollowCar(ui->mapCarBox->value());
    } else {
        ui->mapLiveWidget->setFollowCar(-1);
    }
    if (ui->mapTraceBox->isChecked()) {
        ui->mapLiveWidget->setTraceCar(ui->mapCarBox->value());
    } else {
        ui->mapLiveWidget->setTraceCar(-1);
    }
    ui->mapLiveWidget->setSelectedCar(ui->mapCarBox->value());

    // Joystick connected
#ifdef HAS_JOYSTICK
    static bool jsWasconn = false;
    if (mJoystick->isConnected() != jsWasconn) {
        jsWasconn = mJoystick->isConnected();

        if (jsWasconn) {
            ui->jsConnectedLabel->setText("Connected");
            ui->jsConnectedLabel->setStyleSheet("color: green;");
        } else {
            ui->jsConnectedLabel->setText("Not connected");
            ui->jsConnectedLabel->setStyleSheet("color: green;");
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
                ui->mapLiveWidget->setEnuRef(mRtcmState.pos.lat, mRtcmState.pos.lon, mRtcmState.pos.height);
                ui->mapWidgetFields->setEnuRef(mRtcmState.pos.lat, mRtcmState.pos.lon, mRtcmState.pos.height);
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
    ui->mapLiveWidget->setEnuRef(lat, lon, height);
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
                ui->mapLiveWidget->setEnuRef(i_llh[0], i_llh[1], i_llh[2]);
                ui->mapStreamNmeaZeroEnuBox->setChecked(false);
            } else {
                ui->mapLiveWidget->getEnuRef(i_llh);
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
            ui->mapLiveWidget->addInfoPoint(p);

            if (ui->mapStreamNmeaFollowBox->isChecked()) {
                ui->mapLiveWidget->moveView(p.getX(), p.getY());
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
//        qDebug() << "JS BT:" << button << pressed;

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
                                	// front up
                                    mPacketInterface->hydraulicMove(car->getId(), HYDRAULIC_POS_FRONT, HYDRAULIC_MOVE_UP);
                                } else {
//                                  // front down
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
    ui->mapLiveWidget->clearTrace();
}

void MainWindow::on_MapRemovePixmapsButton_clicked()
{
    ui->mapLiveWidget->clearPerspectivePixmaps();
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
            // Retrieve the data of the selected row if needed
            QString name = this->modelVehicle->data(this->modelVehicle->index(row, 0)).toString();
            QString ip = this->modelVehicle->data(this->modelVehicle->index(row, 1)).toString();
            QString port = this->modelVehicle->data(this->modelVehicle->index(row, 2)).toString();

            if (port.isEmpty())
            {
                mTcpClientMulti->addConnection(ip,8300);
            } else
            {
                mTcpClientMulti->addConnection(ip,port.toInt());
            };
            addCar(mCars.size(),name);
            qDebug() << "Name: " << name << ", ip: " << ip << ", port: " << port;
            oldrow=row;

        };
    }
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
    ui->mapLiveWidget->setXOffset(0);
    ui->mapLiveWidget->setYOffset(0);
}

void MainWindow::on_mapRemoveRouteButton_clicked()
{
    ui->mapLiveWidget->clearPath();
}

void MainWindow::on_mapRouteSpeedBox_valueChanged(double arg1)
{
    ui->mapLiveWidget->setRoutePointSpeed(arg1 / 3.6);
}

void MainWindow::on_jsConnectButton_clicked()
{
    connectJoystick();
}

void MainWindow::on_jsDisconnectButton_clicked()
{
}

void MainWindow::on_mapAntialiasBox_toggled(bool checked)
{
    ui->mapLiveWidget->setAntialiasDrawings(checked);
}

void MainWindow::on_carsWidget_tabCloseRequested(int index)
{
    QWidget *w = ui->carsWidget->widget(index);
    ui->carsWidget->removeTab(index);

    if (dynamic_cast<CarInterface*>(w) != NULL) {
        CarInterface *car = (CarInterface*)w;
        mCars.removeOne(car);
        delete car;
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
        CarInfo *car = ui->mapLiveWidget->getCarInfo(ui->mapCarBox->value());
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
        MapRoute r = ui->mapLiveWidget->getPath();
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

    MapRoute route;

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
            ui->mapLiveWidget->addRoutePoint(p.getX(), p.getY(), p.getSpeed(), p.getTime());
        }
    } else {
        ui->mapLiveWidget->addPath(route);
    }
}

void MainWindow::on_mapSetAbsYawButton_clicked()
{
    CarInfo *car = ui->mapLiveWidget->getCarInfo(ui->mapCarBox->value());
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
    CarInfo *car = ui->mapLiveWidget->getCarInfo(ui->mapCarBox->value());
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

    MapRoute route = ui->mapLiveWidget->getPath();
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
                    tmpList.append(route[j]);
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

    MapRoute route;
    int routeLen = 0;
    bool ok = mPacketInterface->getRoutePart(ui->mapCarBox->value(), route.size(), 10, route.mRoute, routeLen);

    QElapsedTimer timer;
    timer.start();

    while (route.size() < routeLen && ok) {
        ok = mPacketInterface->getRoutePart(ui->mapCarBox->value(), route.size(), 10, route.mRoute, routeLen);
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
            ui->mapLiveWidget->setPath(route);
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
    if (mCars.size()>0)
    {
        //Note: Moves ALL vehicles to autopilot
        for (int i = 0;i < mCars.size();i++) {
            if (mCars[i]->getId() == ui->mapCarBox->value()) {
                mCars[i]->setCtrlAp();
            }
        }
        ui->throttleOffButton->setChecked(true);
    } else
    {
        QMessageBox msg;
        msg.setText("No cars connected. Connect to one before trying again.");
        msg.exec();
    }
}

void MainWindow::on_mapKbButton_clicked()
{
    if (mJoystick->isConnected())
    {
        if (mCars.size()>0)
        {
            //Note: Moves ALL vehicles to keyboard control
            for (int i = 0;i < mCars.size();i++) {
                if (mCars[i]->getId() == ui->mapCarBox->value()) {
                    mCars[i]->setCtrlKb();
                }
            }
            ui->throttleDutyButton->setChecked(true);
        } else
        {
            QMessageBox msg;
            msg.setText("No cars connected. Connect to one before trying again.");
            msg.exec();
        }
    } else
    {
        QMessageBox msg;
        msg.setText("No joystick connected. Connect one before trying again. After you have connected it, press the connect button below");
        msg.exec();
    }
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
    MapRoute route = ui->mapLiveWidget->getPath();
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

    ui->mapLiveWidget->setPath(route);
}

void MainWindow::on_mapOpenStreetMapBox_toggled(bool checked)
{
    ui->mapLiveWidget->setDrawOpenStreetmap(checked);
    ui->mapLiveWidget->update();
}

void MainWindow::on_mapAntialiasOsmBox_toggled(bool checked)
{
    ui->mapLiveWidget->setAntialiasOsm(checked);
}

void MainWindow::on_mapOsmResSlider_valueChanged(int value)
{
    ui->mapLiveWidget->setOsmRes((double)value / 100.0);
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
                            ui->mapLiveWidget->setEnuRef(i_llh[0], i_llh[1], i_llh[2]);
                        } else {
                            ui->mapLiveWidget->getEnuRef(i_llh);
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
                        ui->mapLiveWidget->setNextEmptyOrCreateNewInfoTrace();
                    }

                    ui->mapLiveWidget->addInfoPoint(p);
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
    ui->mapLiveWidget->clearAllInfoTraces();
}

void MainWindow::on_traceInfoMinZoomBox_valueChanged(double arg1)
{
    ui->mapLiveWidget->setInfoTraceTextZoom(arg1);
}

void MainWindow::on_removeRouteExtraButton_clicked()
{
    on_mapRemoveRouteButton_clicked();
}

void MainWindow::on_mapOsmClearCacheButton_clicked()
{
    ui->mapLiveWidget->osmClient()->clearCache();
    ui->mapLiveWidget->update();
}

void MainWindow::on_mapOsmServerOsmButton_toggled(bool checked)
{
    if (checked) {
                ui->mapLiveWidget->osmClient()->setTileServerUrl("http://tile.openstreetmap.org");
                ui->mapLiveWidget->osmClient()->setType(2);
    }
}

void MainWindow::on_mapOsmServerHiResButton_toggled(bool checked)
{
    if (checked) {
                ui->mapLiveWidget->osmClient()->setTileServerUrl("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile");
                ui->mapLiveWidget->osmClient()->setType(2);
    	//        ui->mapWidget->osmClient()->setTileServerUrl("http://c.osm.rrze.fau.de/osmhd"); // Also https
    }
}

void MainWindow::on_mapOsmServerVedderButton_toggled(bool checked)
{
    if (checked) {
//        ui->mapWidget->osmClient()->setTileServerUrl("http://tiles.vedder.se/osm_tiles");
//        ui->mapWidget->osmClient()->setType(2);
                        ui->mapLiveWidget->osmClient()->setTileServerUrl("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile");
                        ui->mapLiveWidget->osmClient()->setType(1);
    }
}

void MainWindow::on_mapOsmServerVedderHdButton_toggled(bool checked)
{
    if (checked) {
        ui->mapLiveWidget->osmClient()->setTileServerUrl("http://tiles.vedder.se/osm_tiles_hd");
        ui->mapLiveWidget->osmClient()->setType(2);
    }
}

void MainWindow::on_mapOsmMaxZoomBox_valueChanged(int arg1)
{
    ui->mapLiveWidget->setOsmMaxZoomLevel(arg1);
}

void MainWindow::on_mapDrawGridBox_toggled(bool checked)
{
    ui->mapLiveWidget->setDrawGrid(checked);
}

void MainWindow::on_mapGetEnuButton_clicked()
{
    mPacketInterface->getEnuRef(ui->mapCarBox->value());
}

void MainWindow::on_mapSetEnuButton_clicked()
{
    double llh[3];
    ui->mapLiveWidget->getEnuRef(llh);
    mPacketInterface->setEnuRef(ui->mapCarBox->value(), llh);
}

void MainWindow::on_mapOsmStatsBox_toggled(bool checked)
{
    ui->mapLiveWidget->setDrawOsmStats(checked);
}

void MainWindow::on_removeTraceExtraButton_clicked()
{
    ui->mapLiveWidget->clearTrace();
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
    ui->mapLiveWidget->clearInfoTrace();
}

void MainWindow::on_mapRouteBox_valueChanged(int arg1)
{
    ui->mapLiveWidget->setPathNow(arg1);
}

void MainWindow::on_mapRemoveRouteAllButton_clicked()
{
    ui->mapLiveWidget->clearAllPaths();
}

void MainWindow::on_mapUpdateTimeButton_clicked()
{
    bool ok;
    int res = QInputDialog::getInt(this,
                                   tr("Set new route start time"),
                                   tr("Seconds from now"), 30, 0, 60000, 1, &ok);

    if (ok) {
        MapRoute route = ui->mapLiveWidget->getPath();
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

        ui->mapLiveWidget->setPath(route);
    }
}

void MainWindow::on_mapRouteTimeEdit_timeChanged(const QTime &time)
{
    ui->mapLiveWidget->setRoutePointTime(time.msecsSinceStartOfDay());
}

void MainWindow::on_mapTraceMinSpaceCarBox_valueChanged(double arg1)
{
    ui->mapLiveWidget->setTraceMinSpaceCar(arg1 / 1000.0);
}

void MainWindow::on_mapTraceMinSpaceGpsBox_valueChanged(double arg1)
{
    ui->mapLiveWidget->setTraceMinSpaceGps(arg1 / 1000.0);
}

void MainWindow::on_mapInfoTraceBox_valueChanged(int arg1)
{
    ui->mapLiveWidget->setInfoTraceNow(arg1);
}

void MainWindow::on_removeInfoTraceExtraButton_clicked()
{
    ui->mapLiveWidget->clearInfoTrace();
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
        int res = utility::loadRoutes(filename, ui->mapLiveWidget);

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

    MapRoute route = ui->mapLiveWidget->getPath();

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

        MapRoute route;

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
        MapRoute routeReduced;
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

        ui->mapLiveWidget->addPath(routeReduced);

        file.close();
        showStatusInfo("Loaded drive file", true);
    }
}

void MainWindow::on_modeRouteButton_toggled(bool checked)
{
    ui->mapLiveWidget->setAnchorMode(!checked);
}

void MainWindow::on_uploadAnchorButton_clicked()
{
    QVector<UWB_ANCHOR> anchors;
    for (LocPoint p: ui->mapLiveWidget->getAnchors()) {
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
    ui->mapLiveWidget->setAnchorId(arg1);
}

void MainWindow::on_anchorHeightBox_valueChanged(double arg1)
{
    ui->mapLiveWidget->setAnchorHeight(arg1);
}

void MainWindow::on_removeAnchorsButton_clicked()
{
    ui->mapLiveWidget->clearAnchors();
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
    ui->mapLiveWidget->saveXMLRoutes(&stream,withId);
    file.close();
    showStatusInfo("Saved routes", true);
}

void MainWindow::on_mapDrawRouteTextBox_toggled(bool checked)
{
    ui->mapLiveWidget->setDrawRouteText(checked);
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
    ui->mapLiveWidget->setDrawUwbTrace(checked);
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
    ui->mapLiveWidget->setCameraImageWidth(arg1);
}

void MainWindow::on_mapCameraOpacityBox_valueChanged(double arg1)
{
    ui->mapLiveWidget->setCameraImageOpacity(arg1);
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
        ui->mapLiveWidget->setFocus();
    }
}

void MainWindow::on_routeZeroButton_clicked()
{
    ui->mapLiveWidget->zoomInOnRoute(ui->mapRouteBox->currentIndex(), 0.1);
}

void MainWindow::on_routeZeroAllButton_clicked()
{
    ui->mapLiveWidget->zoomInOnRoute(-1, 0.1);
}

void MainWindow::on_mapRoutePosAttrBox_currentIndexChanged(int index)
{
    quint32 attr = ui->mapLiveWidget->getRoutePointAttributes();
    attr &= ~ATTR_POSITIONING_MASK;
    attr |= index;
    ui->mapLiveWidget->setRoutePointAttributes(attr);
}

void MainWindow::on_clearAnchorButton_clicked()
{
    mPacketInterface->clearUwbAnchors(ui->mapCarBox->value());
}


void MainWindow::on_boundsFillPushButton_clicked()
{

    MapRoute bounds = ui->mapWidgetFields->getField(ui->mapWidgetFields->getFieldNow());
    double spacing = ui->boundsFillSpacingSpinBox->value();
    if (spacing < 0.5) return;

    QList<LocPoint> routeLP;
    if (ui->generateFrameCheckBox->isChecked())
    {
        routeLP = RouteMagic::fillConvexPolygonWithFramedZigZag(bounds.mRoute, spacing, ui->boundsFillKeepTurnsInBoundsCheckBox->isChecked(), ui->boundsFillSpeedSpinBox->value()/3.6,
                                                              ui->boundsFillSpeedInTurnsSpinBox->value()/3.6, ui->stepsForTurningSpinBox->value(), ui->visitEverySpinBox->value(),
                                                              ui->lowerToolsCheckBox->isChecked() ? ATTR_HYDRAULIC_FRONT_DOWN : 0, ui->raiseToolsCheckBox->isChecked() ? ATTR_HYDRAULIC_FRONT_UP : 0,
                                                              ui->lowerToolsDistanceSpinBox->value()*2, ui->raiseToolsDistanceSpinBox->value()*2);
                                                              // attribute changes at half distance
    } else
    {
        routeLP = RouteMagic::fillConvexPolygonWithZigZag(bounds.mRoute, spacing, ui->boundsFillKeepTurnsInBoundsCheckBox->isChecked(), ui->boundsFillSpeedSpinBox->value()/3.6,
                                                        ui->boundsFillSpeedInTurnsSpinBox->value()/3.6, ui->stepsForTurningSpinBox->value(), ui->visitEverySpinBox->value(),
                                                        ui->lowerToolsCheckBox->isChecked() ? ATTR_HYDRAULIC_FRONT_DOWN : 0, ui->raiseToolsCheckBox->isChecked() ? ATTR_HYDRAULIC_FRONT_UP : 0,
                                                        ui->lowerToolsDistanceSpinBox->value()*2, ui->raiseToolsDistanceSpinBox->value()*2);
    };


    MapRoute route;
    route.mRoute=routeLP;

    ui->mapWidgetFields->addPath(route);
    int r = ui->mapWidgetFields->getPaths().size()-1;
    ui->mapWidgetFields->setPathNow(r);
    ui->mapRouteBox->setCurrentIndex(r);
    ui->mapWidgetFields->repaint();
}

void MainWindow::on_lowerToolsCheckBox_stateChanged(int arg1)
{
    ui->lowerToolsDistanceSpinBox->setEnabled(arg1 != 0);
}

void MainWindow::on_raiseToolsCheckBox_stateChanged(int arg1)
{
    ui->raiseToolsDistanceSpinBox->setEnabled(arg1 != 0);
}

void MainWindow::on_actionWireguard_triggered()
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
    {
        ui->wgStatusLabel->setText("Connected");
        ui->wgStatusLabel->setStyleSheet("color: green;");
    }
    else
    {
        ui->wgStatusLabel->setText("Status: Config. error");
    };
}

void MainWindow::on_WgDisconnectPushButton_clicked()

{
    system("pkexec wg-quick down wg_sdvp");
    QList<QNetworkInterface> interfaces = QNetworkInterface::allInterfaces();
    if (std::find_if(interfaces.begin(), interfaces.end(),
                     [](QNetworkInterface currInterface){return currInterface.name() == "wg_sdvp";}) != interfaces.end())
    {
        ui->wgStatusLabel->setText("Connected");
        ui->wgStatusLabel->setStyleSheet("color: green;");
    }
    else
    {
        ui->wgStatusLabel->setText("Disconnected");
        ui->wgStatusLabel->setStyleSheet("color: red;");
    };
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


void MainWindow::showError(const QSqlError &err)
{
    QMessageBox::critical(this, "Unable to initialize Database",
                "Error initializing database: " + err.text());
}

void MainWindow::handleAddFieldButton()
{
    QMessageBox msgBox;
    msgBox.setText("Adding field:" + ui->fieldnameEdit->text());
    msgBox.exec();
    QModelIndexList selectedIndexes = ui->farmTable->selectionModel()->selectedIndexes();
    if (!selectedIndexes.isEmpty()) {
        int rowIndex = selectedIndexes.first().row();
        int id = modelFarm->record(rowIndex).value("id").toInt();
        addField(ui->fieldnameEdit->text(), id);
    } else {
        msgBox.setText("No row selected!");
        msgBox.exec();
    }
    modelField->select();
    //ui->fieldTable->update();
}


void MainWindow::handleAddFarmButton()
{
//    QMessageBox msgBox;
//    msgBox.setText("Adding farm:" + ui->locationnameEdit->text());
//    msgBox.exec();
    addLocation(ui->locationnameEdit->text());
    QSqlTableModel *tableModel = qobject_cast<QSqlTableModel*>(ui->farmTable->model());
    if (tableModel) {
        tableModel->select(); // Re-fetch the data from the database to update the model
    }
}

void MainWindow::handleImportPathButton()
{
    QString qstrPathname=ui->pathnameEdit->text();
    if (qstrPathname.trimmed().isEmpty())
    {
    QMessageBox msgBox;
    msgBox.setText("You need to fill in path name before trying to import a path");
    msgBox.exec();
    } else
    {
    QString filename = QFileDialog::getOpenFileName(this,
                                                    tr("Load Routes"), "",
                                                    tr("Xml files (*.xml)"));

    if (!filename.isEmpty()) {
        int res = 0;

        QFile file(filename);
        if (!file.open(QIODevice::ReadOnly)) {
                QMessageBox::critical(this, "Load Routes",
                                      "Could not open\n" + filename + "\nfor reading");
                return;
        } else
        {
        QString xmlString = QTextStream(&file).readAll();


        QModelIndexList selectedIndexes = ui->fieldTable->selectionModel()->selectedIndexes();
        if (selectedIndexes.isEmpty()) {
        qDebug() << "No selection";
        }

        int row = selectedIndexes.first().row();
        QSqlRelationalTableModel* model = qobject_cast<QSqlRelationalTableModel*>(this->ui->fieldTable->model());
        int id = model->data(model->index(row, 0)).toInt();
        qDebug() << "Selected Id: " << id;
        addPath(qstrPathname,xmlString,id);

/*
        // Retrieve the data of the selected row if needed
        QSqlRelationalTableModel* model = qobject_cast<QSqlRelationalTableModel*>(this->ui->pathTable->model());
        model->setData(model->index(row,4),xmlString);
        ui->fieldTable->show();
        return true;
        */
        }
        }
    }
};


QString readXmlToString(const QString& filePath) {
    QString xmlString;

    QFile file(filePath);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        // Handle error if file couldn't be opened
        return QString();
    }

    QXmlStreamReader stream(&file);
    while (!stream.atEnd() && !stream.hasError()) {
        // Read XML content element by element
        if (stream.isStartElement() || stream.isCharacters()) {
        xmlString.append(stream.text());
        }
        stream.readNext();
    }

    if (stream.hasError()) {
        // Handle error if XML parsing error occurred
        return QString();
    }

    // Close the file
    file.close();

    return xmlString;
}

void addPath(const QString &pathname, const QString &xmlstring,const QVariant &fieldId)
{
    const auto INSERT_FIELD_SQL = QLatin1String(R"(
            insert into paths(name,field,xml)
                              values(?,?,?)
            )");
    QSqlQuery q;
    if (q.prepare(INSERT_FIELD_SQL))
    {
        q.addBindValue(pathname);
        q.addBindValue(fieldId);
        q.addBindValue(xmlstring);
        q.exec();
    };
}


void addField(const QString &name, const QVariant &locationId)
{
    const auto INSERT_FIELD_SQL = QLatin1String(R"(
            insert into fields(name,location)
                              values(?,?)
            )");
    QSqlQuery q;
    if (q.prepare(INSERT_FIELD_SQL))
    {
        q.addBindValue(name);
        q.addBindValue(locationId);
        q.exec();
    };

}

void deleteField(const QVariant &fieldId)
{
    const auto DELETE_FIELD_SQL = QLatin1String(R"(DELETE FROM fields WHERE id=:fieldid)");
    QSqlQuery q;
    if (q.prepare(DELETE_FIELD_SQL))
    {
        q.bindValue(":fieldid",fieldId);
        q.exec();
        qDebug() << "DELETE FIELD!!!: " << fieldId;
    } else {
        qDebug() << "Oups..";
    };
}


QVariant addLocation(const QString &name)
{
    const auto INSERT_LOCATION_SQL = QLatin1String(R"(
        insert into locations(name)
                          values(?)
        )");
    QSqlQuery q;
    if (q.prepare(INSERT_LOCATION_SQL))
    {
        q.addBindValue(name);
        q.exec();
        return q.lastInsertId();
    };

}

QSqlError initDb()
{
    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");
    db.setDatabaseName("test2.db");
    if (!db.open())
        return db.lastError();
    return QSqlError();
}











void MainWindow::setRefPos(double lat, double lon, double height, double antenna_height)
{
    ui->refSendLatBox->setValue(lat);
    ui->refSendLonBox->setValue(lon);
    ui->refSendHBox->setValue(height);
    ui->refSendAntHBox->setValue(antenna_height);
}

void MainWindow::timerSlotRtcm()
{
    // Update ntrip connected label
    static bool wasNtripConnected = false;
    if (wasNtripConnected != mRtcm->isTcpConnected()) {
        wasNtripConnected = mRtcm->isTcpConnected();

        if (wasNtripConnected) {
            ui->ntripConnectedLabel->setText("Connected");
            ui->ntripConnectedLabel->setStyleSheet("color: green;");
        } else {
            ui->ntripConnectedLabel->setText("Not connected");
            ui->ntripConnectedLabel->setStyleSheet("color: red;");

        }
    }
    // Send reference position every 5s
    if (ui->sendRefPosBox->isChecked()) {
        static int cnt = 0;
        cnt++;
        if (cnt >= (5000 / mTimerRtcm->interval())) {
            cnt = 0;
            QByteArray data = RtcmClient::encodeBasePos(
                        ui->refSendLatBox->value(),
                        ui->refSendLonBox->value(),
                        ui->refSendHBox->value(),
                        ui->refSendAntHBox->value());

            emit rtcmReceivedStep1(data);
            mTcpServerRtcm->broadcastData(data);
        }
    }
}

void MainWindow::rtcmRx(QByteArray data, int type, bool sync)
{
    (void)sync;
    //Commented away for the moment. Will be logged
    //qDebug() << "Type: " << type << ", data:" << data;
    emit rtcmReceivedStep1(data);
}

void MainWindow::refPosRx(double lat, double lon, double height, double antenna_height)
{
    QString str;
    str.sprintf("Lat:            %.8f\n"
                "Lon:            %.8f\n"
                "Height:         %.3f\n"
                "Antenna Height: %.3f",
                lat, lon, height, antenna_height);
    ui->lastRefPosLablel->setText(str);
}


void MainWindow::ntripConnect(int rowIndex)
{
    QString  ip = modelFarm->data(modelFarm->index(rowIndex, 4)).toString();
    uint  port = modelFarm->data(modelFarm->index(rowIndex, 5)).toUInt();
    bool  NTRIP = modelFarm->data(modelFarm->index(rowIndex, 6)).toBool();
    QString  user = modelFarm->data(modelFarm->index(rowIndex, 7)).toString();
    QString  pwd = modelFarm->data(modelFarm->index(rowIndex, 8)).toString();
    QString  stream = modelFarm->data(modelFarm->index(rowIndex, 5)).toString();
/*    QMessageBox msgBox;
    msgBox.setText("In Ntrip Connect, selected ID:" + QString::number(rowIndex) + ':' + ip + ':' + port);
    msgBox.exec();*/
    if (NTRIP) {
        mRtcm->connectNtrip(ip, stream, user, pwd, port);
    } else {
        mRtcm->connectTcp(ip, port);
    }
}

void MainWindow::on_ntripConnectButton_clicked()
{
    QModelIndexList selectedIndexes = ui->farmTable->selectionModel()->selectedIndexes();
    if (!selectedIndexes.isEmpty()) {
        int rowIndex = selectedIndexes.first().row();
//        int id = modelFarm->record(rowIndex).value("id").toInt();
        ntripConnect(rowIndex);
    } else
    {
        QMessageBox msgBox;
        msgBox.setText("You need to select a farm before you connect!");
        msgBox.exec();

    }
}

void MainWindow::on_ntripDisconnectButton_clicked()
{
    mRtcm->disconnectTcpNtrip();
}


void MainWindow::on_refGetButton_clicked()
{
    emit refPosGet();
}

void MainWindow::on_tcpServerBox_toggled(bool checked)
{
    if (checked) {
        if (!mTcpServerRtcm->startTcpServer(ui->tcpServerPortBox->value())) {
            QMessageBox::warning(this, "TCP Server Error",
                                 "Creating TCP server for RTCM data failed. Make sure that the port is not "
                                 "already in use.");
            ui->tcpServerBox->setChecked(false);
        }
    } else {
        mTcpServerRtcm->stopServer();
    }
}

void MainWindow::on_gpsOnlyBox_toggled(bool checked)
{
    mRtcm->setGpsOnly(checked);
}

CustomDelegate::CustomDelegate(QObject *parent) : QStyledItemDelegate(parent)
{

}


void CustomDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    QStyleOptionViewItem newOption = option;

    // Check if the text color role is set in the data
    if (index.column() == 1 && index.data(Qt::TextColorRole).isValid()) {
        newOption.palette.setColor(QPalette::Text, index.data(Qt::TextColorRole).value<QColor>());
    }

    QStyledItemDelegate::paint(painter, newOption, index);



    /*
    QStyleOptionViewItem newOption = option;

    // Set the text color to red for the second column (column 1)
    if (index.column() == 1) {
        newOption.palette.setColor(QPalette::Text, Qt::red);
    }

    QStyledItemDelegate::paint(painter, newOption, index);*/
}

bool FocusEventFilter::eventFilter(QObject* watched, QEvent* event)
{
    if (event->type() == QEvent::FocusIn)
    {
        emit focusGained();
    }
    else if (event->type() == QEvent::FocusOut)
    {
        emit focusLost();
    }

    return QObject::eventFilter(watched, event);
};

coords_matrix toOrientationMatrix(coords_polar cp)
{
float so = sinf(cp.lon * M_PI / 180.0);
float co = cosf(cp.lon * M_PI / 180.0);
float sa = sinf(cp.lat * M_PI / 180.0);
float ca = cosf(cp.lat * M_PI / 180.0);

coords_matrix retur;
// ENU
retur.r1c1 = -so;
retur.r1c2 = co;
retur.r1c3 = 0.0;

retur.r2c1 = -sa * co;
retur.r2c2 = -sa * so;
retur.r2c3 = ca;

retur.r3c1 = ca * co;
retur.r3c2 = ca * so;
retur.r3c3 = sa;
return retur;
}

coords_cartesian toCartesian(coords_polar cp)
{
    double sinp = sin(cp.lat * D_PI / D(180.0));
    double cosp = cos(cp.lat * D_PI / D(180.0));
    double sinl = sin(cp.lon * D_PI / D(180.0));
    double cosl = cos(cp.lon * D_PI / D(180.0));
    double e2 = FE_WGS84 * (2.0 - FE_WGS84);
    double v = RE_WGS84 / sqrt(1.0 - e2 * sinp * sinp);

    coords_cartesian retur;

    retur.x = (v + cp.h) * cosp * cosl;
    retur.y = (v + cp.h) * cosp * sinl;
    retur.z = (v * (1.0 - e2) + cp.h) * sinp;
    return retur;

}

coords_cartesian toEnu(coords_polar basestation,coords_polar vehicle)
{
    // Convert llh to ecef
    coords_cartesian basestation_cartesian=toCartesian(basestation);
    coords_cartesian vehicle_cartesian=toCartesian(vehicle);


    // Continue if ENU frame is initialized
    float dx = vehicle_cartesian.x - basestation_cartesian.x;
    float dy = vehicle_cartesian.y - basestation_cartesian.y;
    float dz = vehicle_cartesian.z - basestation_cartesian.z;

    coords_cartesian retur;
    coords_matrix orientation=toOrientationMatrix(basestation);

    retur.x = orientation.r1c1 * dx + orientation.r1c2 * dy + orientation.r1c3 * dz;
    retur.y = orientation.r2c1 * dx + orientation.r2c2 * dy + orientation.r2c3 * dz;
    retur.z = orientation.r3c1 * dx + orientation.r3c2 * dy + orientation.r3c3 * dz;

    return retur;
}

coords_cartesian toEnu(coords_cartesian basestation,coords_matrix orientation,coords_polar vehicle)
{
    // Convert llh to ecef
    coords_cartesian vehicle_cartesian=toCartesian(vehicle);


    // Continue if ENU frame is initialized
    float dx = vehicle_cartesian.x - basestation.x;
    float dy = vehicle_cartesian.y - basestation.y;
    float dz = vehicle_cartesian.z - basestation.z;

    coords_cartesian retur;
    retur.x = orientation.r1c1 * dx + orientation.r1c2 * dy + orientation.r1c3 * dz;
    retur.y = orientation.r2c1 * dx + orientation.r2c2 * dy + orientation.r2c3 * dz;
    retur.z = orientation.r3c1 * dx + orientation.r3c2 * dy + orientation.r3c3 * dz;

    return retur;
}
