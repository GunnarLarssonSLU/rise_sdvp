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
#include <QLoggingCategory>
#include <QtSql>
#include <QtCharts>
#include <QtWidgets>
#include <QDir>
#include <ogrsf_frmts.h>
#include <iostream>
#include <fstream>

//using namespace QtCharts;

#include <QtCharts/QChartView>
#include <QtCharts/QLineSeries>
#include <QListView>
#include <QStringListModel>


#include "utility.h"
#include "routemagic.h"
#include "wireguard.h"
#include "attributes_masks.h"
#include "datatypes.h"
#include "arduinoreader.h"
#include "checkboxdelegate.h"

#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
    #include <SDL2/SDL.h>
#else
    #include <QGamepad>
#endif

#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
#define FRONT_UP 5
#define FRONT_DOWN 7
#define REAR_UP 4
#define REAR_DOWN 6
// 7: Front Up
// 5: Front Down
// 6: Rear up
// 4: Rear down
#else
#define FRONT_UP 5
#define FRONT_DOWN 7
#define REAR_UP 4
#define REAR_DOWN 6
// 5: Front Up
// 7: Front Down
// 4: Rear up
// 6: Rear down
#endif




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
    serialReader("/dev/arduino", 9600),
    ui(new Ui::MainWindow),
    db(this)
{
    ui->setupUi(this);
    ui->mapLiveWidget->setMousePressEventHandler([this](QMouseEvent *e) {
        ui->mapLiveWidget->mousePressEventPaths(e);
    });
    ui->mapLiveWidget->setMouseReleaseEventHandler([this](QMouseEvent *e) {
        ui->mapLiveWidget->mouseReleaseEventPaths(e);
    });
    ui->mapLiveWidget->setWheelEventHandler([this](QWheelEvent *e) {
        ui->mapLiveWidget->wheelEventPaths(e);
    });

    ui->mapWidgetFields->setMousePressEventHandler([this](QMouseEvent *e) {
        ui->mapWidgetFields->mousePressEventFields(e);
    });
    ui->mapWidgetFields->setMouseReleaseEventHandler([this](QMouseEvent *e) {
        ui->mapWidgetFields->mousePressEventFields(e);
    });

    ui->mapWidgetFields->setWheelEventHandler([this](QWheelEvent *e) {
        ui->mapWidgetFields->wheelEventFields(e);
    });

    ui->mapWidgetAnalysis->setMousePressEventHandler([this](QMouseEvent *e) {
        ui->mapWidgetAnalysis->mousePressEventAnalysis(e);
    });
    ui->mapWidgetAnalysis->setMouseReleaseEventHandler([this](QMouseEvent *e) {
        ui->mapWidgetAnalysis->mousePressEventFields(e);
    });

    ui->mapWidgetAnalysis->setWheelEventHandler([this](QWheelEvent *e) {
        ui->mapWidgetAnalysis->wheelEventFields(e);
    });

    ui->mapWidgetFields->setMainWindow(this);
    ui->mapWidgetFields->setBorderFocus(true);
    ui->mapWidgetAnalysis->setAnalysisActive(true);

    fileModel = new QStringListModel(this);
    ui->listLogfilesView->setModel(fileModel);  // Link the model to the view

    mVersion = "0.8";
    mSupportedFirmwares.append(qMakePair(12, 3));
    mSupportedFirmwares.append(qMakePair(20, 1));
    mSupportedFirmwares.append(qMakePair(30, 1));

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

//    static MainWindow *mThis=this;          // Just to be able to get the lambdas to work
#ifdef HAS_JOYSTICK
    // Connect joystick by default
    bool connectJs = connectJoystick();

    if (connectJs) {
        on_jsConnectButton_clicked();
    }
#endif

    checkboxdelegate=new CheckBoxDelegate(ui->fieldTable);

    mPing = new Ping(this);
    mNmea = new NmeaServer(this);
    mUdpSocket = new QUdpSocket(this);
    mTcpClientMulti = new TcpClientMulti(this);
    mUdpSocket->setSocketOption(QAbstractSocket::LowDelayOption, true);

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
    ui->baseStationWidget->setMap(ui->mapLiveWidget);

    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));
    connect(mSerialPort, SIGNAL(readyRead()),
            this, SLOT(serialDataAvailable()));
/*    connect(mSerialPort, SIGNAL(error(QSerialPort::SerialPortError)),
            this, SLOT(serialPortError(QSerialPort::SerialPortError)));*/
    connect(mHeartbeatTimer, SIGNAL(timeout()), this, SLOT(sendHeartbeat()));
    connect(mPacketInterface, SIGNAL(dataToSend(QByteArray&)),
            this, SLOT(packetDataToSend(QByteArray&)));
    connect(mPacketInterface, SIGNAL(stateReceived(quint8,CAR_STATE)),
            this, SLOT(stateReceived(quint8,CAR_STATE)));
    connect(ui->mapLiveWidget, SIGNAL(posSet(quint8,LocPoint)),
            this, SLOT(mapPosSet(quint8,LocPoint)));
    connect(mPacketInterface, SIGNAL(ackReceived(quint8,CMD_PACKET,QString)),
            this, SLOT(ackReceived(quint8,CMD_PACKET,QString)));
    connect(ui->rtcmWidget, SIGNAL(rtcmReceived(QByteArray)),
            this, SLOT(rtcmReceived(QByteArray)));
    connect(ui->baseStationWidget, SIGNAL(rtcmOut(QByteArray)),
            this, SLOT(rtcmReceived(QByteArray)));
    connect(ui->rtcmWidget, SIGNAL(refPosGet()), this, SLOT(rtcmRefPosGet()));
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

    connect(ui->buttonGenerate, &QPushButton::clicked, this, &MainWindow::onGeneratePathButtonClicked);
    connect(ui->buttonGenerateLine, &QPushButton::clicked, this, &MainWindow::onGenerateLineButtonClicked);
    // Connect the button's clicked signal to a lambda that opens a file dialog
    connect(ui->pushButton_load_shapefile, &QPushButton::clicked, this, &MainWindow::onLoadShapefile);
    connect(ui->pushButtonLoadLogFile, &QPushButton::clicked, this, &MainWindow::onLoadLogfile);
    connect(ui->listLogfilesView, &QListView::clicked, this, &MainWindow::on_listLogFilesView_clicked);

    connect(ui->actionAboutQt, SIGNAL(triggered(bool)),
            qApp, SLOT(aboutQt()));

    connect(mTcpClientMulti, &TcpClientMulti::packetRx, [this](QByteArray data) {
        mPacketInterface->processPacket((unsigned char*)data.data(), data.size());
    });

    connect(mTcpClientMulti, &TcpClientMulti::stateChanged, [this](QString msg, QString ip, bool isError) {
        showStatusInfo(msg, !isError);

        if (isError) {
            qWarning() << "TCP Error:" << msg << ", ip: " << ip;
            QString all=msg  + ", ip: " + ip;
            QMessageBox::warning(this, "TCP Error", all);
            /*
            for (int i = 0; i < ui->carsWidget->count(); ++i) {
                if (ui->carsWidget->tabText(i) == ip) {
                    ui->carsWidget->removeTab(i);
                    break; // Exit the loop once the tab is found and removed
                }
            }*/
        }
    });
    QObject::connect(&serialReader, &ArduinoReader::signalLost, [this]() {
            on_stopButton_clicked();
            qWarning() << "Signal lost! Did not receive '1'.";
    });

#ifdef HAS_SIM_SCEN
    mSimScen = new PageSimScen;
    ui->mainTabWidget->addTab(mSimScen, QIcon(":/models/Icons/Sedan-96.png"), "");
    ui->mainTabWidget->setTabToolTip(ui->mainTabWidget->count() - 1,
                                     "Simulation Scenarios");
#endif

    ui->mainTabWidget->removeTab(8);
    ui->mainTabWidget->removeTab(7);
    ui->mainTabWidget->removeTab(6);
    ui->mainTabWidget->removeTab(5);
    ui->mainTabWidget->removeTab(4);

    if (!QSqlDatabase::drivers().contains("QSQLITE"))
            QMessageBox::critical(
                this,
                "Unable to load database",
                "This program needs the SQLITE driver"
                );

    modelFarm=setupFarmTable(ui->farmTable,"locations");
    modelField=setupFieldTable(ui->fieldTable,"fields");
    modelPath=setupPathTable(ui->pathTable,"paths");

    // Connect the signal from the first table view to a custom slot
    QObject::connect(ui->farmTable->selectionModel(), &QItemSelectionModel::currentChanged, this, &MainWindow::onSelectedFarm);
    QObject::connect(ui->fieldTable->selectionModel(), &QItemSelectionModel::currentChanged, this, &MainWindow::onSelectedField);

//    ui->fieldTable->installEventFilter(&filterFieldtable);

    MapWidget *mapFields=ui->mapWidgetFields;

    /*
    QObject::connect(&filterFieldtable, &FocusEventFilter::focusGained, [mapFields]() {
        qDebug() << "Focus gained Fields";
    });
*/
    /* ui->pathTable->installEventFilter(&filterPathtable);

  QObject::connect(&filterPathtable, &FocusEventFilter::focusGained, [mapFields]() {
      mapFields->setBorderFocus(false);
      mapFields->update();
      qDebug() << "Focus gained Paths";
  });
*/
    connect(ui->pushButton_farm, &QPushButton::released, this, &MainWindow::handleAddFarmButton);
    connect(ui->pushButton_field, &QPushButton::released, this, &MainWindow::handleAddFieldButton);

//    ui->farmTable->setFocus();
    ui->farmTable->installEventFilter(this);    
    ui->fieldTable->installEventFilter(this);

    //    ui->farmTable->selectRow(0);

    if (ui->fieldTable->model()->rowCount()>0)
    {
        ui->fieldTable->selectRow(0);
    }
    if (ui->pathTable->model()->rowCount()>0)
    {
        ui->pathTable->selectRow(0);
    }

    qApp->installEventFilter(this);
}

MainWindow::~MainWindow()
{
    // Remove all vehicles before this window is destroyed to not get segfaults
    // in their destructors.
    while (mCars.size() > 0) {
        QWidget *w = ui->carsWidget->currentWidget();

        if (dynamic_cast<CarInterface*>(w) != NULL) {
            CarInterface *car = (CarInterface*)w;

            ui->carsWidget->removeTab(ui->carsWidget->currentIndex());
            mCars.removeOne(car);
            delete car;
        }
    }

#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
    if (mController) {
        SDL_GameControllerClose(mController);
    }
    SDL_Quit();
#endif
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
                    db.deleteField(fieldid);
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
    }

#ifdef HAS_JOYSTICK
    if (JSconnected()) {
        return false;
    }

#endif

    if (ui->throttleOffButton->isChecked()) {
        return false;
    }

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

    return false;
}

void MainWindow::updateFarms()
{
    modelFarm->select();
}

QSqlRelationalTableModel* MainWindow::setupFarmTable(QTableView* uiFarmTable,QString sqlTablename)
{
    // Create the data model:
    QSqlRelationalTableModel* model = new QSqlRelationalTableModel(uiFarmTable);
    model->setEditStrategy(QSqlTableModel::OnFieldChange);
    model->setTable(sqlTablename);

    // Set the localized header captions:
    model->setHeaderData(model->fieldIndex("name"), Qt::Horizontal, tr("Name"));
    model->setHeaderData(model->fieldIndex("longitude"), Qt::Horizontal, tr("Longitude"));
    model->setHeaderData(model->fieldIndex("latitude"), Qt::Horizontal, tr("Latitude"));
    model->setHeaderData(model->fieldIndex("ip"), Qt::Horizontal, tr("ip"));
    model->setHeaderData(model->fieldIndex("port"), Qt::Horizontal, tr("port"));
    model->setHeaderData(model->fieldIndex("NTRIP"), Qt::Horizontal, tr("NTRIP"));
    model->setHeaderData(model->fieldIndex("user"), Qt::Horizontal, tr("user"));
    model->setHeaderData(model->fieldIndex("password"), Qt::Horizontal, tr("password"));

    // Populate the model:
    if (!model->select()) {
        db.showError(model->lastError());
        return model;
    }

    // Set the model and hide the ID column:
    uiFarmTable->setModel(model);
    //ui.locationTable->setItemDelegate(new BookDelegate(ui.locationTable));
    uiFarmTable->setColumnHidden(model->fieldIndex("id"), true);

//    uiFarmTable->setColumnHidden(model->fieldIndex("longitude"), true);
//    uiFarmTable->setColumnHidden(model->fieldIndex("latitude"), true);
    uiFarmTable->setColumnHidden(model->fieldIndex("ip"), true);
    uiFarmTable->setColumnHidden(model->fieldIndex("port"), true);
    uiFarmTable->setColumnHidden(model->fieldIndex("NTRIP"), true);
    uiFarmTable->setColumnHidden(model->fieldIndex("user"), true);
    uiFarmTable->setColumnHidden(model->fieldIndex("password"), true);
    uiFarmTable->setColumnHidden(model->fieldIndex("stream"), true);
    uiFarmTable->setColumnHidden(model->fieldIndex("autoconnect"), true);
    uiFarmTable->setSelectionMode(QAbstractItemView::SingleSelection);
    uiFarmTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    uiFarmTable->setCurrentIndex(model->index(0, 0));
    uiFarmTable->resizeColumnsToContents();
    uiFarmTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);

    QDataWidgetMapper *mapperFarm = new QDataWidgetMapper(this);
    mapperFarm->setModel(model);
    mapperFarm->addMapping(this->findChild<QLineEdit*>("locationnameEdit"), model->fieldIndex("name"));
    connect(uiFarmTable->selectionModel(),&QItemSelectionModel::currentRowChanged,mapperFarm,&QDataWidgetMapper::setCurrentModelIndex);
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
    model->setHeaderData(model->fieldIndex("storedinfile"),  Qt::Horizontal, tr("File"));
    model->setHeaderData(model->fieldIndex("location"),  Qt::Horizontal, tr("Location"));
    model->setHeaderData(model->fieldIndex("id"),  Qt::Horizontal, tr("Id"));

    // Set the model and hide the ID column:
    uiFieldTable->setModel(model);
    uiFieldTable->setColumnHidden(model->fieldIndex("id"), true);
    uiFieldTable->setColumnHidden(model->fieldIndex("location"), true);

    uiFieldTable->setSelectionMode(QAbstractItemView::SingleSelection);
    uiFieldTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    uiFieldTable->setItemDelegateForColumn(1, checkboxdelegate);
    uiFieldTable->resizeColumnsToContents();
    uiFieldTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
//    uiFieldTable->horizontalHeader()->setVisible(false); // Hide vertical headers

    QDataWidgetMapper *mapperField = new QDataWidgetMapper(this);
    mapperField->setModel(model);
    mapperField->addMapping(this->findChild<QLineEdit*>("fieldnameEdit"), model->fieldIndex("name"));
    mapperField->addMapping(this->findChild<QLineEdit*>("filenameEdit"), model->fieldIndex("storedinfile"));
    connect(uiFieldTable->selectionModel(),&QItemSelectionModel::currentRowChanged,mapperField,&QDataWidgetMapper::setCurrentModelIndex);

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
        db.showError(model->lastError());
        return model;
    }

    // Set the model and hide the ID column:
    uiPathTable->setModel(model);
    uiPathTable->setColumnHidden(model->fieldIndex("iPath"), true);
    uiPathTable->setColumnHidden(model->fieldIndex("storedinfile"), true);
    uiPathTable->setColumnHidden(model->fieldIndex("field"), true);
    uiPathTable->setColumnHidden(model->fieldIndex("fields_name_2"), true);
    uiPathTable->setSelectionMode(QAbstractItemView::SingleSelection);
    uiPathTable->setCurrentIndex(model->index(0, 0));
    uiPathTable->resizeColumnsToContents();
    uiPathTable->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    uiPathTable->horizontalHeader()->setVisible(false); // Hide vertical headers
    return model;
}

void MainWindow::onSelectedFarm(const QModelIndex& current, const QModelIndex& previous)
{
    int row = current.row();
    int id = modelFarm->data(modelFarm->index(row, 0)).toInt();

    QString lon = modelFarm->data(modelFarm->index(row, 2)).toString();
    QString lat = modelFarm->data(modelFarm->index(row, 3)).toString();
    QString usr = modelFarm->data(modelFarm->index(row, 7)).toString();
    QString pwd = modelFarm->data(modelFarm->index(row, 8)).toString();

    mPacketInterface->sendSetUserCmd(ui->mapCarBox->value(),usr);
    mPacketInterface->sendSetPwdCmd(ui->mapCarBox->value(),pwd);

//    qDebug() << "Vehicle: " << ui->mapCarBox->value();

    double llh[3];
    llh[0]=lat.toDouble();
    llh[1]=lon.toDouble();
    llh[2]=0;
    ui->mapLiveWidget->setEnuRef(llh[0],llh[1],0);
    ui->mapWidgetFields->setEnuRef(llh[0],llh[1],0);
    ui->mapWidgetAnalysis->setEnuRef(llh[0],llh[1],0);

    qDebug() << "lat: " << llh[0];
    qDebug() << "lon: " << llh[1];

    mPacketInterface->setEnuRef(ui->mapCarBox->value(), llh);
//    setEnuRef(quint8 id, double *llh, int retries)

    ui->mapWidgetFields->clearAllFields();
    ui->mapWidgetFields->clearAllPaths();
    qDebug() << "Fields (onSelectedFarm 1): " << ui->mapWidgetFields->mFields->size();
    // Get the selected value from the first table view
    QVariant selectedValue = id;

    // Construct a new query based on the selected value
    QString filter = QString("location = %1").arg(id);


    // Set the new query for the QSqlRelationalTableModel
    modelField->setFilter(filter);
    modelField->select();
    /*
    //To make sure the path table view is empty until a field has been selected
    QString filter2 = QString("field = %1").arg(0);
    modelPath->setFilter(filter2);
    modelPath->select();
    */

    // Execute the SQL query
    QString querystring= QString("SELECT * FROM fields WHERE location = %1").arg(selectedValue.toString());
    QSqlQuery query(querystring);

    // Loop through the query results
    while (query.next()) {
        // Access data for each record
        QString xmlFile= query.value("storedinfile").toString();

        QFile file(xmlFile);
        if (!file.exists()) {
            qDebug() << "File does not exist:" << xmlFile;
            return;
        }

        if (!file.open(QIODevice::ReadOnly)) {
            qDebug() << "Could not open file for reading:" << xmlFile;
            return;
        }

        qDebug() << "name: " << xmlFile;
        qDebug() << "Fields (onSelectedFarm 2): " << ui->mapWidgetFields->mFields->size();
        QXmlStreamReader xmlData(&file);
        ui->mapWidgetFields->loadXMLRoute(&xmlData,true);
        qDebug() << "Fields (onSelectedFarm 3): " << ui->mapWidgetFields->mFields->size();
    }
    qDebug() << "AC";
    //            if (ui->fieldTable->model()->rowCount()>0)
    if (ui->mapWidgetFields->getFieldNum()>0)
    {
        qDebug() << "Skala:";
        std::array<double, 4> extremes_m=ui->mapWidgetFields->findExtremeValuesFieldBorders();
        double fieldareawidth_m=extremes_m[2]-extremes_m[0];
        double fieldareaheight_m=extremes_m[3]-extremes_m[1];
        double offsetx_m=(extremes_m[2]+extremes_m[0])/2;
        double offsety_m=(extremes_m[3]+extremes_m[1])/2;
        double scalex=0.5/(fieldareawidth_m);
        double scaley=0.5/(fieldareaheight_m);
        qDebug() << "AD";
        ui->mapWidgetFields->moveView(offsetx_m, offsety_m);
        ui->mapWidgetFields->setScaleFactor(std::min(scalex,scaley));
        qDebug() << std::min(scalex,scaley);
   //     //                ui->fieldTable->selectRow(0);
    } else
    {
        ui->mapWidgetFields->moveView(0, 0);
  //      ui->mapWidget->moveView(0, 0);
        // If no fields set a zoom matching a with of about 500 m -> scalefactor=0.5/500=0.001
        ui->mapWidgetFields->setScaleFactor(0.001);
    }
}

void MainWindow::onSelectedField(const QModelIndex& current, const QModelIndex& previous)
{
    onSelectedFieldGeneral(modelField,modelPath,current, previous);
};

void MainWindow::onSelectedFieldGeneral(QSqlRelationalTableModel *model,QSqlRelationalTableModel *modelPth,const QModelIndex& current, const QModelIndex& previous)
{
    MapWidget* activeMap=ui->mapWidgetFields;
    int row = current.row();

    // Retrieve the data of the selected row if needed
    int id = model->data(model->index(row, 0)).toInt();

    ui->mapWidgetFields->setFieldNow(row);

    QLabel *areaLabel=ui->label_area_ha;
    MapRoute border=activeMap->getField();
    double area=border.getArea();
    areaLabel->setText(QString::number(area));
    ui->mapLiveWidget->clearAllPaths(); // Drive widget
    activeMap->clearAllPaths();

    // Construct a new query based on the selected value
    QString filter = QString("field = %1").arg(id);

    // Set the new query for the QSqlRelationalTableModel
    modelPth->setFilter(filter);
    modelPth->select();

    // Clear the existing items in the combobox
    QSpinBox *selectedRoute=ui->mapRouteBox;
    selectedRoute->clear();

    // Iterate through the rows in the model and add items to the combobox
    for (int row = 0; row < modelPth->rowCount(); ++row) {
        // Assuming "id" is in column 0 and "name" is in column 1
        QVariant id = modelPth->data(modelPth->index(row, 0));
        QVariant name = modelPth->data(modelPth->index(row, 1));

        // Add the item to the combobox
//        selectedRoute->addItem(name.toString(), id);
    }
    qDebug() << "F";

    // Execute the SQL query
    QString querystring= QString("SELECT * FROM paths WHERE field = %1").arg(QString::number(id));
    QSqlQuery query(querystring);
    //     QMessageBox msg;
    while (query.next()) {
        // Access data for each record
        QString xmlFile= query.value("storedinfile").toString();

        QFile file(xmlFile);
        if (!file.exists()) {
            qDebug() << "File does not exist:" << xmlFile;
            return false;
        }

        if (!file.open(QIODevice::ReadOnly)) {
            qDebug() << "Could not open file for reading:" << xmlFile;
            return false;
        }

        QXmlStreamReader xmlData(xmlFile);
        ui->mapLiveWidget->loadXMLRoute(&xmlData,false); // Drive-widget
    }
    qDebug() << "G";
    activeMap->setBorderFocus(true);
    qDebug() << "H";

    //       mapFields->setRouteNow();   // Make sure that no route is set automatically (in order to make it easier to edit)
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

void MainWindow::removeCars()
{
    int iCars=mCars.size();
    for (int i=0;i<iCars;i++)
    {
        int iCarId=mCars.at(i)->getId();
        ui->mapLiveWidget->removeCar(iCarId);
    }
    mCars.clear();
    ui->mapLiveWidget->update();
}

bool MainWindow::connectJoystick()
{
    bool connectJs = false;
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
    if (SDL_Init(SDL_INIT_GAMECONTROLLER) != 0) {
        qDebug() << "SDL_Init Error:" << SDL_GetError();
        return false;
    }

    if (SDL_NumJoysticks() < 1) {
        qDebug() << "No joysticks connected!";
    } else {
        SDL_GameController* controller = SDL_GameControllerOpen(0);
        if (controller) {
            qDebug() << "Game controller connected:" << SDL_GameControllerName(controller);
            mController = controller;

            // Set up a timer to poll for gamepad events
            QTimer* timer = new QTimer(this);
            connect(timer, &QTimer::timeout, this, &MainWindow::pollGamepad);
            timer->start(16); // Poll every 16ms

            connectJs=true;
        } else {
            qDebug() << "Could not open game controller 0:" << SDL_GetError();
        }
    }
#else
    auto gamepads = QGamepadManager::instance()->connectedGamepads();
    if (gamepads.isEmpty()) {
        qDebug() << "Did not find any connected gamepads";
        //        return;
    }

    mJoystick = new QGamepad(*gamepads.begin(), this);

    connect(mJoystick, SIGNAL(buttonChanged(int,bool)),
            this, SLOT(jsButtonChanged(int,bool)));

    connect(mJoystick, &QGamepad::axisLeftXChanged, this, [](double value){
        //           qDebug() << "Left X" << value;
    });
    connect(mJoystick, &QGamepad::axisLeftYChanged, this, [](double value){
        //           qDebug() << "Left Y" << value;
    });
    connect(mJoystick, &QGamepad::axisRightXChanged, this, [](double value){
        //           qDebug() << "Right X" << value;
    });
    connect(mJoystick, &QGamepad::axisRightYChanged, this, [](double value){
        //           qDebug() << "Right Y" << value;
    });

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

    {
        QGamepad js;
        if (js.isConnected()) {
            connectJs = true;
        }
    }

#endif
    return connectJs;
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
        if (JSconnected()) {
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
                // Read axis values using SDL_GameController
                mThrottle = -SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_LEFTY)*1.0/32768.0;
                mSteering = SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_RIGHTX)*1.0/32768.0;
                //#ifdef DEBUG_FUNCTIONS
                if (mThrottle !=0 || mSteering !=0)
                {
//sys                    qDebug() << QDateTime::currentDateTime().toString() << " - JOYSTICK (timerslot), mThrottle: " << mThrottle << ", mSteering: " << mSteering;
                }
                //  #endif

                js_mr_thr = -SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_LEFTY);
               // js_mr_roll = SDL_GameControllerGetAxis(mController, SDL_CONTROLLER_AXIS_RIGHTX);

#else
                mThrottle = -mJoystick->axisLeftY();
                mSteering = mJoystick->axisRightX();

                js_mr_thr = -mJoystick->axisLeftY();
                js_mr_roll = mJoystick->axisRightX();
#endif
                js_mr_pitch = 0; // mJoystick->getAxis(4) / 32768.0; // GL - not sure which controller this corresponds to
                js_mr_yaw = 0; //mJoystick->getAxis(0) / 32768.0;   // GL - not sure which controller this corresponds to


                deadband(mThrottle,0.1, 1.0);
                // qDebug () << "Throttle: " << mThrottle << ", Steering: " << mSteering;

                utility::truncate_number(&js_mr_thr, 0.0, 1.0);
                utility::truncate_number_abs(&js_mr_roll, 1.0);
                utility::truncate_number_abs(&js_mr_pitch, 1.0);
                utility::truncate_number_abs(&js_mr_yaw, 1.0);
            //mSteering /= 2.0;
        } else {
            if (mKeyUp) {
                stepTowards(mThrottle, 1.0, ui->throttleGainBox->value());
            } else if (mKeyDown) {
                stepTowards(mThrottle, -1.0, ui->throttleGainBox->value());
            } else {
                stepTowards(mThrottle, 0.0, ui->throttleGainBox->value());
            }

            if (mKeyRight) {
                stepTowards(mSteering, 1.0, ui->steeringGainBox->value());
            } else if (mKeyLeft) {
                stepTowards(mSteering, -1.0, ui->steeringGainBox->value());
            } else {
                stepTowards(mSteering, 0.0, ui->steeringGainBox->value());
            }
        }

        ui->mrThrottleBar->setValue(js_mr_thr * 100.0);
        ui->mrRollBar->setValue(js_mr_roll * 100.0);
        ui->mrPitchBar->setValue(js_mr_pitch * 100.0);
        ui->mrYawBar->setValue(js_mr_yaw * 100.0);

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
    if (ui->mapShowTextBox->isChecked()) {
        ui->mapLiveWidget->setDrawRouteText(true);
    } else {
        ui->mapLiveWidget->setDrawRouteText(false);
    }
    ui->mapLiveWidget->setSelectedCar(ui->mapCarBox->value());

    // Joystick connected
#ifdef HAS_JOYSTICK
    static bool jsWasconn = false;
    if (JSconnected() != jsWasconn) {
        jsWasconn = JSconnected();

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

#ifdef HAS_JOYSTICK_CHECK
bool MainWindow::JSconnected()
{
    #if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
        int joystickIndex=0;
        return SDL_JoystickGetAttached(SDL_JoystickOpen(joystickIndex)) == SDL_TRUE;
    #else
    return mJoystick->isConnected();
    #endif
};
#endif
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

    if (!mSupportedFirmwares.contains(qMakePair(static_cast<int>(state.fw_major), static_cast<int>(state.fw_minor)))) {
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
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
    QString str = QString("Vehicle %1 ack: ").arg(id);
#else
    QString str;
    str.sprintf("Vehicle %d ack: ", id);
#endif
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
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
    QString str = QString("ping response time: % ms").arg((double)time / 1000.0,0,'f',3);
#else
    QString str;
    ("ping response time: %.3f ms", (double)time / 1000.0);
#endif
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


#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
/*            info = QString("Fix type: %1\n"
                         "Sats    : %2\n"
                         "Height  : %3\n"
                         "Age     : %4")
                      .arg(fix_t.toLocal8Bit().data())
                      .arg(gga.n_sat,0,'d',0)
                      .arg(gga.height,0,'f',2)
                      .arg(gga.diff_age,0,'f',2);*/
            QByteArray fixBytes = fix_t.toLocal8Bit();
            info = QString("Fix type: %1\n"
                           "Sats    : %2\n"
                           "Height  : %3\n"
                           "Age     : %4")
                       .arg(QString::fromLocal8Bit(fixBytes))
                       .arg(QString::number(gga.n_sat))
                       .arg(gga.height, 0, 'f', 2)
                       .arg(gga.diff_age, 0, 'f', 2);



#else
            info.sprintf("Fix type: %s\n"
                         "Sats    : %d\n"
                         "Height  : %.2f\n"
                         "Age     : %.2f",
                         fix_t.toLocal8Bit().data(),
                         gga.n_sat,
                         gga.height,
                         gga.diff_age);
#endif
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
/*
                    out << str.sprintf("%d\n", seq);          // Seq
                    out << str.sprintf("%05f\n", xyz[0]);     // X
                    out << str.sprintf("%05f\n", xyz[1]);     // Y
                    out << str.sprintf("%05f\n", xyz[2]);     // Height
                    out << str.sprintf("%05f\n", gga.t_tow);  // GPS time of week
                    out << str.sprintf("%d\n", 2);            // Vehicle ID
*/
                    out << QString("%1\n").arg(seq);                  // Seq
                    out << QString("%1\n").arg(xyz[0], 0, 'f', 5);    // X
                    out << QString("%1\n").arg(xyz[1], 0, 'f', 5);    // Y
                    out << QString("%1\n").arg(xyz[2], 0, 'f', 5);    // Height
                    out << QString("%1\n").arg(gga.t_tow, 0, 'f', 5); // GPS time of week
                    out << QString("%1\n").arg(2);                    // Vehicle ID

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
            // 1: Extra out
            // 3: Extra in
        qDebug() << "in logic 1";

            if (button == FRONT_UP || button == FRONT_DOWN || button == 1 ||
                    button == REAR_DOWN || button == REAR_UP || button == 6) {
            qDebug() << "in logic 2";
                for(QList<CarInterface*>::Iterator it_car = mCars.begin();it_car < mCars.end();it_car++) {
                qDebug() << "in logic 3";
                    CarInterface *car = *it_car;
                    if (car->getCtrlKb()) {
                    qDebug() << "in logic 4";
                        if (button == FRONT_UP || button == FRONT_DOWN) {
                        qDebug() << "in logic 5";
                            if (pressed) {
                            qDebug() << "pressed";
                                if (button == FRONT_UP) {
                                    qDebug() << "Hydraulic, front up";
                                    mPacketInterface->hydraulicMove(car->getId(), HYDRAULIC_POS_FRONT, HYDRAULIC_MOVE_UP);
                                } else {
                                    qDebug() << "Hydraulic, front down";
                                    mPacketInterface->hydraulicMove(car->getId(), HYDRAULIC_POS_FRONT, HYDRAULIC_MOVE_DOWN);
                                }
                            } else {
                                mPacketInterface->hydraulicMove(car->getId(), HYDRAULIC_POS_FRONT, HYDRAULIC_MOVE_STOP);
                            }
                        } else if (button == REAR_UP || button == REAR_DOWN) {
                            if (pressed) {
                                if (button == REAR_UP) {
                                    qDebug() << "Hydraulic, rear up";
                                    mPacketInterface->hydraulicMove(car->getId(), HYDRAULIC_POS_REAR, HYDRAULIC_MOVE_UP);
                                } else {
                                    qDebug() << "Hydraulic, rear down";
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
    removeCars();
    ui->carsWidget->clear();

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

    QStringList conns = ui->tcpConnEdit->toPlainText().split("\n");

    for (QString c: conns) {
        QStringList ipPort = c.split(":");

        if (ipPort.size() == 1) {
            mTcpClientMulti->addConnection(ipPort.at(0),
                                           8300);
        } else if (ipPort.size() == 2) {
            mTcpClientMulti->addConnection(ipPort.at(0),
                                           ipPort.at(1).toInt());
        }
        addCar(mCars.size(),ipPort.at(0));
    }
}

void MainWindow::on_tcpPingButton_clicked()
{
    QStringList conns = ui->tcpConnEdit->toPlainText().split("\n");

    for (QString c: conns) {
        QStringList ipPort = c.split(":");

        if (ipPort.size() == 2) {
            mPing->pingHost(ipPort.at(0), 64, "TCP Host");
            break;
        }
    }
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

    try {
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


    } catch (...) {
        qDebug() << "Could not get route!";
    }

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

void MainWindow::onGenerateLineButtonClicked()
{
    int p1=ui->point1LineEdit->text().toInt();
    int p2=ui->point2LineEdit->text().toInt();

    qDebug() << "p1: " << p1;
    qDebug() << "p2: " << p2;

    int iField=ui->mapWidgetFields->getFieldNow();
    qDebug() << "field: " << iField;
    MapRoute CurField=ui->mapWidgetFields->getField(iField);

    qDebug() << "Size: " << CurField.size();
    LocPoint first=CurField.at(p1);
    LocPoint second=CurField.at(p2);

    qDebug() << "first, x: " << first.getX() << ", y: " << first.getY();
    qDebug() << "second, x: " << second.getX() << ", y: " << second.getY();

    MapRoute *currentRoute;
    bool bHasCurrentRoute=false;

    if (ui->mapLiveWidget->mPaths->size()>0)
    {
        qDebug() << "Existing route";
        currentRoute=&(ui->mapLiveWidget->mPaths->getCurrent());
        bHasCurrentRoute=true;
    } else
    {
        qDebug() << "New route";
        currentRoute=new MapRoute();
    }
    qDebug() << "Before append";
    currentRoute->append(first);
    qDebug() << "After append";
    emit routePointAdded(first);
    currentRoute->prepend(second);
    qDebug() << "After prepend";
    emit routePointAdded(second);
    qDebug() << "ok";
    if (!bHasCurrentRoute)
    {
        ui->mapLiveWidget->mPaths->append(*currentRoute);
        ui->mapLiveWidget->mPaths->mRouteNow=ui->mapLiveWidget->mPaths->size()-1;
    }
    qDebug() << "added path";


//    ui->mapLiveWidget->addRoutePoint(first.getX(), first.getY());
//    ui->mapLiveWidget->addRoutePoint(second.getX(), second.getY());
}
void MainWindow::onGeneratePathButtonClicked()
{
    double fieldLength_m = ui->fieldLengthMLineEdit->text().toDouble(); // Assuming input1 is the object name of a QLineEdit
    double fieldWidth_m = ui->fieldWidthMLineEdit->text().toDouble(); // Assuming input1 is the object name of a QLineEdit

    double implementLength_m = ui->implementLengthLineEdit->text().toDouble(); // Assuming input1 is the object name of a QLineEdit
    double implementWidth_m = ui->implementWidthLineEdit->text().toDouble(); // Assuming input1 is the object name of a QLineEdit

    int plots_DrivingDirection = ui->plotsInDrivingDirectionLineEdit->text().toInt();
    int plots_NonDrivingDirection = ui->plotsInNonDrivingDirectionLineEdit->text().toInt();

    double distancebetweenplots_drivingdirection_m = ui->distanceBetweenPlotsMDdLineEdit->text().toDouble();
    double distancebetweenplots_nondrivingdirection_m = ui->distanceBetweenPlotsMNddLineEdit->text().toDouble();


    if (ui->mapLiveWidget->getPathNum()>0)
    {
        MapRoute activeRoute=ui->mapLiveWidget->getCurrentPath();
        LocPoint p1,p2;
        if (ui->inveseDirectionCheckBox->isChecked())
        {
            p1=activeRoute[1];
            p2=activeRoute[0];
        } else
        {
            p1=activeRoute[0];
            p2=activeRoute[1];
        }
        RouteGenerator rg(fieldLength_m,fieldWidth_m,implementLength_m,implementWidth_m,plots_DrivingDirection,plots_NonDrivingDirection,distancebetweenplots_drivingdirection_m,distancebetweenplots_nondrivingdirection_m,p1,p2);
        rg.generateXmlFile();

        QString filePath="output.xml";
        QFile file(filePath);
        if (!file.exists()) {
            qDebug() << "File does not exist:" << filePath;
            return;
        }

        if (!file.open(QIODevice::ReadOnly)) {
            qDebug() << "Could not open file for reading:" << filePath;
            return;
        }

        QXmlStreamReader xmlData(&file);
        ui->mapLiveWidget->loadXMLRoute(&xmlData,false);
        int newPath=ui->mapLiveWidget->getPathNow()+1;
        ui->mapRouteBox->setValue(newPath);
    //    ui->mapLiveWidget->setPathNow(newPath);

    } else
    {
        QMessageBox msgBox;
        msgBox.setText("You need to add a main line");
        msgBox.exec();
    }
}

bool MainWindow::onLoadShapefile()
{
    // Create a file dialog
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    "Open File", "", "Shape Files [*.shp](*.shp)");

    // Check if a file was selected
    if (!fileName.isEmpty()) {
        qDebug() << "Selected file:" << fileName;
        // Use the selected file path as needed
    }

    QByteArray utf8Path = fileName.toUtf8();     // store the QByteArray
    const char* shapefile = utf8Path.constData(); // safe pointer

    qDebug() << "loading shapefile";

    // 1️⃣ Register all OGR drivers
    GDALAllRegister();

    // 2️⃣ Open the shapefile (read-only)
    GDALDataset *poDS = (GDALDataset*) GDALOpenEx(shapefile, GDAL_OF_VECTOR, NULL, NULL, NULL);
    if (poDS == nullptr) {
        std::cerr << "Failed to open shapefile: " << shapefile << std::endl;
        return false;
    }

    // 3️⃣ Get the first layer
    OGRLayer* poLayer = poDS->GetLayer(0);
    if (poLayer == nullptr) {
        std::cerr << "Could not get layer from shapefile" << std::endl;
        GDALClose(poDS);
        return false;
    }

    double illh[3];
    ui->mapWidgetFields->getEnuRef(illh);

    // 4️⃣ Loop through features
    OGRFeature* poFeature;
    poLayer->ResetReading();
    while ((poFeature = poLayer->GetNextFeature()) != nullptr) {
        OGRGeometry* geom = poFeature->GetGeometryRef();
        if (geom != nullptr && wkbFlatten(geom->getGeometryType()) == wkbMultiPolygon) {
            OGRMultiPolygon* multiPoly = geom->toMultiPolygon();

            for (int i = 0; i < multiPoly->getNumGeometries(); i++) {
                OGRPolygon* poly = multiPoly->getGeometryRef(i)->toPolygon();

                // Outer ring (boundary)
                OGRLinearRing* outer = poly->getExteriorRing();

                MapRoute MR;
                if (outer) {
                    int nPoints = outer->getNumPoints();
                    for (int j = 0; j < nPoints; j++) {
                        double llh[3];
                        double xyh[3];
                        llh[0]=outer->getY(j);
                        llh[1]=outer->getX(j);
                        llh[2]=0;
                        utility::llhToEnu(illh, llh, xyh);
                        LocPoint lp(xyh[0],xyh[1]);
                        MR.append(lp);

                    }
                }
                ui->mapWidgetFields->addField(MR);
                /*
                svg << "' style='fill:none;stroke:black;stroke-width:1'/>\n";

                // Inner rings (holes)
                for (int h = 0; h < poly->getNumInteriorRings(); h++) {
                    OGRLinearRing* hole = poly->getInteriorRing(h);
                    qDebug() << "  Hole " << h << ":" ;
                    for (int j = 0; j < hole->getNumPoints(); j++) {
                        qDebug() << hole->getX(j) << ", " << hole->getY(j);
                    }
                }
                */
            }
        }

        OGRFeature::DestroyFeature(poFeature);
    }

    // 5️⃣ Close dataset
    GDALClose(poDS);

    // Get the current date and time
//    QDateTime currentDateTime = QDateTime::currentDateTime();

    // Format the date and time as a string
//    QString formattedDateTime = currentDateTime.toString("yyyyMMdd-hhmmss");

//    QString storedfilename= "shapefileimport" + formattedDateTime;
//    qDebug() << "file name" << storedfilename;

    if (ui->mapWidgetFields->saveRoutes(ui->filenameEdit->text()))
    {
        showStatusInfo("Saved routes", true);
    } else
    {
        showStatusInfo("Could not save routes", false);
    };
    qDebug() << "stored file";

    addField();

//    svg << "</svg>\n";
    return true;
};


bool MainWindow::onLoadLogfile()
{
    // Create a file dialog
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    "Open File", "", "Shape Files [*.csv](*.csv)");

    // Check if a file was selected
    if (!fileName.isEmpty()) {
        qDebug() << "Selected file:" << fileName;
        // Use the selected file path as needed
    }

    QByteArray utf8Path = fileName.toUtf8();     // store the QByteArray
    const string &nmeafilename = utf8Path.constData(); // safe pointer

    double refs[3]={0.0,0.0,0.0};
    ui->mapWidgetAnalysis->getEnuRef(refs);
    QByteArray xmlData;
    if (NmeaServer::toXML(refs[0],refs[1],nmeafilename, &xmlData)) {
        QXmlStreamReader xmlReader(xmlData);
        ui->mapWidgetAnalysis->loadXMLRoute(&xmlReader, false);
        // Add to listview
        fileList.append(QString::fromStdString(nmeafilename));
        fileModel->setStringList(fileList);  // Update the model
    } else
    {
        QMessageBox msgBox;
        msgBox.setText("The file does not store nmea data in the right format!");
        msgBox.exec();
    }

    return true;
};


void MainWindow::on_listLogFilesView_clicked(const QModelIndex& index) {
    if (index.isValid()) {
        QString filename = fileModel->data(index, Qt::DisplayRole).toString();
        qDebug() << "Selected file:" << filename;
        int row = index.row();  // 0-based row index
        qDebug() << "Clicked row:" << row;
         ui->mapWidgetAnalysis->setPathNow(row);
    }
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
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
                    info = QString("Fix type: %s\n"
                                   "Sats    : %d\n"
                                   "Height  : %.2f")
                               .arg(fix_t.toLocal8Bit().data())
                               .arg(gga.n_sat)
                               .arg(gga.height);
#else
                    info.sprintf("Fix type: %s\n"
                                 "Sats    : %d\n"
                                 "Height  : %.2f",
                                 fix_t.toLocal8Bit().data(),
                                 gga.n_sat,
                                 gga.height);
#endif
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
                ui->mapLiveWidget->osmClient()->setTileServerUrl("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile");
    	//        ui->mapWidget->osmClient()->setTileServerUrl("http://tile.openstreetmap.org");
    }
}

void MainWindow::on_mapOsmServerHiResButton_toggled(bool checked)
{
    if (checked) {
                ui->mapLiveWidget->osmClient()->setTileServerUrl("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile");
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
    llh[0]=llh[0]-346;
    llh[1]=llh[1]-75;
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
#if (QT_VERSION < QT_VERSION_CHECK(6, 0, 0))
    stream.setCodec("UTF-8");
#endif
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

/*
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
*/
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

    if (ui->mapLiveWidget->saveRoutes(filename))
    {
        showStatusInfo("Saved routes", true);
    } else
    {
        showStatusInfo("Could not save routes", false);
    };
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
    ui->mapLiveWidget->zoomInOnRoute(ui->mapRouteBox->value(), 0.1);
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

void MainWindow::on_setBoundsRoutePushButton_clicked()
{
    int r = ui->mapLiveWidget->getPathNow();
    ui->boundsRouteSpinBox->setValue(r);
}

void MainWindow::on_boundsFillPushButton_clicked()
{

    MapRoute bounds = ui->mapLiveWidget->getField(ui->mapLiveWidget->getFieldNow());
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

    ui->mapLiveWidget->addPath(route);
    int r = ui->mapLiveWidget->getPaths().size()-1;
    ui->mapLiveWidget->setPathNow(r);
    // ui->mapRouteBox->setCurrentIndex(r);
    ui->mapRouteBox->setValue(r);
    ui->mapLiveWidget->repaint();
}
/*
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

*/
void MainWindow::on_lowerToolsCheckBox_stateChanged(int arg1)
{
    ui->lowerToolsDistanceSpinBox->setEnabled(arg1 != 0);
}

void MainWindow::on_raiseToolsCheckBox_stateChanged(int arg1)
{
    ui->raiseToolsDistanceSpinBox->setEnabled(arg1 != 0);
}

void MainWindow::on_AutopilotConfigurePushButton_clicked()
{
    ui->mainTabWidget->setCurrentIndex(ui->mainTabWidget->indexOf(ui->tab));
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

#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))

void MainWindow::pollGamepad() {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_CONTROLLERBUTTONDOWN || event.type == SDL_CONTROLLERBUTTONUP) {
            qDebug() << "up or down";
            handleButtonEvent(event.cbutton);
        } else if (event.type == SDL_CONTROLLERAXISMOTION) {
//            qDebug() << "axis";
            handleAxisEvent(event.caxis);
        }
    }
}

void MainWindow::handleButtonEvent(const SDL_ControllerButtonEvent& event) {
    bool pressed = (event.state == SDL_PRESSED);
    switch (event.button) {
    case SDL_CONTROLLER_BUTTON_LEFTSHOULDER:
        qDebug() << "Button L1" << pressed;
        jsButtonChanged(4, pressed);
        break;
    case SDL_CONTROLLER_BUTTON_RIGHTSHOULDER:
        qDebug() << "Button R1" << pressed;
        jsButtonChanged(5, pressed);
        break;
    }
}

void MainWindow::handleAxisEvent(const SDL_ControllerAxisEvent& event) {
    switch (event.axis) {
    case SDL_CONTROLLER_AXIS_LEFTX:
 //       qDebug() << "Left X" << event.value;
        break;
    case SDL_CONTROLLER_AXIS_LEFTY:
 //       qDebug() << "Left Y" << event.value;
        break;
    case SDL_CONTROLLER_AXIS_RIGHTX:
 //       qDebug() << "Right X" << event.value;
        break;
    case SDL_CONTROLLER_AXIS_RIGHTY:
//        qDebug() << "Right Y" << event.value;
        break;
    case SDL_CONTROLLER_AXIS_TRIGGERLEFT:
//        qDebug() << "Button L2:" << event.value;
        jsButtonChanged(6, event.value > 0);
        break;
    case SDL_CONTROLLER_AXIS_TRIGGERRIGHT:
//        qDebug() << "Button R2:" << event.value;
        jsButtonChanged(7, event.value > 0);
        break;
    }
}

#endif

void MainWindow::handleAddFieldButton()
{
    addField();
    //ui->fieldTable->update();
}

void MainWindow::handleAddFarmButton()
{
    //    QMessageBox msgBox;
    //    msgBox.setText("Adding farm:" + ui->locationnameEdit->text());
    //    msgBox.exec();
    db.addFarm(ui->locationnameEdit->text());
    QSqlTableModel *tableModel = qobject_cast<QSqlTableModel*>(ui->farmTable->model());
    if (tableModel) {
        tableModel->select(); // Re-fetch the data from the database to update the model
    }
}

void MainWindow::addField()
{
    qDebug() << "in AddField";
    int farmid=currentFarm();
    if (farmid)
    {
        db.addField(ui->fieldnameEdit->text(),farmid,ui->filenameEdit->text());
    } else {
        QMessageBox msgBox;
        msgBox.setText("No row selected!");
        msgBox.exec();
    };
    modelField->refresh();
    modelField->select();
}

int MainWindow::currentFarm()
{
    QModelIndexList selectedIndexes = ui->farmTable->selectionModel()->selectedIndexes();
    if (!selectedIndexes.isEmpty()) {
        int rowIndex = selectedIndexes.first().row();
        return modelFarm->record(rowIndex).value("id").toInt(); // farm id
    } else {
        return -1;
    };
}

void MainWindow::setCurrentFarm(int farm)
{
    selectRowByPrimaryKey(ui->farmTable, modelFarm, "id", farm);
}

QLabel* MainWindow::getLogLabel()
{
    return ui->textAnalysis;
}


void selectRowByPrimaryKey(QTableView* tableView, QSqlRelationalTableModel* model, const QString& primaryKeyColumnName, const QVariant& primaryKeyValue) {
    qDebug() << "primaryKeyValue: " << primaryKeyValue;
    // Find the column index for the primary key
    int primaryKeyColumn = model->fieldIndex(primaryKeyColumnName);

    // Iterate through the rows to find the target primary key
    for (int row = 0; row < model->rowCount(); ++row) {
        QModelIndex index = model->index(row, primaryKeyColumn);
        QVariant currentPrimaryKeyValue = model->data(index);
        qDebug() << "currentPrimaryKeyValue: " << currentPrimaryKeyValue;

        if (currentPrimaryKeyValue == primaryKeyValue) {
            // Select the row in the table view
            QModelIndex rowIndex = model->index(row, 0);
            tableView->setCurrentIndex(rowIndex);
            tableView->scrollTo(rowIndex);
            break;
        }
    }
}

MainWindow* findMainWindow() {
    // Iterate through all top-level widgets
    for (QWidget *widget : qApp->topLevelWidgets()) {
        // Try to cast the widget to MainWindow
        MainWindow *mainWindow = qobject_cast<MainWindow*>(widget);
        if (mainWindow) {
            return mainWindow; // Return the first MainWindow found
        }
    }
    return nullptr; // Return nullptr if no MainWindow is found
}
