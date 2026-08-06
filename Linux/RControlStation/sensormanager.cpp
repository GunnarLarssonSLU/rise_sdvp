#include "sensormanager.h"
#include "colourutils.h"
#include <QHeaderView>
#include <QMessageBox>
#include <QInputDialog>
#include <QDebug>
#include <QLabel>

SensorManager::SensorManager(QWidget *parent) : QWidget(parent)
{
    // Create UI elements
    sensorsTableView = new QTableView(this);
    addButton = new QPushButton("Add Sensor", this);
    removeButton = new QPushButton("Remove Sensor", this);
    editButton = new QPushButton("Edit Sensor", this);
    colourButton = new QPushButton("Choose Colour", this);
    
    // Setup layouts
    mainLayout = new QVBoxLayout(this);
    buttonLayout = new QHBoxLayout();
    
    buttonLayout->addWidget(addButton);
    buttonLayout->addWidget(removeButton);
    buttonLayout->addWidget(editButton);
    buttonLayout->addWidget(colourButton);
    buttonLayout->addStretch();
    
    mainLayout->addWidget(sensorsTableView);
    mainLayout->addLayout(buttonLayout);
    
    // Set layout properties
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(6);
    
    // Connect signals
    connect(addButton, &QPushButton::clicked, this, &SensorManager::addSensor);
    connect(removeButton, &QPushButton::clicked, this, &SensorManager::removeSensor);
    connect(editButton, &QPushButton::clicked, this, &SensorManager::editSensor);
    connect(colourButton, &QPushButton::clicked, this, &SensorManager::editSensorColour);
    
    // Set selection mode
    sensorsTableView->setSelectionMode(QAbstractItemView::SingleSelection);
    sensorsTableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    
    // Initial button states
    removeButton->setEnabled(false);
    editButton->setEnabled(false);
    colourButton->setEnabled(false);
    
    // Setup sensor edit dialog
    setupSensorEditDialog();
}

void SensorManager::setupUi(QSqlDatabase db)
{
    // Setup the table model
    sensorsModel = new QSqlTableModel(this, db);
    setupTableModel();
    
    sensorsTableView->setModel(sensorsModel);
    
    // Configure table view
    sensorsTableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    sensorsTableView->setSelectionMode(QAbstractItemView::SingleSelection);
    sensorsTableView->setEditTriggers(QAbstractItemView::NoEditTriggers); // Use edit button instead
    sensorsTableView->horizontalHeader()->setStretchLastSection(true);
    sensorsTableView->verticalHeader()->setVisible(false);
    
    // Set column headers
    sensorsModel->setHeaderData(1, Qt::Horizontal, "Sensor Name");
    sensorsModel->setHeaderData(2, Qt::Horizontal, "Type");
    sensorsModel->setHeaderData(3, Qt::Horizontal, "Current Value");
    sensorsModel->setHeaderData(4, Qt::Horizontal, "Min");
    sensorsModel->setHeaderData(5, Qt::Horizontal, "Max");
    sensorsModel->setHeaderData(6, Qt::Horizontal, "Units");
    sensorsModel->setHeaderData(7, Qt::Horizontal, "Colour");
    
    // Hide ID, offset, and scale columns (they're for internal use)
    sensorsTableView->setColumnHidden(0, true); // ID
    sensorsTableView->setColumnHidden(8, true); // calibration_offset
    sensorsTableView->setColumnHidden(9, true); // calibration_scale
    
    // Set custom delegate for colour column
    ColourDelegate *colourDelegate = new ColourDelegate(this);
    sensorsTableView->setItemDelegateForColumn(7, colourDelegate);
    
    // Connect selection model
    QItemSelectionModel *selectionModel = sensorsTableView->selectionModel();
    connect(selectionModel, &QItemSelectionModel::selectionChanged, 
            this, &SensorManager::onSensorSelectionChanged);
    
    // Refresh data
    sensorsModel->select();
    
    // Set initial column widths
    sensorsTableView->setColumnWidth(1, 200); // Name column
    sensorsTableView->setColumnWidth(2, 120); // Type column
    sensorsTableView->setColumnWidth(3, 100); // Current value column
    sensorsTableView->setColumnWidth(4, 80);  // Min column
    sensorsTableView->setColumnWidth(5, 80);  // Max column
    sensorsTableView->setColumnWidth(6, 80);  // Units column
    sensorsTableView->setColumnWidth(7, 100); // Colour column
}

void SensorManager::setupTableModel()
{
    sensorsModel->setTable("sensors");
    sensorsModel->setEditStrategy(QSqlTableModel::OnManualSubmit);
    
    // Select all columns
    sensorsModel->select();
}

void SensorManager::setupSensorEditDialog()
{
    sensorEditDialog = new QDialog(this);
    sensorEditDialog->setWindowTitle("Edit Sensor");
    sensorEditDialog->setModal(true);
    
    QVBoxLayout *dialogLayout = new QVBoxLayout(sensorEditDialog);
    
    // Main form
    QFormLayout *formLayout = new QFormLayout();
    
    // Type selection
    typeComboBox = new QComboBox();
    typeComboBox->addItem("Position");
    typeComboBox->addItem("Pressure");
    typeComboBox->addItem("Temperature");
    typeComboBox->addItem("Voltage");
    typeComboBox->addItem("Current");
    typeComboBox->addItem("Angle");
    typeComboBox->addItem("Distance");
    typeComboBox->addItem("Speed");
    typeComboBox->addItem("Force");
    formLayout->addRow("Sensor Type:", typeComboBox);
    
    // Current value (read-only display)
    currentValueSpinBox = new QDoubleSpinBox();
    currentValueSpinBox->setRange(-1000.0, 1000.0);
    currentValueSpinBox->setDecimals(2);
    currentValueSpinBox->setReadOnly(true);
    formLayout->addRow("Current Value:", currentValueSpinBox);
    
    // Range controls
    QHBoxLayout *rangeLayout = new QHBoxLayout();
    minValueSpinBox = new QDoubleSpinBox();
    minValueSpinBox->setRange(-10000.0, 10000.0);
    minValueSpinBox->setValue(0.0);
    minValueSpinBox->setDecimals(2);
    
    maxValueSpinBox = new QDoubleSpinBox();
    maxValueSpinBox->setRange(-10000.0, 10000.0);
    maxValueSpinBox->setValue(100.0);
    maxValueSpinBox->setDecimals(2);
    
    rangeLayout->addWidget(new QLabel("Min:"));
    rangeLayout->addWidget(minValueSpinBox);
    rangeLayout->addWidget(new QLabel("Max:"));
    rangeLayout->addWidget(maxValueSpinBox);
    formLayout->addRow("Value Range:", rangeLayout);
    
    // Units
    unitsComboBox = new QComboBox();
    unitsComboBox->addItem("%");
    unitsComboBox->addItem("mm");
    unitsComboBox->addItem("cm");
    unitsComboBox->addItem("m");
    unitsComboBox->addItem("degrees");
    unitsComboBox->addItem("radians");
    unitsComboBox->addItem("Pa");
    unitsComboBox->addItem("bar");
    unitsComboBox->addItem("psi");
    unitsComboBox->addItem("°C");
    unitsComboBox->addItem("°F");
    unitsComboBox->addItem("V");
    unitsComboBox->addItem("A");
    unitsComboBox->addItem("N");
    unitsComboBox->addItem("unitless");
    formLayout->addRow("Units:", unitsComboBox);
    
    // Calibration controls
    QGroupBox *calibrationGroupBox = new QGroupBox("Calibration");
    QFormLayout *calibrationLayout = new QFormLayout(calibrationGroupBox);
    
    calibrationOffsetSpinBox = new QDoubleSpinBox();
    calibrationOffsetSpinBox->setRange(-1000.0, 1000.0);
    calibrationOffsetSpinBox->setValue(0.0);
    calibrationOffsetSpinBox->setDecimals(3);
    calibrationLayout->addRow("Offset:", calibrationOffsetSpinBox);
    
    calibrationScaleSpinBox = new QDoubleSpinBox();
    calibrationScaleSpinBox->setRange(0.001, 1000.0);
    calibrationScaleSpinBox->setValue(1.0);
    calibrationScaleSpinBox->setDecimals(3);
    calibrationLayout->addRow("Scale:", calibrationScaleSpinBox);
    
    formLayout->addRow(calibrationGroupBox);
    
    // Dialog buttons
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &SensorManager::saveSensorData);
    connect(buttonBox, &QDialogButtonBox::rejected, sensorEditDialog, &QDialog::reject);
    
    dialogLayout->addLayout(formLayout);
    dialogLayout->addWidget(buttonBox);
}

void SensorManager::addSensor()
{
    bool ok;
    QString sensorName = QInputDialog::getText(this, "Add Sensor", 
                                                "Enter sensor name:", 
                                                QLineEdit::Normal, 
                                                "New Sensor", &ok);
    
    if (ok && !sensorName.isEmpty()) {
        // Insert new record
        QSqlRecord record = sensorsModel->record();
        record.setValue("name", sensorName);
        record.setValue("type", "Position"); // Default type
        record.setValue("current_value", 0.0); // Default value
        record.setValue("min_value", 0.0); // Default min
        record.setValue("max_value", 100.0); // Default max
        record.setValue("units", "%"); // Default units
        record.setValue("calibration_offset", 0.0); // Default offset
        record.setValue("calibration_scale", 1.0); // Default scale
        
        // Generate a colour for the new sensor
        int newRow = sensorsModel->rowCount();
        QColor colour = ColourUtils::generateColourForAction(newRow);
        QString colourHex = ColourUtils::qColorToHex(colour);
        record.setValue("colour", colourHex);
        
        if (sensorsModel->insertRecord(-1, record)) {
            if (sensorsModel->submitAll()) {
                sensorsModel->select();
                QMessageBox::information(this, "Success", "Sensor added successfully!");
            } else {
                QMessageBox::warning(this, "Error", "Failed to save sensor: " + sensorsModel->lastError().text());
                sensorsModel->revertAll();
            }
        } else {
            QMessageBox::warning(this, "Error", "Failed to add sensor: " + sensorsModel->lastError().text());
        }
    }
}

void SensorManager::removeSensor()
{
    QModelIndex current = sensorsTableView->selectionModel()->currentIndex();
    QModelIndexList selected = sensorsTableView->selectionModel()->selectedRows();
    
    QModelIndex index;
    if (!current.isValid() && selected.isEmpty()) {
        QMessageBox::warning(this, "No Selection", "Please select a sensor to remove.");
        return;
    }
    
    // Prefer current index, fall back to first selected row
    if (current.isValid()) {
        index = current;
    } else {
        index = selected.first();
    }
    
    int row = index.row();
    QString sensorName = sensorsModel->data(sensorsModel->index(row, 1)).toString();
    
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Remove Sensor", 
                                   QString("Are you sure you want to remove the sensor '%1'?").arg(sensorName),
                                   QMessageBox::Yes|QMessageBox::No);
    
    if (reply == QMessageBox::Yes) {
        // Check if this is the last sensor
        if (sensorsModel->rowCount() <= 1) {
            QMessageBox::warning(this, "Cannot Remove", "You cannot remove the last sensor. At least one sensor must remain.");
            return;
        }
        
        sensorsModel->removeRow(row);
        
        if (sensorsModel->submitAll()) {
            sensorsModel->select();
            QMessageBox::information(this, "Success", "Sensor removed successfully!");
        } else {
            QMessageBox::warning(this, "Error", "Failed to remove sensor: " + sensorsModel->lastError().text());
            sensorsModel->revertAll();
        }
    }
}

void SensorManager::editSensor()
{
    QModelIndex current = sensorsTableView->selectionModel()->currentIndex();
    
    if (!current.isValid()) {
        QMessageBox::warning(this, "No Selection", "Please select a sensor to edit.");
        return;
    }
    
    int sensorId = sensorsModel->data(sensorsModel->index(current.row(), 0)).toInt();
    loadSensorData(sensorId);
    sensorEditDialog->exec();
}

void SensorManager::loadSensorData(int sensorId)
{
    // Find the row with the given sensor ID
    for (int row = 0; row < sensorsModel->rowCount(); row++) {
        int id = sensorsModel->data(sensorsModel->index(row, 0)).toInt();
        if (id == sensorId) {
            // Load data
            QString name = sensorsModel->data(sensorsModel->index(row, 1)).toString();
            QString type = sensorsModel->data(sensorsModel->index(row, 2)).toString();
            double currentValue = sensorsModel->data(sensorsModel->index(row, 3)).toDouble();
            double minValue = sensorsModel->data(sensorsModel->index(row, 4)).toDouble();
            double maxValue = sensorsModel->data(sensorsModel->index(row, 5)).toDouble();
            QString units = sensorsModel->data(sensorsModel->index(row, 6)).toString();
            double offset = sensorsModel->data(sensorsModel->index(row, 8)).toDouble();
            double scale = sensorsModel->data(sensorsModel->index(row, 9)).toDouble();
            
            // Set UI values
            sensorEditDialog->setWindowTitle("Edit Sensor: " + name);
            
            int typeIndex = typeComboBox->findText(type);
            if (typeIndex >= 0) {
                typeComboBox->setCurrentIndex(typeIndex);
            }
            
            currentValueSpinBox->setValue(currentValue);
            minValueSpinBox->setValue(minValue);
            maxValueSpinBox->setValue(maxValue);
            
            int unitsIndex = unitsComboBox->findText(units);
            if (unitsIndex >= 0) {
                unitsComboBox->setCurrentIndex(unitsIndex);
            }
            
            calibrationOffsetSpinBox->setValue(offset);
            calibrationScaleSpinBox->setValue(scale);
            
            break;
        }
    }
}

void SensorManager::saveSensorData()
{
    QModelIndex current = sensorsTableView->selectionModel()->currentIndex();
    
    if (!current.isValid()) {
        QMessageBox::warning(this, "No Selection", "No sensor selected to save.");
        return;
    }
    
    int row = current.row();
    
    // Update model data
    sensorsModel->setData(sensorsModel->index(row, 2), typeComboBox->currentText()); // type
    sensorsModel->setData(sensorsModel->index(row, 4), minValueSpinBox->value()); // min_value
    sensorsModel->setData(sensorsModel->index(row, 5), maxValueSpinBox->value()); // max_value
    sensorsModel->setData(sensorsModel->index(row, 6), unitsComboBox->currentText()); // units
    sensorsModel->setData(sensorsModel->index(row, 8), calibrationOffsetSpinBox->value()); // calibration_offset
    sensorsModel->setData(sensorsModel->index(row, 9), calibrationScaleSpinBox->value()); // calibration_scale
    
    if (sensorsModel->submitAll()) {
        sensorsModel->select();
        QMessageBox::information(this, "Success", "Sensor updated successfully!");
        sensorEditDialog->accept();
    } else {
        QMessageBox::warning(this, "Error", "Failed to update sensor: " + sensorsModel->lastError().text());
        sensorsModel->revertAll();
    }
}

void SensorManager::editSensorColour()
{
    QModelIndex current = sensorsTableView->selectionModel()->currentIndex();
    QModelIndexList selected = sensorsTableView->selectionModel()->selectedRows();
    
    QModelIndex index;
    if (!current.isValid() && selected.isEmpty()) {
        QMessageBox::warning(this, "No Selection", "Please select a sensor first.");
        return;
    }
    
    // Prefer current index, fall back to first selected row
    if (current.isValid()) {
        index = current;
    } else {
        index = selected.first();
    }
    
    int row = index.row();
    
    // Get current colour
    QModelIndex colourIndex = sensorsModel->index(row, 7); // Colour is in column 7
    QString currentColourHex = sensorsModel->data(colourIndex).toString();
    QColor currentColour = ColourUtils::hexToQColor(currentColourHex);
    
    // Show colour dialog
    QColor newColour = QColorDialog::getColor(currentColour, this, "Select Sensor Colour");
    
    if (newColour.isValid()) {
        QString newColourHex = ColourUtils::qColorToHex(newColour);
        sensorsModel->setData(colourIndex, newColourHex);
        
        if (sensorsModel->submitAll()) {
            sensorsModel->select();
        } else {
            QMessageBox::warning(this, "Error", "Failed to update colour: " + sensorsModel->lastError().text());
            sensorsModel->revertAll();
        }
    }
}

void SensorManager::updateButtonStates()
{
    QItemSelectionModel *selectionModel = sensorsTableView->selectionModel();
    if (!selectionModel) {
        removeButton->setEnabled(false);
        editButton->setEnabled(false);
        colourButton->setEnabled(false);
        return;
    }
    
    QModelIndexList selectedRows = selectionModel->selectedRows();
    bool hasValidSelection = !selectedRows.isEmpty();
    
    // Update button states
    removeButton->setEnabled(hasValidSelection && (sensorsModel->rowCount() > 1));
    editButton->setEnabled(hasValidSelection);
    colourButton->setEnabled(hasValidSelection);
}

void SensorManager::onSensorSelectionChanged()
{
    updateButtonStates();
}

void SensorManager::updateSensorValue(int sensorId, double rawValue)
{
    // This method would be called when new sensor data is received
    // Apply calibration and update the database
    
    for (int row = 0; row < sensorsModel->rowCount(); row++) {
        int id = sensorsModel->data(sensorsModel->index(row, 0)).toInt();
        if (id == sensorId) {
            // Get calibration parameters
            double offset = sensorsModel->data(sensorsModel->index(row, 8)).toDouble();
            double scale = sensorsModel->data(sensorsModel->index(row, 9)).toDouble();
            
            // Apply calibration: calibrated_value = (raw_value + offset) * scale
            double calibratedValue = (rawValue + offset) * scale;
            
            // Update the current value
            sensorsModel->setData(sensorsModel->index(row, 3), calibratedValue);
            
            if (sensorsModel->submitAll()) {
                sensorsModel->select();
                qDebug() << "Updated sensor" << sensorId << "value to" << calibratedValue;
            } else {
                qDebug() << "Failed to update sensor value:" << sensorsModel->lastError().text();
                sensorsModel->revertAll();
            }
            break;
        }
    }
}

void SensorManager::updateAllSensorValues()
{
    // This method would be called periodically to update all sensor values
    // from the hardware interface
    // Implementation would depend on the specific hardware interface
}