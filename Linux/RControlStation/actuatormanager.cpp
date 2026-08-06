#include "actuatormanager.h"
#include "colourutils.h"
#include <QHeaderView>
#include <QMessageBox>
#include <QInputDialog>
#include <QDebug>
#include <QLabel>

ActuatorManager::ActuatorManager(QWidget *parent) : QWidget(parent)
{
    // Create UI elements
    actuatorsTableView = new QTableView(this);
    addButton = new QPushButton("Add Actuator", this);
    removeButton = new QPushButton("Remove Actuator", this);
    editButton = new QPushButton("Edit Actuator", this);
    colourButton = new QPushButton("Choose Colour", this);
    
    // Setup layouts
    mainLayout = new QVBoxLayout(this);
    buttonLayout = new QHBoxLayout();
    
    buttonLayout->addWidget(addButton);
    buttonLayout->addWidget(removeButton);
    buttonLayout->addWidget(editButton);
    buttonLayout->addWidget(colourButton);
    buttonLayout->addStretch();
    
    mainLayout->addWidget(actuatorsTableView);
    mainLayout->addLayout(buttonLayout);
    
    // Set layout properties
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(6);
    
    // Connect signals
    connect(addButton, &QPushButton::clicked, this, &ActuatorManager::addActuator);
    connect(removeButton, &QPushButton::clicked, this, &ActuatorManager::removeActuator);
    connect(editButton, &QPushButton::clicked, this, &ActuatorManager::editActuator);
    connect(colourButton, &QPushButton::clicked, this, &ActuatorManager::editActuatorColour);
    
    // Set selection mode
    actuatorsTableView->setSelectionMode(QAbstractItemView::SingleSelection);
    actuatorsTableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    
    // Initial button states
    removeButton->setEnabled(false);
    editButton->setEnabled(false);
    colourButton->setEnabled(false);
    
    // Setup actuator edit dialog
    setupActuatorEditDialog();
}

void ActuatorManager::setupUi(QSqlDatabase db)
{
    // Setup the table model
    actuatorsModel = new QSqlTableModel(this, db);
    setupTableModel();
    
    actuatorsTableView->setModel(actuatorsModel);
    
    // Configure table view
    actuatorsTableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    actuatorsTableView->setSelectionMode(QAbstractItemView::SingleSelection);
    actuatorsTableView->setEditTriggers(QAbstractItemView::NoEditTriggers); // Use edit button instead
    actuatorsTableView->horizontalHeader()->setStretchLastSection(true);
    actuatorsTableView->verticalHeader()->setVisible(false);
    
    // Set column headers
    actuatorsModel->setHeaderData(1, Qt::Horizontal, "Actuator Name");
    actuatorsModel->setHeaderData(2, Qt::Horizontal, "Type");
    actuatorsModel->setHeaderData(3, Qt::Horizontal, "Current Position");
    actuatorsModel->setHeaderData(4, Qt::Horizontal, "Min");
    actuatorsModel->setHeaderData(5, Qt::Horizontal, "Max");
    actuatorsModel->setHeaderData(6, Qt::Horizontal, "Colour");
    actuatorsModel->setHeaderData(7, Qt::Horizontal, "Reversible");
    
    // Hide ID column
    actuatorsTableView->setColumnHidden(0, true);
    
    // Set custom delegate for colour column
    ColourDelegate *colourDelegate = new ColourDelegate(this);
    actuatorsTableView->setItemDelegateForColumn(6, colourDelegate);
    
    // Connect selection model
    QItemSelectionModel *selectionModel = actuatorsTableView->selectionModel();
    connect(selectionModel, &QItemSelectionModel::selectionChanged, 
            this, &ActuatorManager::onActuatorSelectionChanged);
    
    // Refresh data
    actuatorsModel->select();
    
    // Set initial column widths
    actuatorsTableView->setColumnWidth(1, 200); // Name column
    actuatorsTableView->setColumnWidth(2, 120); // Type column
    actuatorsTableView->setColumnWidth(3, 100); // Current position column
    actuatorsTableView->setColumnWidth(4, 80);  // Min column
    actuatorsTableView->setColumnWidth(5, 80);  // Max column
    actuatorsTableView->setColumnWidth(6, 100); // Colour column
    actuatorsTableView->setColumnWidth(7, 80);  // Reversible column
}

void ActuatorManager::setupTableModel()
{
    actuatorsModel->setTable("actuators");
    actuatorsModel->setEditStrategy(QSqlTableModel::OnManualSubmit);
    
    // Select all columns
    actuatorsModel->select();
}

void ActuatorManager::setupActuatorEditDialog()
{
    actuatorEditDialog = new QDialog(this);
    actuatorEditDialog->setWindowTitle("Edit Actuator");
    actuatorEditDialog->setModal(true);
    
    QVBoxLayout *dialogLayout = new QVBoxLayout(actuatorEditDialog);
    
    // Main form
    QFormLayout *formLayout = new QFormLayout();
    
    // Type selection
    typeComboBox = new QComboBox();
    typeComboBox->addItem("Motor");
    typeComboBox->addItem("Valve");
    typeComboBox->addItem("Relay");
    typeComboBox->addItem("Servo");
    typeComboBox->addItem("Linear Actuator");
    typeComboBox->addItem("Pump");
    formLayout->addRow("Actuator Type:", typeComboBox);
    
    // Current position
    currentPositionSpinBox = new QDoubleSpinBox();
    currentPositionSpinBox->setRange(0.0, 100.0);
    currentPositionSpinBox->setDecimals(2);
    formLayout->addRow("Current Position:", currentPositionSpinBox);
    
    // Range controls
    QHBoxLayout *rangeLayout = new QHBoxLayout();
    minPositionSpinBox = new QDoubleSpinBox();
    minPositionSpinBox->setRange(-1000.0, 1000.0);
    minPositionSpinBox->setValue(0.0);
    minPositionSpinBox->setDecimals(2);
    
    maxPositionSpinBox = new QDoubleSpinBox();
    maxPositionSpinBox->setRange(-1000.0, 1000.0);
    maxPositionSpinBox->setValue(100.0);
    maxPositionSpinBox->setDecimals(2);
    
    rangeLayout->addWidget(new QLabel("Min:"));
    rangeLayout->addWidget(minPositionSpinBox);
    rangeLayout->addWidget(new QLabel("Max:"));
    rangeLayout->addWidget(maxPositionSpinBox);
    formLayout->addRow("Position Range:", rangeLayout);
    
    // Units
    unitsComboBox = new QComboBox();
    unitsComboBox->addItem("%");
    unitsComboBox->addItem("mm");
    unitsComboBox->addItem("cm");
    unitsComboBox->addItem("m");
    unitsComboBox->addItem("degrees");
    unitsComboBox->addItem("radians");
    unitsComboBox->addItem("unitless");
    formLayout->addRow("Units:", unitsComboBox);
    
    // Reversible checkbox
    isReversibleCheckBox = new QCheckBox("Actuator is reversible");
    isReversibleCheckBox->setChecked(true);
    formLayout->addRow("Reversible:", isReversibleCheckBox);
    
    // Dialog buttons
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &ActuatorManager::saveActuatorData);
    connect(buttonBox, &QDialogButtonBox::rejected, actuatorEditDialog, &QDialog::reject);
    
    dialogLayout->addLayout(formLayout);
    dialogLayout->addWidget(buttonBox);
}

void ActuatorManager::addActuator()
{
    bool ok;
    QString actuatorName = QInputDialog::getText(this, "Add Actuator", 
                                                  "Enter actuator name:", 
                                                  QLineEdit::Normal, 
                                                  "New Actuator", &ok);
    
    if (ok && !actuatorName.isEmpty()) {
        // Insert new record
        QSqlRecord record = actuatorsModel->record();
        record.setValue("name", actuatorName);
        record.setValue("type", "Motor"); // Default type
        record.setValue("current_position", 0.0); // Default position
        record.setValue("min_position", 0.0); // Default min
        record.setValue("max_position", 100.0); // Default max
        record.setValue("units", "%"); // Default units
        record.setValue("is_reversible", 1); // Reversible by default
        
        // Generate a colour for the new actuator
        int newRow = actuatorsModel->rowCount();
        QColor colour = ColourUtils::generateColourForAction(newRow);
        QString colourHex = ColourUtils::qColorToHex(colour);
        record.setValue("colour", colourHex);
        
        if (actuatorsModel->insertRecord(-1, record)) {
            if (actuatorsModel->submitAll()) {
                actuatorsModel->select();
                QMessageBox::information(this, "Success", "Actuator added successfully!");
            } else {
                QMessageBox::warning(this, "Error", "Failed to save actuator: " + actuatorsModel->lastError().text());
                actuatorsModel->revertAll();
            }
        } else {
            QMessageBox::warning(this, "Error", "Failed to add actuator: " + actuatorsModel->lastError().text());
        }
    }
}

void ActuatorManager::removeActuator()
{
    QModelIndex current = actuatorsTableView->selectionModel()->currentIndex();
    QModelIndexList selected = actuatorsTableView->selectionModel()->selectedRows();
    
    QModelIndex index;
    if (!current.isValid() && selected.isEmpty()) {
        QMessageBox::warning(this, "No Selection", "Please select an actuator to remove.");
        return;
    }
    
    // Prefer current index, fall back to first selected row
    if (current.isValid()) {
        index = current;
    } else {
        index = selected.first();
    }
    
    int row = index.row();
    QString actuatorName = actuatorsModel->data(actuatorsModel->index(row, 1)).toString();
    
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Remove Actuator", 
                                   QString("Are you sure you want to remove the actuator '%1'?").arg(actuatorName),
                                   QMessageBox::Yes|QMessageBox::No);
    
    if (reply == QMessageBox::Yes) {
        // Check if this is the last actuator
        if (actuatorsModel->rowCount() <= 1) {
            QMessageBox::warning(this, "Cannot Remove", "You cannot remove the last actuator. At least one actuator must remain.");
            return;
        }
        
        actuatorsModel->removeRow(row);
        
        if (actuatorsModel->submitAll()) {
            actuatorsModel->select();
            QMessageBox::information(this, "Success", "Actuator removed successfully!");
        } else {
            QMessageBox::warning(this, "Error", "Failed to remove actuator: " + actuatorsModel->lastError().text());
            actuatorsModel->revertAll();
        }
    }
}

void ActuatorManager::editActuator()
{
    QModelIndex current = actuatorsTableView->selectionModel()->currentIndex();
    
    if (!current.isValid()) {
        QMessageBox::warning(this, "No Selection", "Please select an actuator to edit.");
        return;
    }
    
    int actuatorId = actuatorsModel->data(actuatorsModel->index(current.row(), 0)).toInt();
    loadActuatorData(actuatorId);
    actuatorEditDialog->exec();
}

void ActuatorManager::loadActuatorData(int actuatorId)
{
    // Find the row with the given actuator ID
    for (int row = 0; row < actuatorsModel->rowCount(); row++) {
        int id = actuatorsModel->data(actuatorsModel->index(row, 0)).toInt();
        if (id == actuatorId) {
            // Load data
            QString name = actuatorsModel->data(actuatorsModel->index(row, 1)).toString();
            QString type = actuatorsModel->data(actuatorsModel->index(row, 2)).toString();
            double currentPos = actuatorsModel->data(actuatorsModel->index(row, 3)).toDouble();
            double minPos = actuatorsModel->data(actuatorsModel->index(row, 4)).toDouble();
            double maxPos = actuatorsModel->data(actuatorsModel->index(row, 5)).toDouble();
            QString units = actuatorsModel->data(actuatorsModel->index(row, 6)).toString();
            bool isReversible = actuatorsModel->data(actuatorsModel->index(row, 7)).toBool();
            
            // Set UI values
            actuatorEditDialog->setWindowTitle("Edit Actuator: " + name);
            
            int typeIndex = typeComboBox->findText(type);
            if (typeIndex >= 0) {
                typeComboBox->setCurrentIndex(typeIndex);
            }
            
            currentPositionSpinBox->setValue(currentPos);
            minPositionSpinBox->setValue(minPos);
            maxPositionSpinBox->setValue(maxPos);
            
            int unitsIndex = unitsComboBox->findText(units);
            if (unitsIndex >= 0) {
                unitsComboBox->setCurrentIndex(unitsIndex);
            }
            
            isReversibleCheckBox->setChecked(isReversible);
            
            break;
        }
    }
}

void ActuatorManager::saveActuatorData()
{
    QModelIndex current = actuatorsTableView->selectionModel()->currentIndex();
    
    if (!current.isValid()) {
        QMessageBox::warning(this, "No Selection", "No actuator selected to save.");
        return;
    }
    
    int row = current.row();
    
    // Update model data
    actuatorsModel->setData(actuatorsModel->index(row, 2), typeComboBox->currentText()); // type
    actuatorsModel->setData(actuatorsModel->index(row, 3), currentPositionSpinBox->value()); // current_position
    actuatorsModel->setData(actuatorsModel->index(row, 4), minPositionSpinBox->value()); // min_position
    actuatorsModel->setData(actuatorsModel->index(row, 5), maxPositionSpinBox->value()); // max_position
    actuatorsModel->setData(actuatorsModel->index(row, 6), unitsComboBox->currentText()); // units
    actuatorsModel->setData(actuatorsModel->index(row, 7), isReversibleCheckBox->isChecked() ? 1 : 0); // is_reversible
    
    if (actuatorsModel->submitAll()) {
        actuatorsModel->select();
        QMessageBox::information(this, "Success", "Actuator updated successfully!");
        actuatorEditDialog->accept();
    } else {
        QMessageBox::warning(this, "Error", "Failed to update actuator: " + actuatorsModel->lastError().text());
        actuatorsModel->revertAll();
    }
}

void ActuatorManager::editActuatorColour()
{
    QModelIndex current = actuatorsTableView->selectionModel()->currentIndex();
    QModelIndexList selected = actuatorsTableView->selectionModel()->selectedRows();
    
    QModelIndex index;
    if (!current.isValid() && selected.isEmpty()) {
        QMessageBox::warning(this, "No Selection", "Please select an actuator first.");
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
    QModelIndex colourIndex = actuatorsModel->index(row, 6); // Colour is in column 6
    QString currentColourHex = actuatorsModel->data(colourIndex).toString();
    QColor currentColour = ColourUtils::hexToQColor(currentColourHex);
    
    // Show colour dialog
    QColor newColour = QColorDialog::getColor(currentColour, this, "Select Actuator Colour");
    
    if (newColour.isValid()) {
        QString newColourHex = ColourUtils::qColorToHex(newColour);
        actuatorsModel->setData(colourIndex, newColourHex);
        
        if (actuatorsModel->submitAll()) {
            actuatorsModel->select();
        } else {
            QMessageBox::warning(this, "Error", "Failed to update colour: " + actuatorsModel->lastError().text());
            actuatorsModel->revertAll();
        }
    }
}

void ActuatorManager::updateButtonStates()
{
    QItemSelectionModel *selectionModel = actuatorsTableView->selectionModel();
    if (!selectionModel) {
        removeButton->setEnabled(false);
        editButton->setEnabled(false);
        colourButton->setEnabled(false);
        return;
    }
    
    QModelIndexList selectedRows = selectionModel->selectedRows();
    bool hasValidSelection = !selectedRows.isEmpty();
    
    // Update button states
    removeButton->setEnabled(hasValidSelection && (actuatorsModel->rowCount() > 1));
    editButton->setEnabled(hasValidSelection);
    colourButton->setEnabled(hasValidSelection);
}

void ActuatorManager::onActuatorSelectionChanged()
{
    updateButtonStates();
}

void ActuatorManager::setActuatorPosition(int actuatorId, double position)
{
    // This method would be called to set an actuator to a specific position
    // Implementation would depend on the specific hardware interface
    qDebug() << "Setting actuator" << actuatorId << "to position" << position;
    
    // Update the database with the new position
    for (int row = 0; row < actuatorsModel->rowCount(); row++) {
        int id = actuatorsModel->data(actuatorsModel->index(row, 0)).toInt();
        if (id == actuatorId) {
            actuatorsModel->setData(actuatorsModel->index(row, 3), position); // current_position
            if (actuatorsModel->submitAll()) {
                actuatorsModel->select();
            } else {
                qDebug() << "Failed to update actuator position:" << actuatorsModel->lastError().text();
                actuatorsModel->revertAll();
            }
            break;
        }
    }
}

void ActuatorManager::updateActuatorStates()
{
    // This method would be called periodically to update actuator states
    // based on feedback from the hardware
    // Implementation would depend on the specific hardware interface
}