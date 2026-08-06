#include "controlmanager.h"
#include "colourutils.h"
#include <QHeaderView>
#include <QMessageBox>
#include <QInputDialog>
#include <QDebug>
#include <QDialogButtonBox>
#include <QLabel>
#include <QFormLayout>

ControlManager::ControlManager(QWidget *parent) : QWidget(parent)
{
    // Create UI elements
    controlsTableView = new QTableView(this);
    addButton = new QPushButton("Add Control", this);
    removeButton = new QPushButton("Remove Control", this);
    editButton = new QPushButton("Edit Control", this);
    colourButton = new QPushButton("Choose Colour", this);
    
    // Setup layouts
    mainLayout = new QVBoxLayout(this);
    buttonLayout = new QHBoxLayout();
    
    buttonLayout->addWidget(addButton);
    buttonLayout->addWidget(removeButton);
    buttonLayout->addWidget(editButton);
    buttonLayout->addWidget(colourButton);
    buttonLayout->addStretch();
    
    mainLayout->addWidget(controlsTableView);
    mainLayout->addLayout(buttonLayout);
    
    // Set layout properties
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(6);
    
    // Connect signals
    connect(addButton, &QPushButton::clicked, this, &ControlManager::addControl);
    connect(removeButton, &QPushButton::clicked, this, &ControlManager::removeControl);
    connect(editButton, &QPushButton::clicked, this, &ControlManager::editControl);
    connect(colourButton, &QPushButton::clicked, this, &ControlManager::editControlColour);
    
    // Set selection mode
    controlsTableView->setSelectionMode(QAbstractItemView::SingleSelection);
    controlsTableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    
    // Initial button states
    removeButton->setEnabled(false);
    editButton->setEnabled(false);
    colourButton->setEnabled(false);
    
    // Setup control edit dialog
    setupControlEditDialog();
}

void ControlManager::setupUi(QSqlDatabase db)
{
    // Setup the table model
    controlsModel = new QSqlRelationalTableModel(this, db);
    setupTableModel();
    
    controlsTableView->setModel(controlsModel);
    
    // Configure table view
    controlsTableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    controlsTableView->setSelectionMode(QAbstractItemView::SingleSelection);
    controlsTableView->setEditTriggers(QAbstractItemView::NoEditTriggers); // Use edit button instead
    controlsTableView->horizontalHeader()->setStretchLastSection(true);
    controlsTableView->verticalHeader()->setVisible(false);
    
    // Set column headers
    controlsModel->setHeaderData(1, Qt::Horizontal, "Control Name");
    controlsModel->setHeaderData(2, Qt::Horizontal, "Type");
    controlsModel->setHeaderData(3, Qt::Horizontal, "Target Value");
    controlsModel->setHeaderData(4, Qt::Horizontal, "Active");
    controlsModel->setHeaderData(5, Qt::Horizontal, "Colour");
    
    // Hide ID column
    controlsTableView->setColumnHidden(0, true);
    
    // Set custom delegate for colour column
    ColourDelegate *colourDelegate = new ColourDelegate(this);
    controlsTableView->setItemDelegateForColumn(5, colourDelegate);
    
    // Connect selection model
    QItemSelectionModel *selectionModel = controlsTableView->selectionModel();
    connect(selectionModel, &QItemSelectionModel::selectionChanged, 
            this, &ControlManager::onControlSelectionChanged);
    
    // Refresh data
    controlsModel->select();
    
    // Set initial column widths
    controlsTableView->setColumnWidth(1, 200); // Name column
    controlsTableView->setColumnWidth(2, 100); // Type column
    controlsTableView->setColumnWidth(3, 100); // Target value column
    controlsTableView->setColumnWidth(4, 80);  // Active column
    controlsTableView->setColumnWidth(5, 100); // Colour column
}

void ControlManager::setupTableModel()
{
    controlsModel->setTable("controls");
    controlsModel->setEditStrategy(QSqlTableModel::OnManualSubmit);
    
    // Select all columns
    controlsModel->select();
}

void ControlManager::setupControlEditDialog()
{
    controlEditDialog = new QDialog(this);
    controlEditDialog->setWindowTitle("Edit Control");
    controlEditDialog->setModal(true);
    
    QVBoxLayout *dialogLayout = new QVBoxLayout(controlEditDialog);
    
    // Main form
    QFormLayout *formLayout = new QFormLayout();
    
    // Type selection
    typeComboBox = new QComboBox();
    typeComboBox->addItem("Logical", "logical");
    typeComboBox->addItem("PID", "pid");
    connect(typeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), 
            this, &ControlManager::updateControlTypeUI);
    formLayout->addRow("Control Type:", typeComboBox);
    
    // Target value
    targetValueSpinBox = new QDoubleSpinBox();
    targetValueSpinBox->setRange(0.0, 100.0);
    targetValueSpinBox->setDecimals(2);
    formLayout->addRow("Target Value:", targetValueSpinBox);
    
    // Active checkbox
    isActiveCheckBox = new QCheckBox("Control is Active");
    formLayout->addRow("Active:", isActiveCheckBox);
    
    // PID Group Box
    pidGroupBox = new QGroupBox("PID Parameters");
    QFormLayout *pidLayout = new QFormLayout(pidGroupBox);
    
    kpSpinBox = new QDoubleSpinBox();
    kpSpinBox->setRange(0.0, 100.0);
    kpSpinBox->setValue(1.0);
    kpSpinBox->setDecimals(3);
    pidLayout->addRow("Kp (Proportional):", kpSpinBox);
    
    kiSpinBox = new QDoubleSpinBox();
    kiSpinBox->setRange(0.0, 10.0);
    kiSpinBox->setValue(0.0);
    kiSpinBox->setDecimals(3);
    pidLayout->addRow("Ki (Integral):", kiSpinBox);
    
    kdSpinBox = new QDoubleSpinBox();
    kdSpinBox->setRange(0.0, 10.0);
    kdSpinBox->setValue(0.0);
    kdSpinBox->setDecimals(3);
    pidLayout->addRow("Kd (Derivative):", kdSpinBox);
    
    outputMinSpinBox = new QDoubleSpinBox();
    outputMinSpinBox->setRange(-100.0, 100.0);
    outputMinSpinBox->setValue(0.0);
    pidLayout->addRow("Output Min:", outputMinSpinBox);
    
    outputMaxSpinBox = new QDoubleSpinBox();
    outputMaxSpinBox->setRange(-100.0, 100.0);
    outputMaxSpinBox->setValue(100.0);
    pidLayout->addRow("Output Max:", outputMaxSpinBox);
    
    // Logical Group Box
    logicalGroupBox = new QGroupBox("Logical Operation");
    QFormLayout *logicalLayout = new QFormLayout(logicalGroupBox);
    
    logicalOperationComboBox = new QComboBox();
    logicalOperationComboBox->addItem("AND - All sensors must match", "AND");
    logicalOperationComboBox->addItem("OR - Any sensor must match", "OR");
    logicalOperationComboBox->addItem("NOT - Invert sensor logic", "NOT");
    logicalLayout->addRow("Operation:", logicalOperationComboBox);
    
    // Add group boxes to form
    formLayout->addRow(pidGroupBox);
    formLayout->addRow(logicalGroupBox);
    
    // Dialog buttons
    QDialogButtonBox *buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttonBox, &QDialogButtonBox::accepted, this, &ControlManager::saveControlData);
    connect(buttonBox, &QDialogButtonBox::rejected, controlEditDialog, &QDialog::reject);
    
    dialogLayout->addLayout(formLayout);
    dialogLayout->addWidget(buttonBox);
    
    // Initial UI state
    updateControlTypeUI(0); // Show logical by default
}

void ControlManager::updateControlTypeUI(int typeIndex)
{
    QString type = typeComboBox->itemData(typeIndex).toString();
    
    if (type == "pid") {
        pidGroupBox->setVisible(true);
        logicalGroupBox->setVisible(false);
    } else {
        pidGroupBox->setVisible(false);
        logicalGroupBox->setVisible(true);
    }
}

void ControlManager::addControl()
{
    bool ok;
    QString controlName = QInputDialog::getText(this, "Add Control", 
                                                "Enter control name:", 
                                                QLineEdit::Normal, 
                                                "New Control", &ok);
    
    if (ok && !controlName.isEmpty()) {
        // Insert new record
        QSqlRecord record = controlsModel->record();
        record.setValue("name", controlName);
        record.setValue("type", "logical"); // Default to logical
        record.setValue("target_value", 50.0); // Default target
        record.setValue("is_active", 0); // Not active by default
        
        // Generate a colour for the new control
        int newRow = controlsModel->rowCount();
        QColor colour = ColourUtils::generateColourForAction(newRow);
        QString colourHex = ColourUtils::qColorToHex(colour);
        record.setValue("colour", colourHex);
        
        // Set default PID values
        record.setValue("pid_kp", 1.0);
        record.setValue("pid_ki", 0.0);
        record.setValue("pid_kd", 0.0);
        record.setValue("pid_output_min", 0.0);
        record.setValue("pid_output_max", 100.0);
        
        // Set default logical operation
        record.setValue("logical_operation", "AND");
        
        if (controlsModel->insertRecord(-1, record)) {
            if (controlsModel->submitAll()) {
                controlsModel->select();
                QMessageBox::information(this, "Success", "Control added successfully!");
            } else {
                QMessageBox::warning(this, "Error", "Failed to save control: " + controlsModel->lastError().text());
                controlsModel->revertAll();
            }
        } else {
            QMessageBox::warning(this, "Error", "Failed to add control: " + controlsModel->lastError().text());
        }
    }
}

void ControlManager::removeControl()
{
    QModelIndex current = controlsTableView->selectionModel()->currentIndex();
    QModelIndexList selected = controlsTableView->selectionModel()->selectedRows();
    
    QModelIndex index;
    if (!current.isValid() && selected.isEmpty()) {
        QMessageBox::warning(this, "No Selection", "Please select a control to remove.");
        return;
    }
    
    // Prefer current index, fall back to first selected row
    if (current.isValid()) {
        index = current;
    } else {
        index = selected.first();
    }
    
    int row = index.row();
    QString controlName = controlsModel->data(controlsModel->index(row, 1)).toString();
    
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Remove Control", 
                                  QString("Are you sure you want to remove the control '%1'?").arg(controlName),
                                  QMessageBox::Yes|QMessageBox::No);
    
    if (reply == QMessageBox::Yes) {
        // Check if this is the last control
        if (controlsModel->rowCount() <= 1) {
            QMessageBox::warning(this, "Cannot Remove", "You cannot remove the last control. At least one control must remain.");
            return;
        }
        
        controlsModel->removeRow(row);
        
        if (controlsModel->submitAll()) {
            controlsModel->select();
            QMessageBox::information(this, "Success", "Control removed successfully!");
        } else {
            QMessageBox::warning(this, "Error", "Failed to remove control: " + controlsModel->lastError().text());
            controlsModel->revertAll();
        }
    }
}

void ControlManager::editControl()
{
    QModelIndex current = controlsTableView->selectionModel()->currentIndex();
    
    if (!current.isValid()) {
        QMessageBox::warning(this, "No Selection", "Please select a control to edit.");
        return;
    }
    
    int controlId = controlsModel->data(controlsModel->index(current.row(), 0)).toInt();
    loadControlData(controlId);
    controlEditDialog->exec();
}

void ControlManager::loadControlData(int controlId)
{
    // Find the row with the given control ID
    for (int row = 0; row < controlsModel->rowCount(); row++) {
        int id = controlsModel->data(controlsModel->index(row, 0)).toInt();
        if (id == controlId) {
            // Load basic data
            QString name = controlsModel->data(controlsModel->index(row, 1)).toString();
            QString type = controlsModel->data(controlsModel->index(row, 2)).toString();
            double targetValue = controlsModel->data(controlsModel->index(row, 3)).toDouble();
            bool isActive = controlsModel->data(controlsModel->index(row, 4)).toBool();
            
            // Set UI values
            controlEditDialog->setWindowTitle("Edit Control: " + name);
            
            // Find type index
            int typeIndex = typeComboBox->findData(type);
            if (typeIndex >= 0) {
                typeComboBox->setCurrentIndex(typeIndex);
            }
            
            targetValueSpinBox->setValue(targetValue);
            isActiveCheckBox->setChecked(isActive);
            
            // Load type-specific data
            if (type == "pid") {
                double kp = controlsModel->data(controlsModel->index(row, 6)).toDouble();
                double ki = controlsModel->data(controlsModel->index(row, 7)).toDouble();
                double kd = controlsModel->data(controlsModel->index(row, 8)).toDouble();
                double outputMin = controlsModel->data(controlsModel->index(row, 9)).toDouble();
                double outputMax = controlsModel->data(controlsModel->index(row, 10)).toDouble();
                
                kpSpinBox->setValue(kp);
                kiSpinBox->setValue(ki);
                kdSpinBox->setValue(kd);
                outputMinSpinBox->setValue(outputMin);
                outputMaxSpinBox->setValue(outputMax);
            } else {
                QString logicalOp = controlsModel->data(controlsModel->index(row, 11)).toString();
                int opIndex = logicalOperationComboBox->findData(logicalOp);
                if (opIndex >= 0) {
                    logicalOperationComboBox->setCurrentIndex(opIndex);
                }
            }
            
            updateControlTypeUI(typeComboBox->currentIndex());
            break;
        }
    }
}

void ControlManager::saveControlData()
{
    QModelIndex current = controlsTableView->selectionModel()->currentIndex();
    
    if (!current.isValid()) {
        QMessageBox::warning(this, "No Selection", "No control selected to save.");
        return;
    }
    
    int row = current.row();
    
    // Update model data
    controlsModel->setData(controlsModel->index(row, 2), typeComboBox->currentData()); // type
    controlsModel->setData(controlsModel->index(row, 3), targetValueSpinBox->value()); // target_value
    controlsModel->setData(controlsModel->index(row, 4), isActiveCheckBox->isChecked() ? 1 : 0); // is_active
    
    // Update type-specific data
    QString type = typeComboBox->currentData().toString();
    if (type == "pid") {
        controlsModel->setData(controlsModel->index(row, 6), kpSpinBox->value()); // pid_kp
        controlsModel->setData(controlsModel->index(row, 7), kiSpinBox->value()); // pid_ki
        controlsModel->setData(controlsModel->index(row, 8), kdSpinBox->value()); // pid_kd
        controlsModel->setData(controlsModel->index(row, 9), outputMinSpinBox->value()); // pid_output_min
        controlsModel->setData(controlsModel->index(row, 10), outputMaxSpinBox->value()); // pid_output_max
    } else {
        controlsModel->setData(controlsModel->index(row, 11), logicalOperationComboBox->currentData()); // logical_operation
    }
    
    if (controlsModel->submitAll()) {
        controlsModel->select();
        QMessageBox::information(this, "Success", "Control updated successfully!");
        controlEditDialog->accept();
    } else {
        QMessageBox::warning(this, "Error", "Failed to update control: " + controlsModel->lastError().text());
        controlsModel->revertAll();
    }
}

void ControlManager::editControlColour()
{
    QModelIndex current = controlsTableView->selectionModel()->currentIndex();
    QModelIndexList selected = controlsTableView->selectionModel()->selectedRows();
    
    QModelIndex index;
    if (!current.isValid() && selected.isEmpty()) {
        QMessageBox::warning(this, "No Selection", "Please select a control first.");
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
    QModelIndex colourIndex = controlsModel->index(row, 5); // Colour is in column 5
    QString currentColourHex = controlsModel->data(colourIndex).toString();
    QColor currentColour = ColourUtils::hexToQColor(currentColourHex);
    
    // Show colour dialog
    QColor newColour = QColorDialog::getColor(currentColour, this, "Select Control Colour");
    
    if (newColour.isValid()) {
        QString newColourHex = ColourUtils::qColorToHex(newColour);
        controlsModel->setData(colourIndex, newColourHex);
        
        if (controlsModel->submitAll()) {
            controlsModel->select();
        } else {
            QMessageBox::warning(this, "Error", "Failed to update colour: " + controlsModel->lastError().text());
            controlsModel->revertAll();
        }
    }
}

void ControlManager::updateButtonStates()
{
    QItemSelectionModel *selectionModel = controlsTableView->selectionModel();
    if (!selectionModel) {
        removeButton->setEnabled(false);
        editButton->setEnabled(false);
        colourButton->setEnabled(false);
        return;
    }
    
    QModelIndexList selectedRows = selectionModel->selectedRows();
    bool hasValidSelection = !selectedRows.isEmpty();
    
    // Update button states
    removeButton->setEnabled(hasValidSelection && (controlsModel->rowCount() > 1));
    editButton->setEnabled(hasValidSelection);
    colourButton->setEnabled(hasValidSelection);
}

void ControlManager::onControlSelectionChanged()
{
    updateButtonStates();
}

void ControlManager::updateControlStates()
{
    // This method would be called periodically to update control states
    // based on current sensor readings and target values
    // Implementation would depend on the specific control logic
}

void ControlManager::executeControls()
{
    // This method would execute the active controls
    // by sending commands to actuators based on the control logic
    // Implementation would depend on the specific hardware interface
}