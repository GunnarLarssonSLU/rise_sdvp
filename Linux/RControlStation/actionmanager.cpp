#include "actionmanager.h"
#include "colourutils.h"
#include <QHeaderView>
#include <QMessageBox>
#include <QInputDialog>
#include <QDebug>

ActionManager::ActionManager(QWidget *parent) : QWidget(parent)
{
    // Create UI elements
    actionsTableView = new QTableView(this);
    addButton = new QPushButton("Add Action", this);
    removeButton = new QPushButton("Remove Action", this);
    colourButton = new QPushButton("Choose Colour", this);
    
    // Setup layouts
    mainLayout = new QVBoxLayout(this);
    buttonLayout = new QHBoxLayout();
    
    buttonLayout->addWidget(addButton);
    buttonLayout->addWidget(removeButton);
    buttonLayout->addWidget(colourButton);
    buttonLayout->addStretch();
    
    mainLayout->addWidget(actionsTableView);
    mainLayout->addLayout(buttonLayout);
    
    // Set layout properties
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(6);
    
    // Connect signals
    connect(addButton, &QPushButton::clicked, this, &ActionManager::addAction);
    connect(removeButton, &QPushButton::clicked, this, &ActionManager::removeAction);
    connect(colourButton, &QPushButton::clicked, this, &ActionManager::editActionColour);
    
    // Set selection mode first
    actionsTableView->setSelectionMode(QAbstractItemView::SingleSelection);
    actionsTableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    
    // Set selection mode and behavior first
    actionsTableView->setSelectionMode(QAbstractItemView::SingleSelection);
    actionsTableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    
    // Ensure we have a model before creating selection model
    if (!actionsTableView->model()) {
        qDebug() << "ERROR: Table view has no model - setting dummy model";
        // This should not happen, but let's be safe
        actionsTableView->setModel(new QSqlTableModel(this));
    }
    
    // Get or create selection model
    QItemSelectionModel *selectionModel = actionsTableView->selectionModel();
    if (!selectionModel) {
        qDebug() << "Creating new selection model for table view";
        selectionModel = new QItemSelectionModel(actionsTableView->model());
        actionsTableView->setSelectionModel(selectionModel);
    }
    
    // Now connect signals safely
    connect(selectionModel, &QItemSelectionModel::selectionChanged, 
            this, &ActionManager::onActionSelectionChanged);
    connect(selectionModel, &QItemSelectionModel::currentChanged, 
            this, &ActionManager::onActionSelectionChanged);
    connect(selectionModel, &QItemSelectionModel::currentRowChanged, 
            this, &ActionManager::onActionSelectionChanged);
    connect(actionsTableView, &QTableView::clicked, 
            this, &ActionManager::onActionSelectionChanged);
    
    qDebug() << "Successfully connected all selection model signals";
    
    // Also connect clicked signal for direct interaction
    connect(actionsTableView, &QTableView::clicked, 
            this, &ActionManager::onActionSelectionChanged);
    
    // Initial button states
    removeButton->setEnabled(false);
    colourButton->setEnabled(false);
    qDebug() << "ActionManager initialized - buttons disabled initially";
}

void ActionManager::setupUi(QSqlDatabase db)
{
    // Setup the table model
    actionsModel = new QSqlTableModel(this, db);
    setupTableModel();
    
    actionsTableView->setModel(actionsModel);
    
    // Configure table view
    actionsTableView->setSelectionBehavior(QAbstractItemView::SelectRows);
    actionsTableView->setSelectionMode(QAbstractItemView::SingleSelection);
    actionsTableView->setEditTriggers(QAbstractItemView::DoubleClicked); // Allow double-click editing
    actionsTableView->horizontalHeader()->setStretchLastSection(true);
    actionsTableView->verticalHeader()->setVisible(false);
    
    // Set column headers
    actionsModel->setHeaderData(1, Qt::Horizontal, "Action Name");
    actionsModel->setHeaderData(2, Qt::Horizontal, "Colour");
    
    // Hide ID column
    actionsTableView->setColumnHidden(0, true);
    
    // Set custom delegate for colour column
    ColourDelegate *colourDelegate = new ColourDelegate(this);
    actionsTableView->setItemDelegateForColumn(2, colourDelegate);
    
    // Connect double-click on colour column to edit colour
    connect(actionsTableView, &QTableView::doubleClicked, this, [this](const QModelIndex &index) {
        if (index.column() == 2) { // Colour column double-clicked
            editActionColour();
        }
    });
    
    // Refresh data
    actionsModel->select();
    
    // Set initial column widths
    actionsTableView->setColumnWidth(1, 200); // Action name column
    actionsTableView->setColumnWidth(2, 100); // Colour column
    
    // Force a selection update after model is loaded
    QTimer::singleShot(200, this, [this]() {
        qDebug() << "Timer-based selection update triggered";
        
        // Safety checks
        if (!actionsModel) {
            qDebug() << "ERROR: No model available in timer callback";
            return;
        }
        
        if (!actionsTableView->selectionModel()) {
            qDebug() << "ERROR: No selection model available in timer callback";
            return;
        }
        
        // If we have data, select the first row to enable buttons
        if (actionsModel->rowCount() > 0) {
            QModelIndex firstIndex = actionsModel->index(0, 0); // Select first row, first column
            if (firstIndex.isValid()) {
                actionsTableView->selectionModel()->select(firstIndex, QItemSelectionModel::SelectCurrent | QItemSelectionModel::Rows);
                actionsTableView->selectionModel()->setCurrentIndex(firstIndex, QItemSelectionModel::SelectCurrent);
                actionsTableView->scrollTo(firstIndex);
                qDebug() << "Timer auto-selected first row to enable buttons";
            } else {
                qDebug() << "ERROR: First index is invalid";
            }
        } else {
            qDebug() << "Timer: No data in model, buttons remain disabled";
        }
        
        // Force update button states
        onActionSelectionChanged();
    });
}

void ActionManager::setupTableModel()
{
    actionsModel->setTable("actions");
    actionsModel->setEditStrategy(QSqlTableModel::OnManualSubmit);
    
    // Select all columns
    actionsModel->select();
}

void ActionManager::addAction()
{
    bool ok;
    QString actionName = QInputDialog::getText(this, "Add Action", 
                                                "Enter action name:", 
                                                QLineEdit::Normal, 
                                                "New Action", &ok);
    
    if (ok && !actionName.isEmpty()) {
        // Generate a colour for the new action
        int newRow = actionsModel->rowCount();
        QColor colour = ColourUtils::generateColourForAction(newRow);
        QString colourHex = ColourUtils::qColorToHex(colour);
        
        // Insert new record
        QSqlRecord record = actionsModel->record();
        record.setValue("name", actionName);
        record.setValue("colour", colourHex);
        
        if (actionsModel->insertRecord(-1, record)) {
            if (actionsModel->submitAll()) {
                actionsModel->select();
                QMessageBox::information(this, "Success", "Action added successfully!");
            } else {
                QMessageBox::warning(this, "Error", "Failed to save action: " + actionsModel->lastError().text());
                actionsModel->revertAll();
            }
        } else {
            QMessageBox::warning(this, "Error", "Failed to add action: " + actionsModel->lastError().text());
        }
    }
}

void ActionManager::removeAction()
{
    QModelIndex current = actionsTableView->selectionModel()->currentIndex();
    QModelIndexList selected = actionsTableView->selectionModel()->selectedRows();
    
    QModelIndex index;
    if (!current.isValid() && selected.isEmpty()) {
        QMessageBox::warning(this, "No Selection", "Please select an action to remove.");
        return;
    }
    
    // Prefer current index, fall back to first selected row
    if (current.isValid()) {
        index = current;
    } else {
        index = selected.first();
    }
    
    int row = index.row();
    QString actionName = actionsModel->data(actionsModel->index(row, 1)).toString();
    
    QMessageBox::StandardButton reply;
    reply = QMessageBox::question(this, "Remove Action", 
                                  QString("Are you sure you want to remove the action '%1'?").arg(actionName),
                                  QMessageBox::Yes|QMessageBox::No);
    
    if (reply == QMessageBox::Yes) {
        // Check if this is the last action - we might want to keep at least one
        if (actionsModel->rowCount() <= 1) {
            QMessageBox::warning(this, "Cannot Remove", "You cannot remove the last action. At least one action must remain.");
            return;
        }
        
        actionsModel->removeRow(row);
        
        if (actionsModel->submitAll()) {
            actionsModel->select();
            QMessageBox::information(this, "Success", "Action removed successfully!");
        } else {
            QMessageBox::warning(this, "Error", "Failed to remove action: " + actionsModel->lastError().text());
            actionsModel->revertAll();
        }
    }
}

void ActionManager::editActionColour()
{
    QModelIndex current = actionsTableView->selectionModel()->currentIndex();
    QModelIndexList selected = actionsTableView->selectionModel()->selectedRows();
    
    QModelIndex index;
    if (!current.isValid() && selected.isEmpty()) {
        QMessageBox::warning(this, "No Selection", "Please select an action first.");
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
    QModelIndex colourIndex = actionsModel->index(row, 2); // Colour is in column 2
    QString currentColourHex = actionsModel->data(colourIndex).toString();
    QColor currentColour = ColourUtils::hexToQColor(currentColourHex);
    
    // Show colour dialog
    QColor newColour = QColorDialog::getColor(currentColour, this, "Select Action Colour");
    
    if (newColour.isValid()) {
        QString newColourHex = ColourUtils::qColorToHex(newColour);
        actionsModel->setData(colourIndex, newColourHex);
        
        if (actionsModel->submitAll()) {
            actionsModel->select();
        } else {
            QMessageBox::warning(this, "Error", "Failed to update colour: " + actionsModel->lastError().text());
            actionsModel->revertAll();
        }
    }
}

void ActionManager::onActionSelectionChanged()
{
    qDebug() << "=== Selection change triggered ===";
    
    // Defensive programming - check everything before proceeding
    if (!actionsTableView) {
        qDebug() << "ERROR: actionsTableView is null!";
        return;
    }
    
    if (!actionsModel) {
        qDebug() << "ERROR: actionsModel is null!";
        return;
    }
    
    // Get selection model and check its state
    QItemSelectionModel *selectionModel = actionsTableView->selectionModel();
    if (!selectionModel) {
        qDebug() << "ERROR: No selection model available!";
        removeButton->setEnabled(false);
        colourButton->setEnabled(false);
        return;
    }
    
    // Check selected rows
    QModelIndexList selectedRows = selectionModel->selectedRows();
    qDebug() << "Selected rows count:" << selectedRows.size();
    
    // Check current index
    QModelIndex currentIndex = selectionModel->currentIndex();
    qDebug() << "Current index valid:" << currentIndex.isValid();
    if (currentIndex.isValid()) {
        qDebug() << "Current index row:" << currentIndex.row() << "col:" << currentIndex.column();
    }
    
    // Determine if we have a valid selection
    bool hasValidSelection = false;
    
    // Check if we have selected rows
    if (!selectedRows.isEmpty()) {
        QModelIndex firstSelected = selectedRows.first();
        qDebug() << "First selected row:" << firstSelected.row();
        hasValidSelection = true;
    }
    
    // Check if we have a valid current index (even if not officially "selected")
    if (currentIndex.isValid() && currentIndex.row() >= 0) {
        qDebug() << "Using current index as selection";
        hasValidSelection = true;
    }
    
    // Additional safety checks
    qDebug() << "Model row count:" << actionsModel->rowCount();
    if (actionsModel->rowCount() == 0) {
        hasValidSelection = false;
        qDebug() << "Model is empty, disabling buttons";
    }
    
    // Update button states
    bool buttonsEnabled = hasValidSelection && (actionsModel->rowCount() > 0);
    removeButton->setEnabled(buttonsEnabled);
    colourButton->setEnabled(buttonsEnabled);
    
    qDebug() << "Final button states - Remove:" << removeButton->isEnabled() 
             << "Colour:" << colourButton->isEnabled();
    qDebug() << "=== End selection change ===";
}