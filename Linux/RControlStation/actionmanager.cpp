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
    connect(actionsTableView->selectionModel(), &QItemSelectionModel::selectionChanged, 
            this, &ActionManager::onActionSelectionChanged);
    connect(actionsTableView->selectionModel(), &QItemSelectionModel::currentChanged, 
            this, &ActionManager::onActionSelectionChanged);
    
    // Initial button states
    removeButton->setEnabled(false);
    colourButton->setEnabled(false);
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
    QModelIndexList selected = actionsTableView->selectionModel()->selectedRows();
    QModelIndex current = actionsTableView->selectionModel()->currentIndex();
    
    bool hasSelection = !selected.isEmpty() || current.isValid();
    
    // Debug output to help diagnose selection issues
    qDebug() << "Selection changed - selected rows:" << selected.count() 
             << "current index valid:" << current.isValid();
    
    removeButton->setEnabled(hasSelection);
    colourButton->setEnabled(hasSelection);
}