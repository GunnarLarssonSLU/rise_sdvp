#ifndef CONTROLMANAGER_H
#define CONTROLMANAGER_H

#include <QWidget>
#include <QTableView>
#include <QSqlTableModel>
#include <QSqlRelationalTableModel>
#include <QSqlRecord>
#include <QSqlError>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QColorDialog>
#include <QStandardItemModel>
#include <QItemDelegate>
#include <QPainter>
#include <QMouseEvent>
#include <QTimer>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QCheckBox>
#include <QLabel>
#include "colourutils.h"

class ControlManager : public QWidget
{
    Q_OBJECT

public:
    explicit ControlManager(QWidget *parent = nullptr);
    void setupUi(QSqlDatabase db);
    
    // Control execution methods
    void updateControlStates();
    void executeControls();
    
private slots:
    void addControl();
    void removeControl();
    void editControl();
    void editControlColour();
    void onControlSelectionChanged();
    void updateControlTypeUI(int typeIndex);

private:
    // Main UI components
    QTableView *controlsTableView;
    QSqlRelationalTableModel *controlsModel;
    QPushButton *addButton;
    QPushButton *removeButton;
    QPushButton *editButton;
    QPushButton *colourButton;
    QVBoxLayout *mainLayout;
    QHBoxLayout *buttonLayout;
    
    // Control detail editing widgets
    QDialog *controlEditDialog;
    QComboBox *typeComboBox;
    QDoubleSpinBox *targetValueSpinBox;
    QCheckBox *isActiveCheckBox;
    
    // PID control widgets
    QGroupBox *pidGroupBox;
    QDoubleSpinBox *kpSpinBox;
    QDoubleSpinBox *kiSpinBox;
    QDoubleSpinBox *kdSpinBox;
    QDoubleSpinBox *outputMinSpinBox;
    QDoubleSpinBox *outputMaxSpinBox;
    
    // Logical control widgets
    QGroupBox *logicalGroupBox;
    QComboBox *logicalOperationComboBox;
    
    void setupTableModel();
    void setupControlEditDialog();
    void updateButtonStates();
    void loadControlData(int controlId);
    void saveControlData();
    
    // Custom delegate for colour column
    class ColourDelegate : public QItemDelegate {
    public:
        ColourDelegate(QObject *parent = nullptr) : QItemDelegate(parent) {}
        
        void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override {
            if (index.column() == 5) { // Colour column (index 5 in controls table)
                QString colourHex = index.data().toString();
                QColor colour = ColourUtils::hexToQColor(colourHex);
                
                // Draw colour swatch
                painter->save();
                QRect rect = option.rect.adjusted(2, 2, -2, -2);
                painter->fillRect(rect, colour);
                painter->setPen(QColor(128, 128, 128));
                painter->drawRect(rect);
                painter->restore();
            } else {
                QItemDelegate::paint(painter, option, index);
            }
        }
    };
};

#endif // CONTROLMANAGER_H