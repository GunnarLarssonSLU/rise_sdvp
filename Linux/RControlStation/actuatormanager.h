#ifndef ACTUATORMANAGER_H
#define ACTUATORMANAGER_H

#include <QWidget>
#include <QTableView>
#include <QSqlTableModel>
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
#include <QDialog>
#include <QDialogButtonBox>
#include <QCheckBox>
#include <QLabel>
#include "colourutils.h"

class ActuatorManager : public QWidget
{
    Q_OBJECT

public:
    explicit ActuatorManager(QWidget *parent = nullptr);
    void setupUi(QSqlDatabase db);
    
    // Actuator control methods
    void setActuatorPosition(int actuatorId, double position);
    void updateActuatorStates();
    
private slots:
    void addActuator();
    void removeActuator();
    void editActuator();
    void editActuatorColour();
    void onActuatorSelectionChanged();

private:
    // Main UI components
    QTableView *actuatorsTableView;
    QSqlTableModel *actuatorsModel;
    QPushButton *addButton;
    QPushButton *removeButton;
    QPushButton *editButton;
    QPushButton *colourButton;
    QVBoxLayout *mainLayout;
    QHBoxLayout *buttonLayout;
    
    // Actuator detail editing widgets
    QDialog *actuatorEditDialog;
    QComboBox *typeComboBox;
    QDoubleSpinBox *currentPositionSpinBox;
    QDoubleSpinBox *minPositionSpinBox;
    QDoubleSpinBox *maxPositionSpinBox;
    QComboBox *unitsComboBox;
    QCheckBox *isReversibleCheckBox;
    
    void setupTableModel();
    void setupActuatorEditDialog();
    void updateButtonStates();
    void loadActuatorData(int actuatorId);
    void saveActuatorData();
    
    // Custom delegate for colour column
    class ColourDelegate : public QItemDelegate {
    public:
        ColourDelegate(QObject *parent = nullptr) : QItemDelegate(parent) {}
        
        void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override {
            if (index.column() == 6) { // Colour column (index 6 in actuators table)
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

#endif // ACTUATORMANAGER_H