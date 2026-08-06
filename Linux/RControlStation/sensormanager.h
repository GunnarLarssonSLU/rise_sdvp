#ifndef SENSORMANAGER_H
#define SENSORMANAGER_H

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
#include <QGroupBox>
#include <QCheckBox>
#include <QLabel>
#include "colourutils.h"

class SensorManager : public QWidget
{
    Q_OBJECT

public:
    explicit SensorManager(QWidget *parent = nullptr);
    void setupUi(QSqlDatabase db);
    
    // Sensor update methods
    void updateSensorValue(int sensorId, double value);
    void updateAllSensorValues();
    
private slots:
    void addSensor();
    void removeSensor();
    void editSensor();
    void editSensorColour();
    void onSensorSelectionChanged();

private:
    // Main UI components
    QTableView *sensorsTableView;
    QSqlTableModel *sensorsModel;
    QPushButton *addButton;
    QPushButton *removeButton;
    QPushButton *editButton;
    QPushButton *colourButton;
    QVBoxLayout *mainLayout;
    QHBoxLayout *buttonLayout;
    
    // Sensor detail editing widgets
    QDialog *sensorEditDialog;
    QComboBox *typeComboBox;
    QDoubleSpinBox *currentValueSpinBox;
    QDoubleSpinBox *minValueSpinBox;
    QDoubleSpinBox *maxValueSpinBox;
    QComboBox *unitsComboBox;
    QDoubleSpinBox *calibrationOffsetSpinBox;
    QDoubleSpinBox *calibrationScaleSpinBox;
    
    void setupTableModel();
    void setupSensorEditDialog();
    void updateButtonStates();
    void loadSensorData(int sensorId);
    void saveSensorData();
    
    // Custom delegate for colour column
    class ColourDelegate : public QItemDelegate {
    public:
        ColourDelegate(QObject *parent = nullptr) : QItemDelegate(parent) {}
        
        void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override {
            if (index.column() == 7) { // Colour column (index 7 in sensors table)
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

#endif // SENSORMANAGER_H