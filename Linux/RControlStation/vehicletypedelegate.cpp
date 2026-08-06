#include "vehicletypedelegate.h"
#include <QComboBox>
#include <QPainter>
#include <QApplication>

VehicleTypeDelegate::VehicleTypeDelegate(QStandardItemModel *vehicleTypesModel, QObject *parent)
    : QStyledItemDelegate(parent), m_vehicleTypesModel(vehicleTypesModel)
{
}

QWidget *VehicleTypeDelegate::createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    QComboBox *editor = new QComboBox(parent);
    editor->setModel(m_vehicleTypesModel);
    return editor;
}

void VehicleTypeDelegate::setEditorData(QWidget *editor, const QModelIndex &index) const
{
    QComboBox *comboBox = static_cast<QComboBox*>(editor);
    QString vehicleTypeName = index.model()->data(index, Qt::DisplayRole).toString();
    
    // Find the index of the vehicle type with matching name
    for (int i = 0; i < m_vehicleTypesModel->rowCount(); ++i) {
        QStandardItem* item = m_vehicleTypesModel->item(i);
        if (item && item->text() == vehicleTypeName) {
            comboBox->setCurrentIndex(i);
            return;
        }
    }
}

void VehicleTypeDelegate::setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const
{
    QComboBox *comboBox = static_cast<QComboBox*>(editor);
    int selectedIndex = comboBox->currentIndex();
    if (selectedIndex >= 0) {
        QStandardItem* selectedItem = m_vehicleTypesModel->item(selectedIndex);
        if (selectedItem) {
            model->setData(index, selectedItem->text(), Qt::DisplayRole);
        }
    }
}

void VehicleTypeDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const
{
    // Draw the cell with the vehicle type name
    QString vehicleTypeName = index.model()->data(index, Qt::DisplayRole).toString();
    
    QStyleOptionViewItem opt = option;
    opt.text = vehicleTypeName;
    
    QApplication::style()->drawControl(QStyle::CE_ItemViewItem, &opt, painter);
}