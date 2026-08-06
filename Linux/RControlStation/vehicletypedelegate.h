#ifndef VEHICLETYPEDELEGATE_H
#define VEHICLETYPEDELEGATE_H

#include <QStyledItemDelegate>
#include <QStandardItemModel>

class VehicleTypeDelegate : public QStyledItemDelegate {
    Q_OBJECT
public:
    explicit VehicleTypeDelegate(QStandardItemModel *vehicleTypesModel, QObject *parent = nullptr);
    
    QWidget *createEditor(QWidget *parent, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
    void setEditorData(QWidget *editor, const QModelIndex &index) const override;
    void setModelData(QWidget *editor, QAbstractItemModel *model, const QModelIndex &index) const override;
    void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override;
    
private:
    QStandardItemModel *m_vehicleTypesModel;
};

#endif // VEHICLETYPEDELEGATE_H