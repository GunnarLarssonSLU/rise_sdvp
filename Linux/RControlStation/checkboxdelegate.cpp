#include "checkboxdelegate.h"
#include <QCheckBox>

CheckBoxDelegate::CheckBoxDelegate(QObject* parent)
    : QItemDelegate(parent)
{
}

QWidget* CheckBoxDelegate::createEditor(QWidget* parent, const QStyleOptionViewItem& option, const QModelIndex& index) const
{
    QCheckBox* editor = new QCheckBox(parent);
    return editor;
}

void CheckBoxDelegate::setEditorData(QWidget* editor, const QModelIndex& index) const
{
    bool value = index.model()->data(index, Qt::EditRole).toBool();
    QCheckBox* checkBox = static_cast<QCheckBox*>(editor);
    checkBox->setChecked(value);
}

void CheckBoxDelegate::setModelData(QWidget* editor, QAbstractItemModel* model, const QModelIndex& index) const
{
    QCheckBox* checkBox = static_cast<QCheckBox*>(editor);
    model->setData(index, checkBox->isChecked(), Qt::EditRole);
}

