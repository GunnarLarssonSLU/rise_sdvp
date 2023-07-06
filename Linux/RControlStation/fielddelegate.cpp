#include <QtWidgets>

#include "fielddelegate.h"

//! [0]
void FieldDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option,
                         const QModelIndex &index) const
{
        QStyledItemDelegate::paint(painter, option, index);
//! [0]
}

//! [2]
QWidget *FieldDelegate::createEditor(QWidget *parent,
                                    const QStyleOptionViewItem &option,
                                    const QModelIndex &index) const

{
    return QStyledItemDelegate::createEditor(parent, option, index);
}
//! [2]

//! [3]
void FieldDelegate::setEditorData(QWidget *editor,
                                 const QModelIndex &index) const
{
        QStyledItemDelegate::setEditorData(editor, index);
}
//! [3]

//! [4]
void FieldDelegate::setModelData(QWidget *editor, QAbstractItemModel *model,
                                const QModelIndex &index) const
{
    /*
    if (index.data().canConvert<StarRating>()) {
        StarEditor *starEditor = qobject_cast<StarEditor *>(editor);
        model->setData(index, QVariant::fromValue(starEditor->starRating()));
    } else {
        QStyledItemDelegate::setModelData(editor, model, index);
    }*/
}
//! [4]

//! [5]
void FieldDelegate::commitAndCloseEditor()
{
   /* StarEditor *editor = qobject_cast<StarEditor *>(sender());
    emit commitData(editor);
    emit closeEditor(editor);*/
}

