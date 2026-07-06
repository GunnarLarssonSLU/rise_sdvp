#ifndef ACTIONMANAGER_H
#define ACTIONMANAGER_H

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
#include "colourutils.h"

class ActionManager : public QWidget
{
    Q_OBJECT

public:
    explicit ActionManager(QWidget *parent = nullptr);
    void setupUi(QSqlDatabase db);
    
private slots:
    void addAction();
    void removeAction();
    void editActionColour();
    void onActionSelectionChanged();
    
private:
    QTableView *actionsTableView;
    QSqlTableModel *actionsModel;
    QPushButton *addButton;
    QPushButton *removeButton;
    QPushButton *colourButton;
    QVBoxLayout *mainLayout;
    QHBoxLayout *buttonLayout;
    
    void setupTableModel();
    void updateButtonStates();
    
    // Custom delegate for colour column
    class ColourDelegate : public QItemDelegate {
    public:
        ColourDelegate(QObject *parent = nullptr) : QItemDelegate(parent) {}
        
        void paint(QPainter *painter, const QStyleOptionViewItem &option, const QModelIndex &index) const override {
            if (index.column() == 2) { // Colour column
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

#endif // ACTIONMANAGER_H