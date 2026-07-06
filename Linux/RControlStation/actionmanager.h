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
};

#endif // ACTIONMANAGER_H