#ifndef DATABASE_H
#define DATABASE_H

#include <QtSql>
#include <QWidget>

struct ControllerInfo {
    int id;
    QString name;
    // Add other controller properties as needed
};

class database
{
public:
    database(QWidget* _qw);
    QVariant addFarm(const QString &name);
    void addPath(const QString &name,const QString &xmlstring, const QVariant &locationId);
    void deleteField(const QVariant &fieldId);

    void addField(QString fieldname,int farmid,QString filename);
    void updateFarmLocation(int farmid,double latitude, double longitude);
    void showError(const QSqlError &err);
    QSqlDatabase getDb();
    void ensureControllersTableExists();
    void ensureControlsTableExists();
    void ensureActuatorsTableExists();
    void ensureSensorsTableExists();
    void ensureControlRelationshipsExist();
    
    // Old method kept for compatibility
    void ensureActionsTableExists();
    
    // XML format conversion functions
    static QString convertPointToVersion2(const LocPoint &point);
    static LocPoint convertPointFromVersion2(const QString &xmlPoint);
    
    // Methods for inserting default data
    void insertDefaultControls();
    void insertDefaultActuators();
    void insertDefaultSensors();
    
    // Controller methods
    QList<ControllerInfo> getAllControllers();
    ControllerInfo getControllerById(int id);
    void addController(const QString& name);
private:
    QSqlError initDb();
    void insertDefaultActions();
    void updateActionsWithGeneratedColours();
    QString generateColourForAction(int actionIndex);
    QWidget* qw;
    QSqlDatabase db;

};

#endif // DATABASE_H
