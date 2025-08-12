#ifndef DATABASE_H
#define DATABASE_H

#include <QtSql>
#include <QWidget>

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
private:
    QSqlError initDb();
    QWidget* qw;

};

#endif // DATABASE_H
