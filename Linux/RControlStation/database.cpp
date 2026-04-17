#include "database.h"
#include <QtWidgets>

database::database(QWidget* _qw) {
    qw=_qw;
    // Initialize the database:
    QSqlError err = initDb();
    if (err.type() != QSqlError::NoError) {
        showError(err);
        return;
    }
}

QVariant database::addFarm(const QString &name)
{
    const auto INSERT_LOCATION_SQL = QLatin1String(R"(
        insert into locations(name)
                          values(?)
        )");
    QSqlQuery q;
    if (q.prepare(INSERT_LOCATION_SQL))
    {
        q.addBindValue(name);
        q.exec();
        return q.lastInsertId();
    };

}

void database::addPath(const QString &pathname, const QString &xmlstring,const QVariant &fieldId)
{
    const auto INSERT_FIELD_SQL = QLatin1String(R"(
            insert into paths(name,field,xml)
                              values(?,?,?)
            )");
    QSqlQuery q;
    if (q.prepare(INSERT_FIELD_SQL))
    {
        q.addBindValue(pathname);
        q.addBindValue(fieldId);
        q.addBindValue(xmlstring);
        q.exec();
    };
}

#include <QSqlQuery>
#include <QSqlError>
#include <QDebug>
#include <QVariant>

void database::updateFarmLocation(int farmId, double latitude, double longitude)
{
    QString queryString =
        "UPDATE locations "
        "SET longitude=:longitude,latitude=:latitude "
        "WHERE id=:locationid";

    QSqlQuery query;
    query.prepare(queryString);

    query.bindValue(":longitude", longitude);
    query.bindValue(":latitude", latitude);
    query.bindValue(":locationid", farmId);

    qDebug() << "Prepared Query:" << queryString;
    qDebug() << "Bound Values:" << longitude << latitude << farmId;

    if (!query.exec()) {
        qDebug() << "Failed to execute query:" << query.lastError().text();
    } else {
        qDebug() << "Query executed successfully.";
    }
}

/*
void database::updateFarmLocation(int farmid,double latitude, double longitude)
{
    QSqlQuery q;
    if (q.prepare("UPDATE locations SET longitude = :longitude, latitude = :latitude WHERE locationid = :farmid"))
    {
        q.addBindValue(":latitude",QVariant(longitude));
        q.addBindValue(":longitude",QVariant(latitude));
        q.addBindValue(":farmid",farmid);
        if (!q.exec())
        {
            qDebug() << "Failed to execute query:" << q.lastError();
        }
    } else
    {
        qDebug() << "Failed to prepare query:" << q.lastError();
    }
};
*/

void database::addField(QString fieldname,int farmid,QString filename)
{
    const auto INSERT_FIELD_SQL = QLatin1String(R"(
        insert into fields(name,location,storedinfile)
                          values(?,?,?)
        )");
    QSqlQuery q;
    if (q.prepare(INSERT_FIELD_SQL))
    {
        q.addBindValue(fieldname);
        q.addBindValue(farmid);
        q.addBindValue(filename);
        q.exec();
    };
}

void database::deleteField(const QVariant &fieldId)
{
    const auto DELETE_FIELD_SQL = QLatin1String(R"(DELETE FROM fields WHERE id=:fieldid)");
    QSqlQuery q;
    if (q.prepare(DELETE_FIELD_SQL))
    {
        q.bindValue(":fieldid",fieldId);
        q.exec();
        qDebug() << "DELETE FIELD!!!: " << fieldId;
    } else {
        qDebug() << "Oups..";
    };
}

void database::showError(const QSqlError &err)
{
    QMessageBox::critical(qw, "Unable to initialize Database",
                          "Error initializing database: " + err.text());
}


QSqlError database::initDb()
{
    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE");
    db.setDatabaseName("data.db");
    if (!db.open())
        return db.lastError();
    return QSqlError();
}
