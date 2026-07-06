#include "database.h"
#include <QtWidgets>
#include <QStandardPaths>
#include <QDir>
#include <QDebug>
#include <QCoreApplication>

database::database(QWidget* _qw) {
    qw=_qw;
    // Initialize the database:
    QSqlError err = initDb();
    if (err.type() != QSqlError::NoError) {
        showError(err);
        return;
    }
    
    // Ensure required tables exist
    ensureActionsTableExists();
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
    db = QSqlDatabase::addDatabase("QSQLITE");
    
    // Try different locations for the database file
    QStringList dbPaths;
    
    // 1. First try current directory (for development)
    dbPaths << "data.db";
    
    // 2. Try AppImage data directory
    QString appImagePath = QCoreApplication::applicationDirPath();
    dbPaths << appImagePath + "/../share/RControlStation/data.db";
    dbPaths << appImagePath + "/../../share/RControlStation/data.db";
    dbPaths << "/usr/share/RControlStation/data.db";
    
    // 3. Try common data directories
    dbPaths << QStandardPaths::writableLocation(QStandardPaths::AppDataLocation) + "/data.db";
    // DataLocation doesn't exist in Qt 6, use AppDataLocation instead
    // dbPaths << QStandardPaths::writableLocation(QStandardPaths::DataLocation) + "/data.db";
    
    // Try each path until we find a working database
    foreach (const QString &path, dbPaths) {
        db.setDatabaseName(path);
        if (db.open()) {
            qDebug() << "Database opened from:" << path;
            return QSqlError();
        }
    }
    
    // If no database found, create one in the writable data location
    QString defaultPath = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    QDir().mkpath(defaultPath);
    db.setDatabaseName(defaultPath + "/data.db");
    if (!db.open())
        return db.lastError();
    
    qDebug() << "Created new database at:" << defaultPath + "/data.db";
    return QSqlError();
}

QSqlDatabase database::getDb()
{
    return db;
}

void database::ensureActionsTableExists()
{
    QSqlQuery query(db);
    
    // Check if actions table exists
    if (!query.exec("SELECT name FROM sqlite_master WHERE type='table' AND name='actions'")) {
        qDebug() << "Error checking for actions table:" << query.lastError().text();
        return;
    }
    
    // If table doesn't exist, create it
    if (!query.next()) {
        QString createTableSql = 
            "CREATE TABLE actions ("
            "id INTEGER PRIMARY KEY AUTOINCREMENT, "
            "name TEXT NOT NULL, "
            "colour TEXT DEFAULT '#CCCCCC'"
            ")";
        
        if (!query.exec(createTableSql)) {
            qDebug() << "Error creating actions table:" << query.lastError().text();
            return;
        }
        
        qDebug() << "Created actions table with colour column";
        
        // Add some default actions if the table is new
        insertDefaultActions();
    } else {
        // Table exists, check if colour column exists
        QSqlQuery columnQuery(db);
        if (columnQuery.exec("PRAGMA table_info(actions)")) {
            bool hasColourColumn = false;
            while (columnQuery.next()) {
                QString columnName = columnQuery.value(1).toString();
                if (columnName == "colour") {
                    hasColourColumn = true;
                    break;
                }
            }
            
            // If colour column doesn't exist, add it
            if (!hasColourColumn) {
                if (!query.exec("ALTER TABLE actions ADD COLUMN colour TEXT DEFAULT '#CCCCCC'")) {
                    qDebug() << "Error adding colour column:" << query.lastError().text();
                } else {
                    qDebug() << "Added colour column to actions table";
                    // Update existing actions with generated colours
                    updateActionsWithGeneratedColours();
                }
            }
        }
    }
}

void database::insertDefaultActions()
{
    // Insert some default actions if the table is empty
    QStringList defaultActions = {
        "Takeoff", "Land", "Emergency Stop", "Return to Home", "Hover",
        "Waypoint Navigation", "Manual Control", "Auto Mission", "Loiter", "Follow Me"
    };
    
    QSqlQuery query(db);
    query.prepare("INSERT INTO actions (name, colour) VALUES (?, ?)");
    
    for (int i = 0; i < defaultActions.size(); i++) {
        query.addBindValue(defaultActions[i]);
        // Generate a colour based on the action index
        QString colour = generateColourForAction(i);
        query.addBindValue(colour);
        
        if (!query.exec()) {
            qDebug() << "Error inserting default action:" << query.lastError().text();
        }
    }
}

void database::updateActionsWithGeneratedColours()
{
    // Update existing actions with generated colours
    QSqlQuery selectQuery(db);
    if (!selectQuery.exec("SELECT id, name FROM actions")) {
        qDebug() << "Error selecting actions:" << selectQuery.lastError().text();
        return;
    }
    
    QSqlQuery updateQuery(db);
    updateQuery.prepare("UPDATE actions SET colour = ? WHERE id = ?");
    
    int index = 0;
    while (selectQuery.next()) {
        int actionId = selectQuery.value(0).toInt();
        QString colour = generateColourForAction(index++);
        
        updateQuery.addBindValue(colour);
        updateQuery.addBindValue(actionId);
        
        if (!updateQuery.exec()) {
            qDebug() << "Error updating action colour:" << updateQuery.lastError().text();
        }
    }
}

QString database::generateColourForAction(int actionIndex)
{
    // Generate a distinct colour using HSV colour space for better visual distinction
    // Use golden ratio conjugation for good distribution
    int hue = (actionIndex * 137) % 360;  // Golden ratio (≈137.5°) for good distribution
    
    // Convert HSV to RGB and then to hex string
    QColor colour;
    colour.setHsv(hue, 200, 255);  // Saturate and brighten for vibrant colours
    
    return colour.name(QColor::HexRgb);  // Returns "#RRGGBB" format
}
