/*
    Copyright 2016 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "osmclient.h"
#include "qmessagebox.h"
#include <QDebug>
#include <QPainter>

OsmClient::OsmClient(QObject *parent) : QObject(parent)
{
    mMaxMemoryTiles = 600;
    mMaxDownloadingTiles = 6;
    mHddTilesLoaded = 0;
    mTilesDownloaded = 0;
    mRamTilesLoaded = 0;
    type=1;                     // Use ESRI

    // Generate status pixmaps
    for (int i = 0;i < 4;i++) {
        QPixmap pix(512, 512);
        QPainter *p = new QPainter(&pix);

        switch (i) {
        case 0: {
            // Downloading
            p->fillRect(pix.rect(), Qt::white);
            p->setBrush(QBrush(QColor(200, 255, 200)));
            p->setPen(QPen(QBrush(Qt::green), 3, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
            QRect r(3, 3, 506, 506);
            p->drawRect(r);
            QString txt = "Downloading\ntile...";
            p->setPen(QColor(Qt::black));
            QFont font;
            font.setPointSize(32);
            p->setFont(font);
            QRect rect;
            rect.setRect(0, 0, 512, 512);
            p->drawText(rect, Qt::AlignCenter, txt);
        } break;

        case 1: {
            // Waiting for other downloads.
            p->fillRect(pix.rect(), Qt::white);
            p->setBrush(QBrush(QColor(255, 255, 200)));
            p->setPen(QPen(QBrush(Qt::yellow), 3, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
            QRect r(3, 3, 506, 506);
            p->drawRect(r);
            QString txt = "Waiting for\nother downloads...";
            p->setPen(QColor(Qt::black));
            QFont font;
            font.setPointSize(32);
            p->setFont(font);
            QRect rect;
            rect.setRect(0, 0, 512, 512);
            p->drawText(rect, Qt::AlignCenter, txt);
        } break;

        case 2: {
            // Download error.
            p->fillRect(pix.rect(), Qt::white);
            p->setBrush(QBrush(QColor(255, 200, 200)));
            p->setPen(QPen(QBrush(Qt::red), 3, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
            QRect r(3, 3, 506, 506);
            p->drawRect(r);
            QString txt = "Download\nerror";
            p->setPen(QColor(Qt::black));
            QFont font;
            font.setPointSize(32);
            p->setFont(font);
            QRect rect;
            rect.setRect(0, 0, 512, 512);
            p->drawText(rect, Qt::AlignCenter, txt);
        } break;

        case 3: {
            // Not in map.
            p->fillRect(pix.rect(), Qt::white);
            p->setBrush(QBrush(QColor(255, 200, 200)));
            p->setPen(QPen(QBrush(Qt::red), 3, Qt::SolidLine, Qt::SquareCap, Qt::MiterJoin));
            QRect r(3, 3, 506, 506);
            p->drawRect(r);
            QString txt = "Tile outside\nof map";
            p->setPen(QColor(Qt::black));
            QFont font;
            font.setPointSize(32);
            p->setFont(font);
            QRect rect;
            rect.setRect(0, 0, 512, 512);
            p->drawText(rect, Qt::AlignCenter, txt);
        } break;

        }

        delete p;
        mStatusPixmaps.append(pix);
    }

    connect(&mWebCtrl, SIGNAL(finished(QNetworkReply*)),
            this, SLOT(fileDownloaded(QNetworkReply*)));
}

bool OsmClient::setCacheDir(QString path)
{
    QDir().mkpath(path);
    QFileInfo file;
    file.setFile(path);

    if (file.isDir()) {
        mCacheDir = path;
        return true;
    } else {
        qWarning() << "Invalid cache directory provided.";
        return false;
    }
}

bool OsmClient::setTileServerUrl(QString path)
{
    QUrl url(path);

    if (url.isValid()) {
        mTileServer = path;
        return true;
    } else {
        qWarning() << "Invalid tile server url provided:" << url.errorString();
        return false;
    }
}

/**
 * @brief OsmClient::getTile
 * Get a openstreetmap tile
 *
 * @param zoom
 * zoom level
 *
 * @param x
 * x index
 *
 * @param y
 * y index
 *
 * @param res
 * Reference to store the result in.
 *
 * Result greater than 0 means that a valid tile is returned. Negative results
 * are errors.
 *
 * -1: Tile not part of map.
 * 0: Tile not cached in memory or on disk.
 * 1: Tile read from memory.
 * 2: Tile read from disk.
 *
 * @return
 * The tile if res > 0, otherwise a tile with a status pixmap.
 */
OsmTile OsmClient::getTile(int zoom, int x, int y, int &res)
{
    res = 0;

    quint64 key = calcKey(zoom, x, y);
    OsmTile t = mMemoryTiles.value(key);

    if (x < 0 || y < 0 ||
            x >= (1 << zoom) ||
            y >= (1 << zoom)) {
        res = -1;
        t = OsmTile(mStatusPixmaps.at(3), zoom, x, y);
    } else if (!t.pixmap().isNull()) {
        res = 1;
        mRamTilesLoaded++;
    } else if (!mCacheDir.isEmpty()) {
        QString path;
        switch(type)
        {
        case 1: //ESRI
            path = mCacheDir + "/" + QString::number(zoom) + "/" +
                    QString::number(y) + "/" + QString::number(x) + ".jpeg";
            break;
        case 2: // OSM
            path = mCacheDir + "/" + QString::number(zoom) + "/" +
                 QString::number(x) + "/" + QString::number(y) + ".png";
            break;
        }

        QFile file;
        file.setFileName(path);

        if (file.exists()) {
            res = 2;
            t = OsmTile(QPixmap(path), zoom, x, y);
            storeTileMemory(key, t);
            mHddTilesLoaded++;
        } else {
            t = OsmTile(getStatusPixmap(key), zoom, x, y);
        }
    } else {
        t = OsmTile(getStatusPixmap(key), zoom, x, y);
    }

    return t;
}

/**
 * @brief OsmClient::downloadTile
 * Start a tile dowmload.
 *
 * @param zoom
 * zoom level
 *
 * @param x
 * x index
 *
 * @param y
 * y index
 *
 * @return
 * -3: Tile server not set.
 * -2: Too many tiles downloading.
 * -1: Unknown error.
 * 1: Tile download started.
 *
 */

void OsmClient::setType(int typearg)
{
    type=typearg;
}
int OsmClient::downloadTile(int zoom, int x, int y)
{
    int retval = -1;
    qDebug() << "TRY TO DOWNLOAD";

    if (!mTileServer.isEmpty()) {
        if (mDownloadingTiles.size() < mMaxDownloadingTiles) {
            quint64 key = calcKey(zoom, x, y);
            qDebug() << "(in downloadTile) x:" << x << ", y: " << y << ", zoom: " << zoom << "key :" << key;

            if (!mDownloadingTiles.contains(key)) {
                // Only add if this tile is not already downloading
                QString path;
                switch(type)
                {
                    case 1:
                    path = mTileServer + QString::number(zoom) + "/" + QString::number(y) + "/" + QString::number(x);
                    break;
                case 2:
                    path = mTileServer + "/" + QString::number(zoom) + "/" + QString::number(x) + "/" + QString::number(y) + ".png";
                    break;
                }
                qDebug() << "path: " << path;
                QNetworkRequest request(path);
                request.setRawHeader("User-Agent", "Firefox");
                mWebCtrl.get(request);
                mDownloadingTiles.insert(key, true);
            }
            qDebug() << "I am confused";
            retval = 1;
        } else {
            emit errorGetTile("Too many tiles downloading.");
            retval = -2;
        }
    } else {
        emit errorGetTile("Tile server not set.");
        retval = -3;
    }

    return retval;
}

bool OsmClient::downloadQueueFull()
{
    return mDownloadingTiles.size() >= mMaxDownloadingTiles;
}

void OsmClient::clearCache()
{
    QDir dir(mCacheDir);
    dir.removeRecursively();
    mMemoryTiles.clear();
    mMemoryTilesOrder.clear();
}

void OsmClient::fileDownloaded(QNetworkReply *pReply)
{
    QString path = pReply->url().toString();
    if (type==2)
    {
        path = path.left(path.length() - 4);   // Not used if ESRI, used if OpenStreetMap
    }
    int ind = path.lastIndexOf("/");
    int item1 = path.mid(ind + 1).toInt();
    path = path.left(ind);
    ind = path.lastIndexOf("/");
    int item2 = path.mid(ind + 1).toInt();
    path = path.left(ind);
    ind = path.lastIndexOf("/");
    int zoom = path.mid(ind + 1).toInt();

    int x,y;
    switch (type)
    {
    case 1:
        //If ESRI
        x=item1;
        y=item2;
        break;
    case 2:
        // If OpenStreetmap
        x=item2;
        y=item1;
        break;
    }

    quint64 key = calcKey(zoom, x, y);
    mDownloadingTiles.remove(key);

    if (pReply->error() == QNetworkReply::NoError) {
        QPixmap pm;
        QByteArray data = pReply->readAll();
        switch(type)
        {
        // IF ESRI
            case 1:
                pm.loadFromData(data, "JPG");    // If ESRI
                break;
            case 2:
                pm.loadFromData(data, "PNG");  // If OpenStreetMap
                break;
        }

        // Try to cache tile
        if (!mCacheDir.isEmpty()) {
            QString path;
            switch(type)
                 {
            // IF ESRI
            case 1:
                path = mCacheDir + "/" + QString::number(zoom) + "/" +
                        QString::number(y) + "/" + QString::number(x) + ".jpeg";
                qDebug() << path;
                break;
                // If OpenStreetMap
            case 2:
                path = mCacheDir + "/" + QString::number(zoom) + "/" +
                        QString::number(x) + "/" + QString::number(y) + ".png";
                break;
            }
              qDebug() << path;
/*            QMessageBox msg;
            msg.setText(path);
            msg.exec();
*/            QFile file;
            file.setFileName(path);
            if (!file.exists()) {
                switch(type)
                 {
                // IF ESRI
                case 1:
                    QDir().mkpath(mCacheDir + "/" + QString::number(zoom) + "/" + QString::number(y));
                    break;
                    // If OpenStreetMap
                case 2:
                QDir().mkpath(mCacheDir + "/" + QString::number(zoom) + "/" + QString::number(x));
                    break;
                }
                if (file.open(QIODevice::ReadWrite)) {
                    file.write(data);
                    file.close();
                } else {
                    emit errorGetTile("Cache error: " + file.errorString());
                }
            }
        }
        mTilesDownloaded++;
        mDownloadErrorTiles.remove(key);
        emitTile(OsmTile(pm, zoom, x, y));
    } else {
        mDownloadErrorTiles.insert(key, true);
        emit errorGetTile("Download error (fileDownloaded): " + pReply->errorString());
    }
}

int OsmClient::getRamTilesLoaded() const
{
    return mRamTilesLoaded;
}

int OsmClient::getTilesDownloaded() const
{
    return mTilesDownloaded;
}

int OsmClient::getMemoryTilesNow() const
{
    return mMemoryTiles.size();
}

int OsmClient::getHddTilesLoaded() const
{
    return mHddTilesLoaded;
}

int OsmClient::getMaxDownloadingTiles() const
{
    return mMaxDownloadingTiles;
}

void OsmClient::setMaxDownloadingTiles(int maxDownloadingTiles)
{
    mMaxDownloadingTiles = maxDownloadingTiles;
}

int OsmClient::getMaxMemoryTiles() const
{
    return mMaxMemoryTiles;
}

void OsmClient::setMaxMemoryTiles(int maxMemoryTiles)
{
    mMaxMemoryTiles = maxMemoryTiles;
}

void OsmClient::emitTile(OsmTile tile)
{
    quint64 key = calcKey(tile.zoom(), tile.x(), tile.y());
    if (!mMemoryTiles.contains(key)) {
        storeTileMemory(key, tile);
    }

    emit tileReady(tile);
}

quint64 OsmClient::calcKey(int zoom, int x, int y)
{
    return (quint64)0 | ((quint64)zoom << 50) | ((quint64)x << 25) | (quint64)y;
}

void OsmClient::storeTileMemory(quint64 key, const OsmTile &tile)
{
    mMemoryTiles.insert(key, tile);
    mMemoryTilesOrder.append(key);

    // Remove old tiles from memory if too much memory is used.
    while (mMemoryTilesOrder.size() > mMaxMemoryTiles) {
        quint64 k = mMemoryTilesOrder.takeFirst();
        mMemoryTiles.remove(k);
    }
}

const QPixmap &OsmClient::getStatusPixmap(quint64 key)
{
    if (mDownloadingTiles.contains(key)) {
        return mStatusPixmaps.at(0);
    } else if (mDownloadErrorTiles.contains(key)) {
        return mStatusPixmaps.at(2);
    } else {
        return mStatusPixmaps.at(1);
    }
}
