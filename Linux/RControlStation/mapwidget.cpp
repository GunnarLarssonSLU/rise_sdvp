/*
    Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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

#include <QDebug>
#include <math.h>
#include <qmath.h>
#include <QPrinter>
#include <QPrintEngine>
#include <QTime>

#include "mapwidget.h"
#include "qmessagebox.h"
#include "utility.h"
#include "attributes_masks.h"
#include "routemagic.h"

namespace
{
static void normalizeAngleRad(double &angle)
{
    angle = fmod(angle, 2.0 * M_PI);

    if (angle < 0.0) {
        angle += 2.0 * M_PI;
    }
}

void minMaxEps(double x, double y, double &min, double &max) {
    double eps;

    if (fabs(x) > fabs(y)) {
        eps = fabs(x) / 1e10;
    } else {
        eps = fabs(y) / 1e10;
    }

    if (x > y) {
        max = x + eps;
        min = y - eps;
    } else {
        max = y + eps;
        min = x - eps;
    }
}

bool isPointWithinRect(const QPointF &p, double xStart, double xEnd, double yStart, double yEnd) {
    return p.x() >= xStart && p.x() <= xEnd && p.y() >= yStart && p.y() <= yEnd;
}

// See https://www.topcoder.com/community/data-science/data-science-tutorials/geometry-concepts-line-intersection-and-its-applications/
bool lineSegmentIntersection(const QPointF &p1, const QPointF &p2, const QPointF &q1, const QPointF &q2) {
    bool res = false;

    const double A1 = p2.y() - p1.y();
    const double B1 = p1.x() - p2.x();
    const double C1 = A1 * p1.x() + B1 * p1.y();

    const double A2 = q2.y() - q1.y();
    const double B2 = q1.x() - q2.x();
    const double C2 = A2 * q1.x() + B2 * q1.y();

    const double det = A1 * B2 - A2 * B1;

    if(fabs(det) < 1e-6) {
        //Lines are parallel
    } else {
        double x = (B2 * C1 - B1 * C2) / det;
        double y = (A1 * C2 - A2 * C1) / det;

        // Check if this point is on both line segments.
        double p1XMin, p1XMax, p1YMin, p1YMax;
        minMaxEps(p1.x(), p2.x(), p1XMin, p1XMax);
        minMaxEps(p1.y(), p2.y(), p1YMin, p1YMax);

        double q1XMin, q1XMax, q1YMin, q1YMax;
        minMaxEps(q1.x(), q2.x(), q1XMin, q1XMax);
        minMaxEps(q1.y(), q2.y(), q1YMin, q1YMax);

        if (    x <= p1XMax && x >= p1XMin &&
                y <= p1YMax && y >= p1YMin &&
                x <= q1XMax && x >= q1XMin &&
                y <= q1YMax && y >= q1YMin) {
            res = true;
        }
    }

    return res;
}

bool isLineSegmentWithinRect(const QPointF &p1, const QPointF &p2, double xStart, double xEnd, double yStart, double yEnd) {
    QPointF q1(xStart, yStart);
    QPointF q2(xEnd, yStart);

    bool res = lineSegmentIntersection(p1, p2, q1, q2);

    if (!res) {
        q1.setX(xStart);
        q1.setY(yStart);
        q2.setX(xStart);
        q2.setY(yEnd);
        res = lineSegmentIntersection(p1, p2, q1, q2);
    }

    if (!res) {
        q1.setX(xStart);
        q1.setY(yEnd);
        q2.setX(xEnd);
        q2.setY(yEnd);
        res = lineSegmentIntersection(p1, p2, q1, q2);
    }

    if (!res) {
        q1.setX(xEnd);
        q1.setY(yStart);
        q2.setX(xEnd);
        q2.setY(yEnd);
        res = lineSegmentIntersection(p1, p2, q1, q2);
    }

    return res;
}
}

MapWidget::MapWidget(QWidget *parent) : QWidget(parent)
{
    mScaleFactor = 0.1;
    mRotation = 0;
    mXOffset = 0;
    mYOffset = 0;
    mMouseLastX = 1000000;
    mMouseLastY = 1000000;
    mFollowCar = -1;
    mTraceCar = -1;
    mSelectedCar = -1;
    xRealPos = 0;
    yRealPos = 0;
    mRoutePointSpeed = 1.0;
    mRoutePointTime = 0.0;
    mRoutePointAttributes = 0;
    mAnchorId = 0;
    mAnchorHeight = 1.0;
    mAntialiasDrawings = false;
    mAntialiasOsm = true;
    mInfoTraceTextZoom = 0.5;
    mDrawGrid = true;
    mRoutePointSelected = -1;
    mAnchorSelected = -1;
    mTraceMinSpaceCar = 0.05;
    mTraceMinSpaceGps = 0.05;
    mInfoTraceNow = 0;
    mAnchorMode = false;
    mDrawRouteText = false;
    mDrawUwbTrace = false;
    mCameraImageWidth = 0.46;
    mCameraImageOpacity = 0.8;
    mInteractionMode = InteractionModeDefault;

    bUpdateable=true;

    mOsm = new OsmClient(this);
    mDrawOpenStreetmap = true;
    mOsmZoomLevel = 14;
    mOsmRes = 1.0;
    mOsmMaxZoomLevel = 18;
    mDrawOsmStats = false;

    focusBorder=true;
    mPaths=new MapRouteCollection();
    mFields=new MapRouteCollection();

    mPaths->clear();

    MapRoute maproute;
    mPaths->append(maproute);

    mTimer = new QTimer(this);
    mTimer->start(20);

    // Set this to the SP base station position for now
    // Ultuna
    mRefLat = 59.812;
    mRefLon = 17.658;
    mRefHeight = 38;

    // Hardcoded for now
    mOsm->setCacheDir("osm_tiles");
    mOsm->setTileServerUrl("https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/");
    //    mOsm->setTileServerUrl("http://tile.openstreetmap.org");

    connect(mOsm, SIGNAL(tileReady(OsmTile)), this, SLOT(tileReady(OsmTile)));
    connect(mOsm, SIGNAL(errorGetTile(QString)), this, SLOT(errorGetTile(QString)));
    connect(mTimer, SIGNAL(timeout()), this, SLOT(timerSlot()));

    setMouseTracking(true);

    // Pre-render some things for speed
    for (int i = 0;i < 5;i++) {
        QPixmap pix(24, 24);
        pix.fill(Qt::transparent);
        QPainter p(&pix);

        QPen pen;
        pen.setWidth(4);

        switch (i) {
        case 0: {
            // Circle
            pen.setColor(Qt::darkYellow);
            p.setBrush(Qt::yellow);
            p.setPen(pen);
            p.drawEllipse(2, 2, 20, 20);
        } break;

        case 1: {
            // Inactive circle
            pen.setColor(Qt::darkGray);
            p.setBrush(Qt::gray);
            p.setPen(pen);
            p.drawEllipse(2, 2, 20, 20);
        } break;

        case 2: {
            // UWB circle
            pen.setColor(Qt::darkGreen);
            p.setBrush(Qt::green);
            p.setPen(pen);
            p.drawEllipse(2, 2, 20, 20);
        } break;

        case 3: {
            // Hydraulic front down circle
            pen.setColor(Qt::darkCyan);
            p.setBrush(Qt::cyan);
            p.setPen(pen);
            p.drawEllipse(2, 2, 20, 20);
        } break;

        case 4: {
            // Hydraulic front up circle
            pen.setColor(Qt::darkYellow);
            p.setBrush(Qt::green);
            p.setPen(pen);
            p.drawEllipse(2, 2, 20, 20);
        } break;

        }

        mPixmaps.append(pix);
    }

    grabGesture(Qt::PinchGesture);
}

CarInfo *MapWidget::getCarInfo(int car)
{
    for (int i = 0;i < mCarInfo.size();i++) {
        if (mCarInfo[i].getId() == car) {
            return &mCarInfo[i];
        }
    }

    return 0;
}

void MapWidget::addCar(const CarInfo &car)
{
    mCarInfo.append(car);
    update();
}

bool MapWidget::removeCar(int carId)
{
    QMutableListIterator<CarInfo> i(mCarInfo);
    while (i.hasNext()) {
        if (i.next().getId() == carId) {
            i.remove();
            return true;
        }
    }

    return false;
}

void MapWidget::clearCars()
{
    mCarInfo.clear();
}

LocPoint *MapWidget::getAnchor(int id)
{
    for (int i = 0;i < mAnchors.size();i++) {
        if (mAnchors[i].getId() == id) {
            return &mAnchors[i];
        }
    }

    return 0;
}

void MapWidget::addAnchor(const LocPoint &anchor)
{
    mAnchors.append(anchor);
    update();
}

bool MapWidget::removeAnchor(int id)
{
    bool found = false;

    QMutableListIterator<LocPoint> i(mAnchors);
    while (i.hasNext()) {
        if (i.next().getId() == id) {
            i.remove();
            found = true;
        }
    }

    return found;
}

void MapWidget::clearAnchors()
{
    mAnchors.clear();
    update();
}

QList<LocPoint> MapWidget::getAnchors()
{
    return mAnchors;
}

void MapWidget::setScaleFactor(double scale)
{
    double scaleDiff = scale / mScaleFactor;
    mScaleFactor = scale;
    mXOffset *= scaleDiff;
    mYOffset *= scaleDiff;
    update();
}

double MapWidget::getScaleFactor()
{
    return mScaleFactor;
}

void MapWidget::setRotation(double rotation)
{
    mRotation = rotation;
    update();
}

void MapWidget::setXOffset(double offset)
{
    mXOffset = offset;
    update();
}

void MapWidget::setYOffset(double offset)
{
    mYOffset = offset;
    update();
}

void MapWidget::moveView(double px, double py)
{
    LocPoint followLoc;
    followLoc.setXY(px, py);
    mXOffset = -followLoc.getX() * 1000.0 * mScaleFactor;
    mYOffset = -followLoc.getY() * 1000.0 * mScaleFactor;
    update();
}

void MapWidget::clearTrace()
{
    mCarTrace.clear();
    mCarTraceGps.clear();
    mCarTraceUwb.clear();
    update();
}

void MapWidget::addRoutePoint(double px, double py, double speed, qint32 time)
{
    LocPoint pos;
    pos.setXY(px, py);
    pos.setSpeed(speed);
    pos.setTime(time);
    mPaths->getCurrent().append(pos);
    update();
}

MapRoute MapWidget::getPath(int ind)
{
    return mPaths->getRoute(ind);
}

MapRoute MapWidget::getField(int ind)
{
    return mFields->getRoute(ind);
}


QList<MapRoute> MapWidget::getPaths()
{
    return mPaths->getRoutes();
}

QList<MapRoute> MapWidget::getFields()
{
    return mFields->getRoutes();
}

void MapWidget::setPath(const MapRoute &path)
{
    mPaths->setRoute(path);
    update();
}

void MapWidget::setField(const MapRoute &field)
{
    mFields->setRoute(field);
    update();
}


void MapWidget::addPath(const MapRoute &path)
{
    mPaths->addRoute(path);
    update();
}

void MapWidget::addField(const MapRoute &field)
{
    mFields->addRoute(field);
    update();
}


int MapWidget::getPathNum()
{
    return mPaths->size();
}

int MapWidget::getFieldNum()
{
    return mFields->size();
}

void MapWidget::clearPath()
{
    mPaths->clearRoute();
    update();
}

void MapWidget::clearField()
{
    mFields->clearRoute();
    update();
}

void MapWidget::clearAllPaths()
{
    mPaths->clearAllRoutes();
    update();
}

void MapWidget::clearAllFields()
{
    mFields->clearAllRoutes();
    update();
}

int MapWidget::getPathNow() const
{
    return mPaths->mRouteNow;
}

void MapWidget::setPathNow(int pathNow)
{
    mPaths->setRouteNow(pathNow);
    update();
}

int MapWidget::getFieldNow() const
{
    return mFields->mRouteNow;
}

void MapWidget::setFieldNow(int fieldNow)
{
    mFields->setRouteNow(fieldNow);
    update();
}


void MapWidget::setBorderFocus(bool focus)
{
    focusBorder=focus;
}

void MapWidget::setRoutePointSpeed(double speed)
{
    mRoutePointSpeed = speed;
}

void MapWidget::addInfoPoint(LocPoint &info, bool updateMap)
{
    mInfoTraces[mInfoTraceNow].append(info);

    if (updateMap) {
        update();
    }
}

void MapWidget::clearInfoTrace()
{
    mInfoTraces[mInfoTraceNow].clear();
    update();
}

void MapWidget::clearAllInfoTraces()
{
    for (int i = 0;i < mInfoTraces.size();i++) {
        mInfoTraces[i].clear();
    }

    update();
}

void MapWidget::addPerspectivePixmap(PerspectivePixmap map)
{
    mPerspectivePixmaps.append(map);
}

void MapWidget::clearPerspectivePixmaps()
{
    mPerspectivePixmaps.clear();
    update();
}

QPoint MapWidget::getMousePosRelative()
{
    QPoint p = this->mapFromGlobal(QCursor::pos());
    p.setX((p.x() - mXOffset - width() / 2) / mScaleFactor);
    p.setY((-p.y() - mYOffset + height() / 2) / mScaleFactor);
    return p;
}

void MapWidget::setAntialiasDrawings(bool antialias)
{
    mAntialiasDrawings = antialias;
    update();
}

void MapWidget::setAntialiasOsm(bool antialias)
{
    mAntialiasOsm = antialias;
    update();
}

void MapWidget::tileReady(OsmTile tile)
{
    (void)tile;
    update();
}

void MapWidget::errorGetTile(QString reason)
{
    qWarning() << "OSM tile error:" << reason;
}

void MapWidget::timerSlot()
{
    updateTraces();
}

void MapWidget::setFollowCar(int car)
{
    int oldCar = mFollowCar;
    mFollowCar = car;

    if (oldCar != mFollowCar) {
        update();
    }
}

void MapWidget::setTraceCar(int car)
{
    mTraceCar = car;
}

void MapWidget::setSelectedCar(int car)
{
    int oldCar = mSelectedCar;
    mSelectedCar = car;

    if (oldCar != mSelectedCar) {
        update();
    }
}

void MapWidget::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    paint(painter, event->rect().width(), event->rect().height());
}

void MapWidget::mouseMoveEvent(QMouseEvent *e)
{
    bool ctrl = e->modifiers() == Qt::ControlModifier;
    bool shift = e->modifiers() == Qt::ShiftModifier;
    bool ctrl_shift = e->modifiers() == (Qt::ControlModifier | Qt::ShiftModifier);

    if (mInteractionMode == InteractionModeCtrlDown) {
        ctrl = true;
        shift = false;
        ctrl_shift = false;
    } else if (mInteractionMode == InteractionModeShiftDown) {
        ctrl = false;
        shift = true;
        ctrl_shift = false;
    } else if (mInteractionMode == InteractionModeCtrlShiftDown) {
        ctrl = false;
        shift = false;
        ctrl_shift = true;
    }

    QPoint mousePosWidget = this->mapFromGlobal(QCursor::pos());
    LocPoint mousePosMap;
    QPoint p = getMousePosRelative();
    mousePosMap.setXY(p.x() / 1000.0, p.y() / 1000.0);

    for (MapModule *m: mMapModules) {
        if (m->processMouse(false, false, true, false,
                            mousePosWidget, mousePosMap, 0.0,
                            ctrl, shift, ctrl_shift,
                            e->buttons() & Qt::LeftButton,
                            e->buttons() & Qt::RightButton,
                            mScaleFactor)) {
            return;
        }
    }

    if (e->buttons() & Qt::LeftButton && !ctrl && !shift && !ctrl_shift) {
        int x = e->pos().x();
        int y = e->pos().y();

        if (mMouseLastX < 100000)
        {
            int diffx = x - mMouseLastX;
            mXOffset += diffx;
            update();
        }

        if (mMouseLastY < 100000)
        {
            int diffy = y - mMouseLastY;
            mYOffset -= diffy;

            emit offsetChanged(mXOffset, mYOffset);
            update();
        }

        mMouseLastX = x;
        mMouseLastY = y;
    }

    if (mRoutePointSelected >= 0) {
 //       MapRoute *current=mPaths->getCurrent();
 //       *current[mRoutePointSelected].setXY(mousePosMap.getX(), mousePosMap.getY());
        mPaths->getCurrent()[mRoutePointSelected].setXY(mousePosMap.getX(), mousePosMap.getY());
        update();
    }

    if (mAnchorSelected >= 0) {
        mAnchors[mAnchorSelected].setXY(mousePosMap.getX(), mousePosMap.getY());
        update();
    }

    updateClosestInfoPoint();
}

void MapWidget::mousePressEvent(QMouseEvent *e)
{
    setFocus();

    bool ctrl = e->modifiers() == Qt::ControlModifier;
    bool shift = e->modifiers() == Qt::ShiftModifier;
    bool ctrl_shift = e->modifiers() == (Qt::ControlModifier | Qt::ShiftModifier);

    if (mInteractionMode == InteractionModeCtrlDown) {
        ctrl = true;
        shift = false;
        ctrl_shift = false;
    } else if (mInteractionMode == InteractionModeShiftDown) {
        ctrl = false;
        shift = true;
        ctrl_shift = false;
    } else if (mInteractionMode == InteractionModeCtrlShiftDown) {
        ctrl = false;
        shift = false;
        ctrl_shift = true;
    }

    QPoint mousePosWidget = this->mapFromGlobal(QCursor::pos());
    LocPoint mousePosMap;
    QPoint p = getMousePosRelative();
    double px=p.x()/1000.0;
    double py=p.y()/1000.0;
    mousePosMap.setXY(px, py);

    qDebug() << "Mouse pressed! Checking fields!";

    for (int i=0;i<getFieldNum();i++)
    {
        if (RouteMagic::isPointWithin(px, py, getField(i).mRoute))
        {
            qDebug() << "Clicked:" << i;
            setFieldNow(i);
            emit mouseClickedInField(i+1);
        };
    };

    for (MapModule *m: mMapModules) {
        if (m->processMouse(true, false, false, false,
                            mousePosWidget, mousePosMap, 0.0,
                            ctrl, shift, ctrl_shift,
                            e->buttons() & Qt::LeftButton,
                            e->buttons() & Qt::RightButton,
                            mScaleFactor)) {
            return;
        }
    }

    mousePosMap.setSpeed(mRoutePointSpeed);
    mousePosMap.setTime(mRoutePointTime);
    mousePosMap.setAttributes(mRoutePointAttributes);
    mousePosMap.setId(mAnchorId);
    mousePosMap.setHeight(mAnchorHeight);


    double routeDist = 0.0;
    double anchorDist = 0.0;

    int anchorInd = getClosestPoint(mousePosMap, mAnchors, anchorDist);
    bool routeFound = (routeDist * mScaleFactor * 1000.0) < 20 && routeDist >= 0.0;
    bool anchorFound = (anchorDist * mScaleFactor * 1000.0) < 20 && anchorDist >= 0.0;


    MapRoute *currentRoute;
    int mRoutePointSelected;
    bool bHasCurrentRoute=false;
    if (focusBorder)
    {
        if (mFields->size()>0)
        {
            qDebug() << "Acting on border";
            currentRoute=&(mFields->getCurrent());
            qDebug() << "Border size: " << currentRoute->size();
            bHasCurrentRoute=true;
        }
    } else
    {
        if (mPaths->size()>0)
        {
            qDebug() << "Acting on path";
            currentRoute=&(mPaths->getCurrent());
            bHasCurrentRoute=true;
        }
    }
    if (bHasCurrentRoute) mRoutePointSelected = getClosestPoint(mousePosMap, currentRoute->mRoute, routeDist);

    if (ctrl) {
        if (e->buttons() & Qt::LeftButton) {
            if (mSelectedCar >= 0) {
                for (int i = 0;i < mCarInfo.size();i++) {
                    CarInfo &carInfo = mCarInfo[i];
                    if (carInfo.getId() == mSelectedCar) {
                        LocPoint pos = carInfo.getLocation();
                        QPoint p = getMousePosRelative();
                        pos.setXY(p.x() / 1000.0, p.y() / 1000.0);
                        carInfo.setLocation(pos);
                        emit posSet(mSelectedCar, pos);
                    }
                }
            }
        } else if (e->buttons() & Qt::RightButton) {
            if (mAnchorMode) {
                if (anchorFound) {
                    mAnchors[anchorInd].setId(mAnchorId);
                    mAnchors[anchorInd].setHeight(mAnchorHeight);
                }
            } else {
                if (routeFound && (!focusBorder) && bHasCurrentRoute) {
                    (*currentRoute)[mRoutePointSelected].setSpeed(mRoutePointSpeed);
                    (*currentRoute)[mRoutePointSelected].setTime(mRoutePointTime);
                    (*currentRoute)[mRoutePointSelected].setAttributes(mRoutePointAttributes);
                }
            }
        }
        update();
    } else if (shift) {
        qDebug() << "Shift clicked!";
        if (mAnchorMode) {
            /*
            if (e->buttons() & Qt::LeftButton) {
                if (anchorFound) {
                    mAnchorSelected = anchorInd;
                    mAnchors[anchorInd].setXY(mousePosMap.getX(), mousePosMap.getY());
                } else {
                    mAnchors.append(mousePosMap);
                }
            } else if (e->buttons() & Qt::RightButton) {
                if (anchorFound) {
                    mAnchors.removeAt(anchorInd);
                }
            }*/
        } else {
            qDebug() << "Point: " << mRoutePointSelected;
            if (e->buttons() & Qt::LeftButton) {
                qDebug() << "Pressed left key";
                if (routeFound) {
                    qDebug() << "Found route";
                    qDebug() << "Size: " << currentRoute->size();
                    qDebug() << "Pos: " << mRoutePointSelected;
                    (*currentRoute)[mRoutePointSelected].setXY(mousePosMap.getX(), mousePosMap.getY());
                    qDebug() << "Set position";
                } else {
                    qDebug() << "Did not find route";
                    if (currentRoute->size() < 2 ||
                        currentRoute->last().getDistanceTo(mousePosMap) <
                                                           currentRoute->first().getDistanceTo(mousePosMap)) {
                    qDebug() << "Appending";
                    currentRoute->append(mousePosMap);
                        emit routePointAdded(mousePosMap);
                    } else {
                        qDebug() << "Prepending";
                        currentRoute->prepend(mousePosMap);
                    }
                }
            } else if (e->buttons() & Qt::RightButton) {
                qDebug() << "Pressed right key";
                if (routeFound) {
                    currentRoute->removeAt(mRoutePointSelected );
                } else {
                    removeLastRoutePoint();
                }
            }
        }
        qDebug() << "Updating";
        update();
    } else if (ctrl_shift) {
        if (e->buttons() & Qt::LeftButton) {
            QPoint p = getMousePosRelative();
            double iLlh[3], llh[3], xyz[3];
            iLlh[0] = mRefLat;
            iLlh[1] = mRefLon;
            iLlh[2] = mRefHeight;
            xyz[0] = p.x() / 1000.0;
            xyz[1] = p.y() / 1000.0;
            xyz[2] = 0.0;
            utility::enuToLlh(iLlh, xyz, llh);
            mRefLat = llh[0];
            mRefLon = llh[1];
            mRefHeight = 0.0;
        }

        update();
    }
}

void MapWidget::mouseReleaseEvent(QMouseEvent *e)
{
    bool ctrl = e->modifiers() == Qt::ControlModifier;
    bool shift = e->modifiers() == Qt::ShiftModifier;
    bool ctrl_shift = e->modifiers() == (Qt::ControlModifier | Qt::ShiftModifier);

    if (mInteractionMode == InteractionModeCtrlDown) {
        ctrl = true;
        shift = false;
        ctrl_shift = false;
    } else if (mInteractionMode == InteractionModeShiftDown) {
        ctrl = false;
        shift = true;
        ctrl_shift = false;
    } else if (mInteractionMode == InteractionModeCtrlShiftDown) {
        ctrl = false;
        shift = false;
        ctrl_shift = true;
    }

    QPoint mousePosWidget = this->mapFromGlobal(QCursor::pos());
    LocPoint mousePosMap;
    QPoint p = getMousePosRelative();
    mousePosMap.setXY(p.x() / 1000.0, p.y() / 1000.0);

    for (MapModule *m: mMapModules) {
        if (m->processMouse(false, true, false, false,
                            mousePosWidget, mousePosMap, 0.0,
                            ctrl, shift, ctrl_shift,
                            e->buttons() & Qt::LeftButton,
                            e->buttons() & Qt::RightButton,
                            mScaleFactor)) {
            return;
        }
    }

    if (!(e->buttons() & Qt::LeftButton)) {
        mMouseLastX = 1000000;
        mMouseLastY = 1000000;
        mRoutePointSelected = -1;
        mAnchorSelected = -1;
    }
}

void MapWidget::wheelEvent(QWheelEvent *e)
{
    bool ctrl = e->modifiers() == Qt::ControlModifier;
    bool shift = e->modifiers() == Qt::ShiftModifier;
    bool ctrl_shift = e->modifiers() == (Qt::ControlModifier | Qt::ShiftModifier);

    if (mInteractionMode == InteractionModeCtrlDown) {
        ctrl = true;
        shift = false;
        ctrl_shift = false;
    } else if (mInteractionMode == InteractionModeShiftDown) {
        ctrl = false;
        shift = true;
        ctrl_shift = false;
    } else if (mInteractionMode == InteractionModeCtrlShiftDown) {
        ctrl = false;
        shift = false;
        ctrl_shift = true;
    }

    QPoint mousePosWidget = this->mapFromGlobal(QCursor::pos());
    LocPoint mousePosMap;
    QPoint p = getMousePosRelative();
    mousePosMap.setXY(p.x() / 1000.0, p.y() / 1000.0);

    for (MapModule *m: mMapModules) {
        if (m->processMouse(false, false, false, true,
                            mousePosWidget, mousePosMap, e->angleDelta().y(),
                            ctrl, shift, ctrl_shift,
                            e->buttons() & Qt::LeftButton,
                            e->buttons() & Qt::RightButton,
                            mScaleFactor)) {
            return;
        }
    }

    if (mInteractionMode == InteractionModeCtrlDown) {
        ctrl = true;
    }

    if (ctrl && mSelectedCar >= 0) {
        for (int i = 0;i < mCarInfo.size();i++) {
            CarInfo &carInfo = mCarInfo[i];
            if (carInfo.getId() == mSelectedCar) {
                LocPoint pos = carInfo.getLocation();
//                double angle = pos.getYaw() + (double)e->angleDelta().y() * 0.0005;
                double angle = pos.getYaw() + (double)e->angleDelta().y() * 0.05;
                normalizeAngleRad(angle);
                pos.setYaw(angle);
                carInfo.setLocation(pos);
                emit posSet(mSelectedCar, pos);
                update();
            }
        }
    } else {
        double scaleDiff = ((double)e->angleDelta().y() / 600.0);
        if (scaleDiff > 0.8)
        {
            scaleDiff = 0.8;
        }

        if (scaleDiff < -0.8)
        {
            scaleDiff = -0.8;
        }

        mScaleFactor += mScaleFactor * scaleDiff;
        mXOffset += mXOffset * scaleDiff;
        mYOffset += mYOffset * scaleDiff;

        emit scaleChanged(mScaleFactor);
        emit offsetChanged(mXOffset, mYOffset);
        update();
    }

    updateClosestInfoPoint();
}

bool MapWidget::event(QEvent *event)
{
    if (event->type() == QEvent::Gesture) {
        QGestureEvent *ge = static_cast<QGestureEvent*>(event);

        if (QGesture *pinch = ge->gesture(Qt::PinchGesture)) {
            QPinchGesture *pg = static_cast<QPinchGesture *>(pinch);

            if (pg->changeFlags() & QPinchGesture::ScaleFactorChanged) {
                mScaleFactor *= pg->scaleFactor();
                mXOffset *= pg->scaleFactor();
                mYOffset *= pg->scaleFactor();
                update();
            }

            return true;
        }
    } else if (event->type() == QEvent::KeyPress) {
        // Generate scroll events from up and down arrow keys
        QKeyEvent *ke = static_cast<QKeyEvent*>(event);
        if (ke->key() == Qt::Key_Up) {
            QWheelEvent we(QPointF(0, 0),
                           QPointF(0, 0),
                           QPoint(0, 0),
                           QPoint(0, 120),
                           0, Qt::Vertical, 0,
                           ke->modifiers());
            wheelEvent(&we);
            return true;
        } else if (ke->key() == Qt::Key_Down) {
            QWheelEvent we(QPointF(0, 0),
                           QPointF(0, 0),
                           QPoint(0, 0),
                           QPoint(0, -120),
                           0, Qt::Vertical, 0,
                           ke->modifiers());
            wheelEvent(&we);
            return true;
        }
    }

    return QWidget::event(event);
}

quint32 MapWidget::getRoutePointAttributes() const
{
    return mRoutePointAttributes;
}

void MapWidget::setRoutePointAttributes(const quint32 &routePointAttributes)
{
    mRoutePointAttributes = routePointAttributes;
}

MapWidget::InteractionMode MapWidget::getInteractionMode() const
{
    return mInteractionMode;
}

void MapWidget::setInteractionMode(const MapWidget::InteractionMode &controlMode)
{
    mInteractionMode = controlMode;
    update();
}

void MapWidget::addMapModule(MapModule *m)
{
    mMapModules.append(m);
}

void MapWidget::removeMapModule(MapModule *m)
{
    for (int i = 0;i < mMapModules.size();i++) {
        if (mMapModules.at(i) == m) {
            mMapModules.remove(i);
            break;
        }
    }
}

void MapWidget::removeMapModuleLast()
{
    if (!mMapModules.isEmpty()) {
        mMapModules.removeLast();
    }
}

double MapWidget::getCameraImageOpacity() const
{
    return mCameraImageOpacity;
}

void MapWidget::setCameraImageOpacity(double cameraImageOpacity)
{
    mCameraImageOpacity = cameraImageOpacity;
    update();
}

double MapWidget::getCameraImageWidth() const
{
    return mCameraImageWidth;
}

void MapWidget::setCameraImageWidth(double cameraImageWidth)
{
    mCameraImageWidth = cameraImageWidth;
    update();
}

void MapWidget::setLastCameraImage(const QImage &lastCameraImage)
{
    mLastCameraImage = lastCameraImage;

    if (mCameraImageWidth > 0.0001) {
        update();
    }
}

bool MapWidget::getDrawRouteText() const
{
    return mDrawRouteText;
}

void MapWidget::setDrawRouteText(bool drawRouteText)
{
    mDrawRouteText = drawRouteText;
    update();
}

bool MapWidget::getDrawUwbTrace() const
{
    return mDrawUwbTrace;
}

void MapWidget::setDrawUwbTrace(bool drawUwbTrace)
{
    mDrawUwbTrace = drawUwbTrace;
    update();
}

double MapWidget::getTraceMinSpaceGps() const
{
    return mTraceMinSpaceGps;
}

void MapWidget::setTraceMinSpaceGps(double traceMinSpaceGps)
{
    mTraceMinSpaceGps = traceMinSpaceGps;
}

double MapWidget::getTraceMinSpaceCar() const
{
    return mTraceMinSpaceCar;
}

void MapWidget::setTraceMinSpaceCar(double traceMinSpaceCar)
{
    mTraceMinSpaceCar = traceMinSpaceCar;
}

qint32 MapWidget::getRoutePointTime() const
{
    return mRoutePointTime;
}

void MapWidget::setRoutePointTime(const qint32 &routePointTime)
{
    mRoutePointTime = routePointTime;
}

int MapWidget::getInfoTraceNow() const
{
    return mInfoTraceNow;
}

void MapWidget::setInfoTraceNow(int infoTraceNow)
{
    int infoTraceOld = mInfoTraceNow;
    mInfoTraceNow = infoTraceNow;

    while (mInfoTraces.size() < (mInfoTraceNow + 1)) {
        QList<LocPoint> l;
        mInfoTraces.append(l);
    }
    update();

    if (infoTraceOld != mInfoTraceNow) {
        emit infoTraceChanged(mInfoTraceNow);
    }
}

bool MapWidget::getDrawOsmStats() const
{
    return mDrawOsmStats;
}

void MapWidget::setDrawOsmStats(bool drawOsmStats)
{
    mDrawOsmStats = drawOsmStats;
    update();
}

bool MapWidget::getDrawGrid() const
{
    return mDrawGrid;
}

void MapWidget::setDrawGrid(bool drawGrid)
{
    bool drawGridOld = mDrawGrid;
    mDrawGrid = drawGrid;

    if (drawGridOld != mDrawGrid) {
        update();
    }
}

void MapWidget::updateClosestInfoPoint()
{
    QPointF mpq = getMousePosRelative();
    LocPoint mp(mpq.x() / 1000.0, mpq.y() / 1000.0);
    double dist_min = 1e30;
    LocPoint closest;

    for (int i = 0;i < mVisibleInfoTracePoints.size();i++) {
        const LocPoint &ip = mVisibleInfoTracePoints[i];
        if (mp.getDistanceTo(ip) < dist_min) {
            dist_min = mp.getDistanceTo(ip);
            closest = ip;
        }
    }

    bool drawBefore = mClosestInfo.getInfo().size() > 0;
    bool drawNow = false;
    if ((dist_min * mScaleFactor) < 0.02) {
        if (closest != mClosestInfo) {
            mClosestInfo = closest;
            update();
        }

        drawNow = true;
    } else {
        mClosestInfo.setInfo("");
    }

    if (drawBefore && !drawNow) {
        update();
    }
}

int MapWidget::drawInfoPoints(QPainter &painter, const QList<LocPoint> &pts,
                              QTransform drawTrans, QTransform txtTrans,
                              double xStart, double xEnd, double yStart, double yEnd,
                              double min_dist)
{
    int last_visible = 0;
    int drawn = 0;
    QPointF pt_txt;
    QRectF rect_txt;

    painter.setTransform(txtTrans);

    for (int i = 0;i < pts.size();i++) {
        const LocPoint &ip = pts[i];
        QPointF p = ip.getPointMm();
        QPointF p2 = drawTrans.map(p);

        if (isPointWithinRect(p, xStart, xEnd, yStart, yEnd)) {
            if (drawn > 0) {
                double dist_view = pts.at(i).getDistanceTo(pts.at(last_visible)) * mScaleFactor;
                if (dist_view < min_dist) {
                    continue;
                }

                last_visible = i;
            }

            painter.setBrush(ip.getColor());
            painter.setPen(ip.getColor());
            painter.drawEllipse(p2, ip.getRadius(), ip.getRadius());

            drawn++;
            mVisibleInfoTracePoints.append(ip);

            if (mScaleFactor > mInfoTraceTextZoom) {
                pt_txt.setX(p.x() + 5 / mScaleFactor);
                pt_txt.setY(p.y());
                pt_txt = drawTrans.map(pt_txt);
                painter.setPen(Qt::black);
                painter.setFont(QFont("monospace"));
                rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 20,
                                   pt_txt.x() + 500, pt_txt.y() + 500);
                painter.drawText(rect_txt, Qt::AlignTop | Qt::AlignLeft, ip.getInfo());
            }
        }
    }

    return drawn;
}

int MapWidget::getClosestPoint(LocPoint p, QList<LocPoint> route, double &dist)
{
    QList<LocPoint> points=route;
    int closest = -1;
    dist = -1.0;
    for (int i = 0;i < points.size();i++) {
        double d = points[i].getDistanceTo(p);
        if (dist < 0.0 || d < dist) {
            dist = d;
            closest = i;
        }
    }

    return closest;
}

void MapWidget::drawCircleFast(QPainter &painter, QPointF center, double radius, int type)
{
    painter.drawPixmap(center.x() - radius, center.y() - radius,
                       2.0 * radius, 2.0 * radius, mPixmaps.at(type));
}

int MapWidget::getOsmZoomLevel() const
{
    return mOsmZoomLevel;
}

int MapWidget::getOsmMaxZoomLevel() const
{
    return mOsmMaxZoomLevel;
}

void MapWidget::setOsmMaxZoomLevel(int osmMaxZoomLevel)
{
    mOsmMaxZoomLevel = osmMaxZoomLevel;
    update();
}

double MapWidget::getInfoTraceTextZoom() const
{
    return mInfoTraceTextZoom;
}

void MapWidget::setInfoTraceTextZoom(double infoTraceTextZoom)
{
    mInfoTraceTextZoom = infoTraceTextZoom;
    update();
}

OsmClient *MapWidget::osmClient()
{
    return mOsm;
}

int MapWidget::getInfoTraceNum()
{
    return mInfoTraces.size();
}

int MapWidget::getInfoPointsInTrace(int trace)
{
    int res = -1;

    if (trace >= 0 && trace < mInfoTraces.size()) {
        res = mInfoTraces.at(trace).size();
    }

    return res;
}

int MapWidget::setNextEmptyOrCreateNewInfoTrace()
{
    int next = -1;

    for (int i = 0;i < mInfoTraces.size();i++) {
        if (mInfoTraces.at(i).isEmpty()) {
            next = i;
            break;
        }
    }

    if (next < 0) {
        next = mInfoTraces.size();
    }

    setInfoTraceNow(next);

    return next;
}

void MapWidget::setAnchorMode(bool anchorMode)
{
    mAnchorMode = anchorMode;
}

bool MapWidget::getAnchorMode()
{
    return mAnchorMode;
}

void MapWidget::setAnchorId(int id)
{
    mAnchorId = id;
}

void MapWidget::setAnchorHeight(double height)
{
    mAnchorHeight = height;
}

void MapWidget::removeLastRoutePoint()
{
    LocPoint pos;
    MapRoute &current=mPaths->getCurrent();
    if (current.size() > 0) {
        pos = current.last();
        current.removeLast();
    }
    emit lastRoutePointRemoved(pos);
    update();
}

void MapWidget::zoomInOnRoute(int id, double margins, double wWidth, double wHeight)
{
    MapRoute route;

    if (id >= 0) {
        route = getPath(id);
    } else {
        for (auto r: mPaths->mCollection) {
            route.append(r);
        }
    }

    if (route.size() > 0) {
        double xMin = 1e12;
        double xMax = -1e12;
        double yMin = 1e12;
        double yMax = -1e12;

        for (auto p: route) {
            if (p.getX() < xMin) {
                xMin = p.getX();
            }
            if (p.getX() > xMax) {
                xMax = p.getX();
            }
            if (p.getY() < yMin) {
                yMin = p.getY();
            }
            if (p.getY() > yMax) {
                yMax = p.getY();
            }
        }

        double width = xMax - xMin;
        double height = yMax - yMin;

        if (wWidth <= 0 || wHeight <= 0) {
            wWidth = this->width();
            wHeight = this->height();
        }

        xMax += width * margins * 0.5;
        xMin -= width * margins * 0.5;
        yMax += height * margins * 0.5;
        yMin -= height * margins * 0.5;

        width = xMax - xMin;
        height = yMax - yMin;

        double scaleX = 1.0 / ((width * 1000) / wWidth);
        double scaleY = 1.0 / ((height * 1000) / wHeight);

        mScaleFactor = qMin(scaleX, scaleY);
        mXOffset = -(xMin + width / 2.0) * mScaleFactor * 1000.0;
        mYOffset = -(yMin + height / 2.0) * mScaleFactor * 1000.0;

        update();
    }
}

double MapWidget::getOsmRes() const
{
    return mOsmRes;
}

void MapWidget::setOsmRes(double osmRes)
{
    mOsmRes = osmRes;
    update();
}

bool MapWidget::getDrawOpenStreetmap() const
{
    return mDrawOpenStreetmap;
}

void MapWidget::setDrawOpenStreetmap(bool drawOpenStreetmap)
{
    mDrawOpenStreetmap = drawOpenStreetmap;
    update();
}

void MapWidget::setEnuRef(double lat, double lon, double height)
{
    mRefLat = lat;
    mRefLon = lon;
    mRefHeight = height;
    update();
}

void MapWidget::getEnuRef(double *llh)
{
    llh[0] = mRefLat;
    llh[1] = mRefLon;
    llh[2] = mRefHeight;
}

void MapWidget::setTrace(QVector<LocPoint> mTrace)
{
    mCarTrace=mTrace;
    setLogStart(0);
    setLogEnd(99);
    bUpdateable=false;
}

void MapWidget::setLogStart(int iStart)
{
    iLogstart=(int)(iStart*mCarTrace.size()/99);
    qDebug() << "istart: " << iStart;
    update();
};

void MapWidget::setLogEnd(int iEnd)
{
    iLogend=(int)(mCarTrace.size()*iEnd/99);
    qDebug() << "iend: " << iEnd;
    update();
};

int MapWidget::Elements()
{
    return iLogend-iLogstart+1;
}

int MapWidget::firstElement()
{
    return iLogstart;
}

int MapWidget::lastElement()
{
    return iLogend-1;
}


void MapWidget::paint(QPainter &painter, int width, int height, bool highQuality)
{
//    qDebug() << "Start painting";
    if (highQuality) {
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.setRenderHint(QPainter::TextAntialiasing, true);
        painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    } else {
        painter.setRenderHint(QPainter::Antialiasing, mAntialiasDrawings);
        painter.setRenderHint(QPainter::TextAntialiasing, mAntialiasDrawings);
        painter.setRenderHint(QPainter::SmoothPixmapTransform, mAntialiasDrawings);
    }

    const double scaleMax = 20;
    const double scaleMin = 0.000001;

    // Make sure scale and offsetappend is reasonable
    if (mScaleFactor < scaleMin) {
        double scaleDiff = scaleMin / mScaleFactor;
        mScaleFactor = scaleMin;
        mXOffset *= scaleDiff;
        mYOffset *= scaleDiff;
    } else if (mScaleFactor > scaleMax) {
        double scaleDiff = scaleMax / mScaleFactor;
        mScaleFactor = scaleMax;
        mXOffset *= scaleDiff;
        mYOffset *= scaleDiff;
    }

    // Optionally follow a car
    if (mFollowCar >= 0) {
        for (int i = 0;i < mCarInfo.size();i++) {
            CarInfo &carInfo = mCarInfo[i];
            if (carInfo.getId() == mFollowCar) {
                LocPoint followLoc = carInfo.getLocation();
                mXOffset = -followLoc.getX() * 1000.0 * mScaleFactor;
                mYOffset = -followLoc.getY() * 1000.0 * mScaleFactor;
            }
        }
    }

    // Limit the offset to avoid overflow at 2^31 mm
    double lim = 2000000000.0 * mScaleFactor;
    if (mXOffset > lim) {
        mXOffset = lim;
    } else if (mXOffset < -lim) {
        mXOffset = -lim;
    }

    if (mYOffset > lim) {
        mYOffset = lim;
    } else if (mYOffset < -lim) {
        mYOffset = -lim;
    }

    // Paint begins here
    painter.fillRect(0, 0, width, height, QBrush(Qt::transparent));


    // Map coordinate transforms
    QTransform drawTrans;
    drawTrans.translate(width / 2 + mXOffset, height / 2 - mYOffset);
    drawTrans.scale(mScaleFactor, -mScaleFactor);
    drawTrans.rotate(mRotation);

    // Text coordinates
    QTransform txtTrans;
    txtTrans.translate(0, 0);
    txtTrans.scale(1, 1);
    txtTrans.rotate(0);

    // Set font
    QFont font = this->font();
    font.setPointSize(10);
    font.setFamily("Monospace");
    painter.setFont(font);

    // Grid parameters
    double stepGrid = 20.0 * ceil(1.0 / ((mScaleFactor * 10.0) / 50.0));
    bool gridDiv = false;
    if (round(log10(stepGrid)) > log10(stepGrid)) {
        gridDiv = true;
    }
    stepGrid = pow(10.0, round(log10(stepGrid)));
    if (gridDiv) {
        stepGrid /= 2.0;
    }

    // Draw perspective pixmaps first
    painter.setTransform(drawTrans);
    for(int i = 0;i < mPerspectivePixmaps.size();i++) {
        mPerspectivePixmaps[i].drawUsingPainter(painter);
    }

    // View center, width and height in m
    const double cx = -mXOffset / mScaleFactor / 1000.0;
    const double cy = -mYOffset / mScaleFactor / 1000.0;
    const double view_w = width / mScaleFactor / 1000.0;
    const double view_h = height / mScaleFactor / 1000.0;

    // Draw openstreetmap tiles
    if (mDrawOpenStreetmap) {
        paintTiles(painter,cx, cy, view_w, view_h, highQuality);
    }

    const double zeroAxisWidth = 3;
    const QColor zeroAxisColor = Qt::red;
    const QColor firstAxisColor = Qt::gray;
    const QColor secondAxisColor = Qt::blue;
    const QColor textColor = QPalette::Foreground;

    QString txt;
    QPointF pt_txt;
    QRectF rect_txt;
    QPen pen;

    if (mDrawGrid) {
        paintGrid(painter,pen, drawTrans, txtTrans, width, height, stepGrid, txt, pt_txt, textColor, zeroAxisColor, firstAxisColor,secondAxisColor,zeroAxisWidth);
    }
    //paintInfoTraces();
    mVisibleInfoTracePoints.clear();

    // Draw info trace
    int info_segments = 0;
    int info_points = 0;

    for (int in = 0;in < mInfoTraces.size();in++) {
        paintTrace(mInfoTraces[in],painter,pen,mInfoTraceNow == in,drawTrans, txtTrans,cx, cy, view_w, view_h,info_segments, info_points);
    }

    paintClosestPoint(painter,pen,drawTrans, txtTrans, pt_txt,rect_txt );

//    qDebug() << "Middle painting";

    paintCarTraces(painter,pen,drawTrans);
    // Draw fields
    bool isSelected;
    for (int fn = 0;fn < mFields->size();fn++) {
        isSelected= (mFields->mRouteNow == fn) && (focusBorder==true);
        MapRoute &fieldNow = mFields->at(fn);
        fieldNow.paintBorder(painter, pen, isSelected, mScaleFactor, drawTrans);
    }

    // Draw routes
    isSelected;
    for (int rn = 0;rn < mPaths->size();rn++) {
        MapRoute &routeNow = mPaths->at(rn);
        isSelected= (mPaths->mRouteNow == rn) && (focusBorder==false);
        routeNow.paintPath(this, painter, pen, isSelected, mScaleFactor, drawTrans, txt, pt_txt, rect_txt, txtTrans, highQuality);
    }


    // Map module painting
    painter.save();
    for (MapModule *m: mMapModules) {
        m->processPaint(painter, width, height, highQuality,
                        drawTrans, txtTrans, mScaleFactor);
    }
    painter.restore();

    // Draw cars
    painter.setPen(QPen(textColor));
    for(int i = 0;i < mCarInfo.size();i++) {
        paintCar(mCarInfo[i],painter,pen, drawTrans, txtTrans,txt, pt_txt,textColor,rect_txt);
    }

    // Draw anchors
    for (int i = 0;i < mAnchors.size();i++) {
        paintAnchor(mAnchors[i],painter,pen, drawTrans, txtTrans,txt, pt_txt,textColor,rect_txt);
    };

    double start_txt = 30.0;

    paintCamera(painter,txtTrans, width,start_txt);

    const double txtOffset = 145.0;
    const double txt_row_h = 20.0;

    paintUnitZoomGeneralinfo(painter,font, txtTrans, width, stepGrid,txt,textColor,start_txt,txtOffset,txt_row_h, info_segments, info_points);

    // Current route info
    if (mPaths->size()>0)
    {
        mPaths->getCurrent().routeinfo(this, painter,start_txt,txtOffset,txt_row_h,width,txt);
    };
    painter.end();
//    qDebug() << "End painting";

}

void MapWidget::paintClosestPoint(QPainter &painter,QPen &pen,QTransform drawTrans, QTransform txtTrans, QPointF& pt_txt,QRectF &rect_txt )
{
    // Draw point closest to mouse pointer
    if (mClosestInfo.getInfo().size() > 0) {
        QPointF p = mClosestInfo.getPointMm();
        QPointF p2 = drawTrans.map(p);

        painter.setTransform(txtTrans);
        pen.setColor(Qt::green);
        painter.setBrush(Qt::green);
        painter.setPen(Qt::green);
        painter.drawEllipse(p2, mClosestInfo.getRadius(), mClosestInfo.getRadius());

        pt_txt.setX(p.x() + 5 / mScaleFactor);
        pt_txt.setY(p.y());
        painter.setTransform(txtTrans);
        pt_txt = drawTrans.map(pt_txt);
        pen.setColor(Qt::black);
        painter.setPen(pen);
        rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 20,
                           pt_txt.x() + 500, pt_txt.y() + 500);
        painter.drawText(rect_txt, Qt::AlignTop | Qt::AlignLeft, mClosestInfo.getInfo());
    }

}

void MapWidget::paintUnitZoomGeneralinfo(QPainter &painter,QFont font, QTransform txtTrans, int width, double stepGrid,QString& txt, const QColor textColor,   double& start_txt,const double txtOffset,const double txt_row_h,    int &info_segments, int &info_points)
{
    painter.setTransform(txtTrans);
    font.setPointSize(10);
    painter.setFont(font);
    painter.setPen(QPen(textColor));

    // Draw units (m)
    if (mDrawGrid) {
        double res = stepGrid / 1000.0;
        if (res >= 1000.0) {
            txt.sprintf("Grid res: %.0f km", res / 1000.0);
        } else if (res >= 1.0) {
            txt.sprintf("Grid res: %.0f m", res);
        } else {
            txt.sprintf("Grid res: %.0f cm", res * 100.0);
        }
        painter.drawText(width - txtOffset, start_txt, txt);
        start_txt += txt_row_h;
    }

    // Draw zoom level
    txt.sprintf("Zoom: %.7f", mScaleFactor);
    painter.drawText(width - txtOffset, start_txt, txt);
    start_txt += txt_row_h;

    // Draw OSM zoom level
    if (mDrawOpenStreetmap) {
        txt.sprintf("OSM zoom: %d", mOsmZoomLevel);
        painter.drawText(width - txtOffset, start_txt, txt);
        start_txt += txt_row_h;

        if (mDrawOsmStats) {
            txt.sprintf("DL Tiles: %d", mOsm->getTilesDownloaded());
            painter.drawText(width - txtOffset, start_txt, txt);
            start_txt += txt_row_h;

            txt.sprintf("HDD Tiles: %d", mOsm->getHddTilesLoaded());
            painter.drawText(width - txtOffset, start_txt, txt);
            start_txt += txt_row_h;

            txt.sprintf("RAM Tiles: %d", mOsm->getRamTilesLoaded());
            painter.drawText(width - txtOffset, start_txt, txt);
            start_txt += txt_row_h;
        }
    }

    if (mInteractionMode != InteractionModeDefault) {
        txt.sprintf("IMode: %d", mInteractionMode);
        painter.drawText(width - txtOffset, start_txt, txt);
        start_txt += txt_row_h;
    }

    // Some info
    if (info_segments > 0) {
        txt.sprintf("Info seg: %d", info_segments);
        painter.drawText(width - txtOffset, start_txt, txt);
        start_txt += txt_row_h;
    }

    if (info_points > 0) {
        txt.sprintf("Info pts: %d", info_points);
        painter.drawText(width - txtOffset, start_txt, txt);
        start_txt += txt_row_h;
    }

}

void MapWidget::paintCamera(QPainter &painter,QTransform txtTrans, int width,double& start_txt)
{
    painter.setTransform(txtTrans);

    if (!mLastCameraImage.isNull() && mCameraImageWidth > 0.001) {
        double imgWidth = (double)width * mCameraImageWidth;
        double imgHeight = (double)mLastCameraImage.height() *
                (imgWidth / (double)mLastCameraImage.width());

        QRectF target(width - imgWidth, 0.0, imgWidth, imgHeight);
        QRectF source(0.0, 0.0, mLastCameraImage.width(), mLastCameraImage.height());

        start_txt += imgHeight;

        painter.setOpacity(mCameraImageOpacity);
        painter.drawImage(target, mLastCameraImage, source);
        painter.setOpacity(1.0);
    }

}


void MapWidget::paintTraceVechicle(QPainter &painter,QPen &pen, QTransform drawTrans,QVector<LocPoint> mTrace,double traceWidth,QColor traceColour)
{
    // Draw trace for the selected car
    pen.setWidthF(traceWidth);
    pen.setColor(traceColour);
    painter.setPen(pen);
    painter.setTransform(drawTrans);
    if (bUpdateable)
    {
        for (int i = 1;i < mTrace.size();i++) {
        painter.drawLine(mTrace[i - 1].getX() * 1000.0, mTrace[i - 1].getY() * 1000.0,
                         mTrace[i].getX() * 1000.0, mTrace[i].getY() * 1000.0);
        }
    } else
    {
        if(mTrace.size()>0)
        {
            for (int i = iLogstart+1;i < iLogend;i++) {
                    painter.drawLine(mTrace[i - 1].getX() * 1000.0, mTrace[i - 1].getY() * 1000.0,
                                     mTrace[i].getX() * 1000.0, mTrace[i].getY() * 1000.0);
            }
        }
    };
}

void MapWidget::paintCarTraces(QPainter &painter,QPen &pen, QTransform drawTrans)
{
    paintTraceVechicle(painter,pen, drawTrans,mCarTrace,5.0 / mScaleFactor,Qt::red);
    paintTraceVechicle(painter,pen, drawTrans,mCarTraceGps,2.5 / mScaleFactor,Qt::magenta);

    // Draw UWB trace for the selected car
    if (mDrawUwbTrace) {
        paintTraceVechicle(painter,pen, drawTrans,mCarTraceUwb ,2.5 / mScaleFactor,Qt::green);
    }
}

void MapWidget::paintTrace(QList<LocPoint> &itNow,QPainter &painter,QPen &pen, bool isActive, QTransform drawTrans, QTransform txtTrans,const double cx, const double cy, const double view_w, const double view_h,int& info_segments, int& info_points)
{
    // View boundries in mm
    const double xStart2 = (cx - view_w / 2.0) * 1000.0;
    const double yStart2 = (cy - view_h / 2.0) * 1000.0;
    const double xEnd2 = (cx + view_w / 2.0) * 1000.0;
    const double yEnd2 = (cy + view_h / 2.0) * 1000.0;

    if (isActive) {
        pen.setColor(Qt::darkGreen);
        painter.setBrush(Qt::green);
    } else {
        pen.setColor(Qt::darkGreen);
        painter.setBrush(Qt::green);
    }

    pen.setWidthF(3.0);
    painter.setPen(pen);
    painter.setTransform(txtTrans);

    const double info_min_dist = 0.02;

    int last_visible = 0;
    for (int i = 1;i < itNow.size();i++) {
        double dist_view = itNow.at(i).getDistanceTo(itNow.at(last_visible)) * mScaleFactor;
        if (dist_view < info_min_dist) {
            continue;
        }

        bool draw = isPointWithinRect(itNow[last_visible].getPointMm(), xStart2, xEnd2, yStart2, yEnd2);

        if (!draw) {
            draw = isPointWithinRect(itNow[i].getPointMm(), xStart2, xEnd2, yStart2, yEnd2);
        }

        if (!draw) {
            draw = isLineSegmentWithinRect(itNow[last_visible].getPointMm(),
                                           itNow[i].getPointMm(),
                                           xStart2, xEnd2, yStart2, yEnd2);
        }

        if (draw && itNow[i].getDrawLine()) {
            QPointF p1 = drawTrans.map(itNow[last_visible].getPointMm());
            QPointF p2 = drawTrans.map(itNow[i].getPointMm());

            painter.drawLine(p1, p2);
            info_segments++;
        }

        last_visible = i;
    }

    QList<LocPoint> pts_green;
    QList<LocPoint> pts_red;
    QList<LocPoint> pts_other;

    for (int i = 0;i < itNow.size();i++) {
        LocPoint ip = itNow[i];

        if (!isActive) {
            ip.setColor(Qt::gray);
        }

        if (ip.getColor() == Qt::darkGreen || ip.getColor() == Qt::green) {
            pts_green.append(ip);
        } else if (ip.getColor() == Qt::darkRed || ip.getColor() == Qt::red) {
            pts_red.append(ip);
        } else {
            pts_other.append(ip);
        }
    }

    info_points += drawInfoPoints(painter, pts_green, drawTrans, txtTrans,
                                  xStart2, xEnd2, yStart2, yEnd2, info_min_dist);
    info_points += drawInfoPoints(painter, pts_other, drawTrans, txtTrans,
                                  xStart2, xEnd2, yStart2, yEnd2, info_min_dist);
    info_points += drawInfoPoints(painter, pts_red, drawTrans, txtTrans,
                                  xStart2, xEnd2, yStart2, yEnd2, info_min_dist);

}

void MapWidget::paintAnchor(LocPoint &anchor,QPainter &painter,QPen &pen, QTransform drawTrans, QTransform txtTrans,QString& txt, QPointF& pt_txt,const QColor &textColor,QRectF &rect_txt)
{
    double x = anchor.getX() * 1000.0;
    double y = anchor.getY() * 1000.0;
    painter.setTransform(drawTrans);

    // Draw anchor
    painter.setBrush(QBrush(Qt::red));
    painter.translate(x, y);

    pt_txt.setX(x);
    pt_txt.setY(y);
    painter.setTransform(txtTrans);
    pt_txt = drawTrans.map(pt_txt);

    painter.drawRoundedRect(pt_txt.x() - 10,
                            pt_txt.y() - 10,
                            20, 20, 10, 10);

    // Print data
    txt.sprintf("Anchor %d\n"
                "Pos    : (%.3f, %.3f)\n"
                "Height : %.2f m",
                anchor.getId(),
                anchor.getX(), anchor.getY(),
                anchor.getHeight());
    pt_txt.setX(x);
    pt_txt.setY(y);
    painter.setTransform(txtTrans);
    pt_txt = drawTrans.map(pt_txt);
    rect_txt.setCoords(pt_txt.x() + 15, pt_txt.y() - 20,
                       pt_txt.x() + 500, pt_txt.y() + 500);
    painter.drawText(rect_txt, txt);
}

void MapWidget::paintCar(CarInfo &carInfo,QPainter &painter,QPen &pen, QTransform drawTrans, QTransform txtTrans,QString &txt, QPointF &pt_txt,const QColor &textColor,QRectF &rect_txt)
{
    LocPoint pos = carInfo.getLocation();
    LocPoint pos_gps = carInfo.getLocationGps();

    const double car_len = carInfo.getLength() * 1000.0;
    const double car_w = carInfo.getWidth() * 1000.0;
    const double car_corner = carInfo.getCornerRadius() * 1000.0;

    double x = pos.getX() * 1000.0;
    double y = pos.getY() * 1000.0;
    double x_gps = pos_gps.getX() * 1000.0;
    double y_gps = pos_gps.getY() * 1000.0;
    double angle = pos.getYaw() * 180.0 / M_PI;
    painter.setTransform(drawTrans);

    QColor col_sigma = Qt::red;
    QColor col_wheels = Qt::black;
    QColor col_bumper = Qt::green;
    QColor col_hull = carInfo.getColor();
    QColor col_center = Qt::blue;
    QColor col_gps = Qt::magenta;
    QColor col_ap = carInfo.getColor();

    if (carInfo.getId() != mSelectedCar) {
        col_wheels = Qt::darkGray;
        col_bumper = Qt::lightGray;
        col_ap = Qt::lightGray;
    }

    // Draw standard deviation
    if (pos.getSigma() > 0.0) {
        QColor col = col_sigma;
        col.setAlphaF(0.2);
        painter.setBrush(QBrush(col));
        painter.drawEllipse(pos.getPointMm(), pos.getSigma() * 1000.0, pos.getSigma() * 1000.0);
    }

    // Draw car
    painter.setBrush(QBrush(col_wheels));
    painter.save();
    painter.translate(x, y);
    painter.rotate(-angle);
    // Wheels
    painter.drawRoundedRect(-car_len / 12.0,-(car_w / 2), car_len / 6.0, car_w, car_corner / 3, car_corner / 3);
    painter.drawRoundedRect(car_len - car_len / 2.5,-(car_w / 2), car_len / 6.0, car_w, car_corner / 3, car_corner / 3);
    // Front bumper
    painter.setBrush(col_bumper);
    painter.drawRoundedRect(-car_len / 6.0, -((car_w - car_len / 20.0) / 2.0), car_len, car_w - car_len / 20.0, car_corner, car_corner);
    // Hull
    painter.setBrush(col_hull);
    painter.drawRoundedRect(-car_len / 6.0, -((car_w - car_len / 20.0) / 2.0), car_len - (car_len / 20.0), car_w - car_len / 20.0, car_corner, car_corner);
    painter.restore();

    // Center
    painter.setBrush(col_center);
    painter.drawEllipse(QPointF(x, y), car_w / 15.0, car_w / 15.0);

    // GPS Location
    painter.setBrush(col_gps);
//        painter.drawEllipse(QPointF(x_gps, y_gps), car_w / 15.0, car_w / 15.0);
    painter.drawEllipse(QPointF(x_gps, y_gps), car_w / 3.0, car_w / 3.0);

    // Autopilot state
    LocPoint ap_goal = carInfo.getApGoal();
    if (ap_goal.getRadius() > 0.0) {
        pen.setColor(col_ap);
        painter.setPen(pen);
        QPointF pm = pos.getPointMm();
        painter.setBrush(Qt::transparent);
        painter.drawEllipse(pm, ap_goal.getRadius() * 1000.0, ap_goal.getRadius() * 1000.0);

        QPointF p = ap_goal.getPointMm();
        pen.setWidthF(3.0 / mScaleFactor);
        painter.setBrush(col_gps);
        painter.drawEllipse(p, 10 / mScaleFactor, 10 / mScaleFactor);
    }

    painter.setPen(QPen(textColor));

    // Print data
    QTime t = QTime::fromMSecsSinceStartOfDay(carInfo.getTime());
    QString solStr;
    if (!carInfo.getLocationGps().getInfo().isEmpty()) {
        solStr = QString("Sol: %1\n").arg(carInfo.getLocationGps().getInfo());
    }
    txt.sprintf("%s\n"
                "%s"
                "(%.3f, %.3f, %.0f)\n"
                "%02d:%02d:%02d:%03d",
                carInfo.getName().toLocal8Bit().data(),
                solStr.toLocal8Bit().data(),
                pos.getX(), pos.getY(), angle,
                t.hour(), t.minute(), t.second(), t.msec());
    pt_txt.setX(x + 120 + (car_len - 190) * ((cos(pos.getYaw()) + 1) / 2));
    pt_txt.setY(y);
    painter.setTransform(txtTrans);
    pt_txt = drawTrans.map(pt_txt);
    rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 20,
                       pt_txt.x() + 400, pt_txt.y() + 100);
    painter.drawText(rect_txt, txt);

}

void MapWidget::paintGrid(QPainter &painter,QPen &pen, QTransform drawTrans, QTransform txtTrans, int width, int height, double stepGrid,QString txt, QPointF pt_txt,const QColor textColor, const QColor zeroAxisColor,const QColor firstAxisColor,const QColor secondAxisColor,const double zeroAxisWidth)
{
    // Grid boundries in mm
    const double xStart = -ceil(width / stepGrid / mScaleFactor) * stepGrid - ceil(mXOffset / stepGrid / mScaleFactor) * stepGrid;
    const double xEnd = ceil(width / stepGrid / mScaleFactor) * stepGrid - floor(mXOffset / stepGrid / mScaleFactor) * stepGrid;
    const double yStart = -ceil(height / stepGrid / mScaleFactor) * stepGrid - ceil(mYOffset / stepGrid / mScaleFactor) * stepGrid;
    const double yEnd = ceil(height / stepGrid / mScaleFactor) * stepGrid - floor(mYOffset / stepGrid / mScaleFactor) * stepGrid;

    painter.setTransform(txtTrans);

    // Draw Y-axis segments
    for (double i = xStart;i < xEnd;i += stepGrid) {
        if (fabs(i) < 1e-3) {
            i = 0.0;
        }

        if ((int)(i / stepGrid) % 2) {
            pen.setWidth(0);
            pen.setColor(firstAxisColor);
            painter.setPen(pen);
        } else {
            txt.sprintf("%.2f m", i / 1000.0);

            pt_txt.setX(i);
            pt_txt.setY(0);
            pt_txt = drawTrans.map(pt_txt);
            pt_txt.setX(pt_txt.x() - 5);
            pt_txt.setY(height - 10);
            painter.setPen(QPen(textColor));
            painter.save();
            painter.translate(pt_txt);
            painter.rotate(-90);
            painter.drawText(0, 0, txt);
            painter.restore();

            if (fabs(i) < 1e-3) {
                pen.setWidthF(zeroAxisWidth);
                pen.setColor(zeroAxisColor);
            } else {
                pen.setWidth(0);
                pen.setColor(secondAxisColor);
            }
            painter.setPen(pen);
        }

        QPointF pt_start(i, yStart);
        QPointF pt_end(i, yEnd);
        pt_start = drawTrans.map(pt_start);
        pt_end = drawTrans.map(pt_end);
        painter.drawLine(pt_start, pt_end);
    }

    // Draw X-axis segments
    for (double i = yStart;i < yEnd;i += stepGrid) {
        if (fabs(i) < 1e-3) {
            i = 0.0;
        }

        if ((int)(i / stepGrid) % 2) {
            pen.setWidth(0);
            pen.setColor(firstAxisColor);
            painter.setPen(pen);
        } else {
            txt.sprintf("%.2f m", i / 1000.0);
            pt_txt.setY(i);

            pt_txt = drawTrans.map(pt_txt);
            pt_txt.setX(10);
            pt_txt.setY(pt_txt.y() - 5);
            painter.setPen(QPen(textColor));
            painter.drawText(pt_txt, txt);

            if (fabs(i) < 1e-3) {
                pen.setWidthF(zeroAxisWidth);
                pen.setColor(zeroAxisColor);
            } else {
                pen.setWidth(0);
                pen.setColor(secondAxisColor);
            }
            painter.setPen(pen);
        }

        QPointF pt_start(xStart, i);
        QPointF pt_end(xEnd, i);
        pt_start = drawTrans.map(pt_start);
        pt_end = drawTrans.map(pt_end);
        painter.drawLine(pt_start, pt_end);
    }
}

void MapWidget::paintTiles(QPainter &painter,const double cx, const double cy, const double view_w, const double view_h, bool highQuality)
{
    double i_llh[3];
    i_llh[0] = mRefLat;
    i_llh[1] = mRefLon;
    i_llh[2] = mRefHeight;

    mOsmZoomLevel = (int)round(log(mScaleFactor * mOsmRes * 100000000.0 *
                                    cos(i_llh[0] * M_PI / 180.0)) / log(2.0));
    if (mOsmZoomLevel > mOsmMaxZoomLevel) {
        mOsmZoomLevel = mOsmMaxZoomLevel;
    } else if (mOsmZoomLevel < 0) {
        mOsmZoomLevel = 0;
    }

    int xt = OsmTile::long2tilex(i_llh[1], mOsmZoomLevel);
    int yt = OsmTile::lat2tiley(i_llh[0], mOsmZoomLevel);

    double llh_t[3];
    llh_t[0] = OsmTile::tiley2lat(yt, mOsmZoomLevel);
    llh_t[1] = OsmTile::tilex2long(xt, mOsmZoomLevel);
    llh_t[2] = 0.0;

    double xyz[3];
    utility::llhToEnu(i_llh, llh_t, xyz);

    // Calculate scale at ENU origin
    double w = OsmTile::lat2width(i_llh[0], mOsmZoomLevel);

    int t_ofs_x = (int)ceil(-(cx - view_w / 2.0) / w);
    int t_ofs_y = (int)ceil((cy + view_h / 2.0) / w);

    if (!highQuality) {
        painter.setRenderHint(QPainter::SmoothPixmapTransform, mAntialiasOsm);
    }

    QTransform transOld = painter.transform();
    QTransform trans = painter.transform();
    trans.scale(1, -1);
    painter.setTransform(trans);

    for (int j = 0;j < 40;j++) {
        for (int i = 0;i < 40;i++) {
            int xt_i = xt + i - t_ofs_x;
            int yt_i = yt + j - t_ofs_y;
            double ts_x = xyz[0] + w * i - (double)t_ofs_x * w;
            double ts_y = -xyz[1] + w * j - (double)t_ofs_y * w;

            // We are outside the view
            if (ts_x > (cx + view_w / 2.0)) {
                break;
            } else if ((ts_y - w) > (-cy + view_h / 2.0)) {
                break;
            }

            int res;
            OsmTile t = mOsm->getTile(mOsmZoomLevel, xt_i, yt_i, res);

            if (w < 0.0) {
                w = t.getWidthTop();
            }

            painter.drawPixmap(ts_x * 1000.0, ts_y * 1000.0,
                               w * 1000.0, w * 1000.0, t.pixmap());

            if (res == 0 && !mOsm->downloadQueueFull()) {
                mOsm->downloadTile(mOsmZoomLevel, xt_i, yt_i);
            }
        }
    }

    // Restore painter
    painter.setTransform(transOld);

    if (!highQuality) {
        painter.setRenderHint(QPainter::SmoothPixmapTransform, mAntialiasDrawings);
    }

}

bool MapWidget::loadXMLRoute(QXmlStreamReader* stream,bool isBorder)
{
    // Look for routes tag
    bool routes_found = false;
    while (stream->readNextStartElement()) {
        if (stream->name() == "routes") {
            routes_found = true;
            break;
        }
    }

    if (routes_found) {
        QList<QPair<int, MapRoute > > routes;
        QList<LocPoint> anchors;

        while (stream->readNextStartElement()) {
            QString name = stream->name().toString();

            if (name == "route") {
                int id = -1;
                MapRoute route;

                while (stream->readNextStartElement()) {
                    QString name2 = stream->name().toString();

                    if (name2 == "id") {
                        id = stream->readElementText().toInt();
                    } else if (name2 == "point") {
                        LocPoint p;

                        while (stream->readNextStartElement()) {
                            QString name3 = stream->name().toString();

                            if (name3 == "x") {
                                p.setX(stream->readElementText().toDouble());
                            } else if (name3 == "y") {
                                p.setY(stream->readElementText().toDouble());
                            } else if (name3 == "speed") {
                                p.setSpeed(stream->readElementText().toDouble());
                            } else if (name3 == "time") {
                                p.setTime(stream->readElementText().toInt());
                            } else if (name3 == "attributes") {
                                p.setAttributes(stream->readElementText().toInt());
                            } else {
                                qWarning() << ": Unknown XML element :" << name2;
                                stream->skipCurrentElement();
                            }
                        }

                        route.append(p);
                    } else {
                        qWarning() << ": Unknown XML element :" << name2;
                        stream->skipCurrentElement();
                    }

                    if (stream->hasError()) {
                        qWarning() << " : XML ERROR :" << stream->errorString();
                    }
                }

                routes.append(QPair<int, MapRoute >(id, route));
            } else if (name == "anchors") {
                while (stream->readNextStartElement()) {
                    QString name2 = stream->name().toString();

                    if (name2 == "anchor") {
                        LocPoint p;

                        while (stream->readNextStartElement()) {
                            QString name3 = stream->name().toString();

                            if (name3 == "x") {
                                p.setX(stream->readElementText().toDouble());
                            } else if (name3 == "y") {
                                p.setY(stream->readElementText().toDouble());
                            } else if (name3 == "height") {
                                p.setHeight(stream->readElementText().toDouble());
                            } else if (name3 == "id") {
                                p.setId(stream->readElementText().toInt());
                            } else {
                                qWarning() << ": Unknown XML element :" << name2;
                                stream->skipCurrentElement();
                            }
                        }

                        anchors.append(p);
                    } else {
                        qWarning() << ": Unknown XML element :" << name2;
                        stream->skipCurrentElement();
                    }

                    if (stream->hasError()) {
                        qWarning() << " : XML ERROR :" << stream->errorString();
                    }
                }
            }

            if (stream->hasError()) {
                qWarning() << "XML ERROR :" << stream->errorString();
                qWarning() << stream->lineNumber() << stream->columnNumber();
            }
        }
        for (QPair<int, MapRoute > r: routes) {
            if (r.first >= 0) {
                if (isBorder)
                {
                    int fieldLast = this->getFieldNow();
                    this->setFieldNow(r.first);
                    this->setField(r.second);
                    this->setFieldNow(fieldLast);
                } else
                {
                    int routeLast = this->getPathNow();
                    this->setPathNow(r.first);
                    this->setPath(r.second);
                    this->setPathNow(routeLast);
                }
            } else {
                if (isBorder)
                {
                    this->addField(r.second);
                } else
                {
                    this->addPath(r.second);
                }
            }
        }

        for (LocPoint p: anchors) {
            this->addAnchor(p);
        }
    }
    return routes_found;

}

void MapWidget::saveXMLCurrentRoute(QXmlStreamWriter* stream)
{
    stream->setCodec("UTF-8");
    stream->setAutoFormatting(true);
    stream->writeStartDocument();

    stream->writeStartElement("routes");

    MapRoute *currentRoute;
    int mRoutePointSelected;
    if (focusBorder)
    {
        currentRoute=&(mFields->getCurrent());
        qDebug() << "Acting on border";
        qDebug() << "Border size: " << currentRoute->size();
    } else
    {
        currentRoute=&(mPaths->getCurrent());
        qDebug() << "Acting on path";
    }
    saveXMLRoute(stream, *currentRoute,false, 0);

    stream->writeEndDocument();
}

void MapWidget::saveXMLRoute(QXmlStreamWriter* stream, MapRoute route,bool withId, int i)
{
    stream->writeStartElement("route");

    if (withId) {
        stream->writeTextElement("id", QString::number(i));
    }

    for (const LocPoint p: route.mRoute) {
        stream->writeStartElement("point");
        stream->writeTextElement("x", QString::number(p.getX()));
        stream->writeTextElement("y", QString::number(p.getY()));
        stream->writeTextElement("speed", QString::number(p.getSpeed()));
        stream->writeTextElement("time", QString::number(p.getTime()));
        stream->writeTextElement("attributes", QString::number(p.getAttributes()));
        stream->writeEndElement();
    }
    stream->writeEndElement();
}


void MapWidget::saveXMLRoutes(QXmlStreamWriter* stream, bool withId)
{
    stream->setCodec("UTF-8");
    stream->setAutoFormatting(true);
    stream->writeStartDocument();

    stream->writeStartElement("routes");

    QList<LocPoint> anchors = this->getAnchors();
    QList<MapRoute> routes;
    if (focusBorder)
    {
        routes = this->getFields();
    } else
    {
        routes = this->getPaths();
    }
/*
    if (!anchors.isEmpty()) {
        stream->writeStartElement("anchors");
        for (LocPoint p: anchors) {
            stream->writeStartElement("anchor");
            stream->writeTextElement("x", QString::number(p.getX()));
            stream->writeTextElement("y", QString::number(p.getY()));
            stream->writeTextElement("height", QString::number(p.getHeight()));
            stream->writeTextElement("id", QString::number(p.getId()));
            stream->writeEndElement();
        }
        stream->writeEndElement();
    }
*/
    for (int i = 0;i < routes.size();i++) {
        if (!routes.at(i).isEmpty()) {
            saveXMLRoute(stream,routes.at(i),i,withId);
        }
    }
    stream->writeEndDocument();
}


void MapWidget::updateTraces()
{
    if (bUpdateable)
    {
        // Store trace for the selected car
        if (mTraceCar >= 0) {
            for (int i = 0;i < mCarInfo.size();i++) {
                CarInfo &carInfo = mCarInfo[i];
                if (carInfo.getId() == mTraceCar) {
                    if (mCarTrace.isEmpty()) {
                        mCarTrace.append(carInfo.getLocation());
                    }
                    if (mCarTrace.last().getDistanceTo(carInfo.getLocation()) > mTraceMinSpaceCar) {
                        mCarTrace.append(carInfo.getLocation());
                    }
                    // GPS trace
                    if (mCarTraceGps.isEmpty()) {
                        mCarTraceGps.append(carInfo.getLocationGps());
                    }
                    if (mCarTraceGps.last().getDistanceTo(carInfo.getLocationGps()) > mTraceMinSpaceGps) {
                        mCarTraceGps.append(carInfo.getLocationGps());
                    }
                    // UWB trace
                    if (mCarTraceUwb.isEmpty()) {
                        mCarTraceUwb.append(carInfo.getLocationUwb());
                    }
                    if (mCarTraceUwb.last().getDistanceTo(carInfo.getLocationUwb()) > mTraceMinSpaceCar) {
                        mCarTraceUwb.append(carInfo.getLocationUwb());
                    }
                }
            }
        }

        // Truncate traces
        while (mCarTrace.size() > 5000) {
            mCarTrace.removeFirst();
        }

        while (mCarTraceGps.size() > 1800) {
            mCarTraceGps.removeFirst();
        }

        while (mCarTraceUwb.size() > 5000) {
            mCarTraceUwb.removeFirst();
        }
    }
}

std::array<double, 4> MapWidget::findExtremeValuesFieldBorders()
{
    std::array<double, 4> extremes;
    extremes[0]=9999999;
    extremes[1]=9999999;
    extremes[2]=-9999999;
    extremes[3]=-9999999;

    QList<LocPoint> allpoints;
/*    for (const MapRoute& route : mRoutes)
    {
        if (route.getIsBorder())
        allpoints.append(route.mRoute);
    }*/
    for (const MapRoute& field : mFields->mCollection)
    {
        allpoints.append(field.mRoute);
    }
    for (const LocPoint& point : allpoints)
    {
        double x=point.getX();
        double y=point.getY();

        if (x<extremes[0]) extremes[0]=x;
        if (x>extremes[2]) extremes[2]=x;
        if (y<extremes[1]) extremes[1]=y;
        if (y>extremes[3]) extremes[3]=y;

    }
    return extremes;
}

/*
MapRoute::MapRoute(QList<LocPoint> route, QList<LocPoint> infotrace)
{
    mRoute=route;
    mInfoTrace=infotrace;
}
*/

int MapRoute::size()
{
    return mRoute.size();
}

void MapRoute::clear()
{
    mRoute.clear();
}

bool MapRoute::isEmpty() const
{
    return mRoute.isEmpty();
}

void MapRoute::append(LocPoint point)
{
    mRoute.append(point);
}

void MapRoute::append(const QList<LocPoint> &points)
{
    mRoute.append(points);
}

void MapRoute::append(const MapRoute &mr)
{
    mRoute.append(mr.mRoute);
}

LocPoint& MapRoute::first()
{
    return mRoute.first();
}

LocPoint& MapRoute::last()
{
    return mRoute.last();
}

void MapRoute::removeLast()
{
    mRoute.removeLast();
}
void MapRoute::prepend(const LocPoint &value)
{
    mRoute.prepend(value);
}

void MapRoute::removeAt(int i)
{
    mRoute.removeAt(i);
}

QList<LocPoint>::const_iterator MapRoute::begin() const
{
    return mRoute.begin();
}

QList<LocPoint>::const_iterator MapRoute::end() const
{
    return mRoute.end();
}

double MapRoute::getArea()
{
    int n = this->size();
    double area = 0.0;

    for (int i = 0; i < n; ++i) {
        int j = (i + 1) % n;
        area += mRoute[i].getX() * mRoute[j].getY();
        area -= mRoute[i].getY() * mRoute[j].getX();
    }
    area = std::abs(area) * 0.5;
    return area;
}

LocPoint &	MapRoute::operator[](int i)
{
    return mRoute[i];
}

void MapRoute::setIsBorder(bool border)
{
    isBorder=border;
}

bool MapRoute::getIsBorder() const
{
    return isBorder;
}

/*
void MapRoute::paint(MapWidget* mapWidget, QPainter &painter, QPen &pen, bool isSelected, double mScaleFactor, QTransform drawTrans, QString txt, QPointF pt_txt, QRectF rect_txt, QTransform txtTrans, bool highQuality)
{
    if (isBorder)
    {
        paintBorder(painter, pen, isSelected, mScaleFactor, drawTrans);
    } else
    {
        paintPath(mapWidget, painter, pen, isSelected, mScaleFactor, drawTrans, txt, pt_txt, rect_txt, txtTrans, highQuality);
    }
}
*/

void MapRoute::paintPath(MapWidget* mapWidget, QPainter &painter, QPen &pen, bool isSelected, double mScaleFactor, QTransform drawTrans, QString txt, QPointF pt_txt, QRectF rect_txt, QTransform txtTrans, bool highQuality)
{
    Qt::GlobalColor defaultDarkColor = Qt::darkGray;
    Qt::GlobalColor defaultColor = Qt::gray;
    if (isSelected) {
        defaultDarkColor = Qt::darkYellow;
        defaultColor = Qt::yellow;
    }

    pen.setWidthF(5.0 / mScaleFactor);
    painter.setTransform(drawTrans);

    int nPoints=this->size();
    for (int i = 1;i < nPoints;i++) {
        pen.setColor(defaultDarkColor);
        painter.setBrush(defaultColor);
        if (isSelected && (mRoute[i - 1].getAttributes() & ATTR_HYDRAULIC_FRONT_DOWN) && (mRoute[i].getAttributes() & ATTR_HYDRAULIC_FRONT_DOWN)) {
            pen.setColor(Qt::darkCyan);
            painter.setBrush(Qt::cyan);
        }
        painter.setPen(pen);

        painter.setOpacity(0.7);
        painter.drawLine(mRoute[i - 1].getX() * 1000.0, mRoute[i - 1].getY() * 1000.0,
                mRoute[i].getX() * 1000.0, mRoute[i].getY() * 1000.0);
        painter.setOpacity(1.0);
    }

    for (int i = 0;i < nPoints;i++) {
        QPointF p = mRoute[i].getPointMm();
        quint32 attr = mRoute[i].getAttributes();

        painter.setTransform(drawTrans);

        if (highQuality) {
            if (isSelected) {
                if ((attr & ATTR_POSITIONING_MASK) == 2) {
                    pen.setColor(Qt::darkGreen);
                    painter.setBrush(Qt::green);
                } else if ((attr & ATTR_HYDRAULIC_FRONT_DOWN) != 0) {
                    pen.setColor(Qt::darkCyan);
                    painter.setBrush(Qt::cyan);
                } else if ((attr & ATTR_HYDRAULIC_FRONT_UP) != 0) {
                    pen.setColor(Qt::darkYellow);
                    painter.setBrush(Qt::green);
                } else {
                    pen.setColor(Qt::darkYellow);
                    painter.setBrush(Qt::yellow);
                }
            } else {
                pen.setColor(Qt::darkGray);
                painter.setBrush(Qt::gray);
            }

            pen.setWidthF(3.0 / mScaleFactor);
            painter.setPen(pen);

            painter.drawEllipse(p, 10.0 / mScaleFactor,
                                10.0 / mScaleFactor);
        } else {
            mapWidget->drawCircleFast(painter, p, 10.0 / mScaleFactor, isSelected ?
                               ((attr & ATTR_POSITIONING_MASK) == 2 ? 2 : ((attr & ATTR_HYDRAULIC_FRONT_DOWN) != 0 ? 3 : ((attr & ATTR_HYDRAULIC_FRONT_UP) != 0 ? 4 : 0))) : 1);
        }

        // Draw highlight for first and last point in active route
        if (isSelected && (i == 0 || i == nPoints-1)) {
            QPointF ptmp;
            ptmp.setX(p.x() - 7.0 / mScaleFactor);
            ptmp.setY(p.y() + 7.0 / mScaleFactor);
            if (i == 0) {
                pen.setColor(Qt::green);
                painter.setBrush(Qt::darkGreen);
            } else {
                pen.setColor(Qt::red);
                painter.setBrush(Qt::darkRed);
            }
            pen.setWidthF(2.0 / mScaleFactor);
            painter.setPen(pen);
            painter.drawEllipse(ptmp, 5.0 / mScaleFactor,
                                5.0 / mScaleFactor);
        }

        // Draw text only for selected route
        if (isSelected && mapWidget->mDrawRouteText) {
            QTime t = QTime::fromMSecsSinceStartOfDay(mRoute[i].getTime());
            txt.sprintf("P: %d %s\n"
                        "%.1f km/h\n"
                        "%02d:%02d:%02d:%03d\n"
                        "A: %08X",
                        i, ((i == 0) ? "- start" : ((i == nPoints-1) ? "- end" : "")),
                        mRoute[i].getSpeed() * 3.6,
                        t.hour(), t.minute(), t.second(), t.msec(),
                        mRoute[i].getAttributes());

            pt_txt.setX(p.x() + 10 / mScaleFactor);
            pt_txt.setY(p.y());
            painter.setTransform(txtTrans);
            pt_txt = drawTrans.map(pt_txt);
            pen.setColor(Qt::black);
            painter.setPen(pen);
            rect_txt.setCoords(pt_txt.x(), pt_txt.y() - 20,
                               pt_txt.x() + 200, pt_txt.y() + 60);
            painter.drawText(rect_txt, txt);
        } else {
 //           txt.sprintf("%d", rn);
            pt_txt.setX(p.x());
            pt_txt.setY(p.y());
            painter.setTransform(txtTrans);
            pt_txt = drawTrans.map(pt_txt);
            pen.setColor(Qt::black);
            painter.setPen(pen);
            rect_txt.setCoords(pt_txt.x() - 20, pt_txt.y() - 20,
                               pt_txt.x() + 20, pt_txt.y() + 20);
            painter.drawText(rect_txt, Qt::AlignCenter, txt);
        }
    }

}

void MapRoute::paintBorder(QPainter &painter, QPen &pen, bool isSelected, double mScaleFactor, QTransform drawTrans)
{
    pen.setWidthF(5.0 / mScaleFactor);
    painter.setTransform(drawTrans);

    QPolygon polygon;
    int nPoints=this->size();
//    for (int i = 1;i < nPoints;i++) {
    for (int i = 0;i < nPoints;i++) {
        double x=mRoute[i].getX() * 1000.0;
        double y=mRoute[i].getY() * 1000.0;

        polygon << QPoint(x, y);
        QPointF p = mRoute[i].getPointMm();
        painter.setPen(QPen(Qt::black, 3.0 / mScaleFactor)); // Set the pen color to blue and width to 2
        painter.drawEllipse(p, 5.0 / mScaleFactor,
                            5.0 / mScaleFactor);

    }
    // Set the brush color for filling the polygon
    painter.setBrush(QBrush(Qt::yellow));
    if (isSelected)
    {
        painter.setOpacity(1.0);
    } else
    {
        painter.setOpacity(0.35);
    }
    // Set the pen color for drawing the border
    painter.setPen(QPen(Qt::darkRed, 5.0 / mScaleFactor, Qt::DashDotLine, Qt::RoundCap)); // Set the pen color to blue and width to 2

    // Draw and fill the polygon
    painter.drawPolygon(polygon);
}

void MapRoute::routeinfo(MapWidget* mapWidget, QPainter &painter,double start_txt,const double txtOffset,const double txt_row_h, int width, QString txt)
{
    int nPoints=this->size();
    if (nPoints > 0) {
        LocPoint prev = this->first();
        double len = 0.0;
        for (int i = 1;i < nPoints;i++) {
            len += prev.getDistanceTo(mRoute[i]);
            prev = mRoute[i];
        }

        txt.sprintf("RP: %d", nPoints);
        painter.drawText(width - txtOffset, start_txt, txt);
        start_txt += txt_row_h;

        txt.sprintf("RLen: %.2f m", len);
        painter.drawText(width - txtOffset, start_txt, txt);
        start_txt += txt_row_h;
    }
}


MapRouteCollection::MapRouteCollection()
{
    mRouteNow = 0;
}

MapRoute MapRouteCollection::getRoute(int ind)
{
    if (ind < 0) {
        return mCollection[mRouteNow];
    } else {
        if (mCollection.size() > ind) {
            return mCollection[ind];
        } else {
            MapRoute tmp;
            return tmp;
        }
    }
}

QList<MapRoute> MapRouteCollection::getRoutes()
{
    return mCollection;
}

void MapRouteCollection::setRoute(const MapRoute &route)
{
    mCollection[mRouteNow] = route;
}


void MapRouteCollection::addRoute(const MapRoute &route)
{
    while (!mCollection.isEmpty() &&
           mCollection.last().isEmpty() &&
           mRouteNow < mCollection.size()) {
        mCollection.removeLast();
    }
    mCollection.append(route);
}

int MapRouteCollection::getRouteNum()
{
    return mCollection.size();
}

void MapRouteCollection::clearRoute()
{
    mCollection[mRouteNow].clear();
}

void MapRouteCollection::clearAllRoutes()
{
    for (int i = 0;i < mCollection.size();i++) {
        mCollection[i].clear();
    }
    mCollection.clear();
}

void MapRouteCollection::setRouteNow(int routeNow)
{
    mRouteNow = routeNow;
    while (mCollection.size() < (mRouteNow + 1)) {
        MapRoute l;
        mCollection.append(l);
    }

    // Clean empty routes
    while (mRouteNow < (mCollection.size() - 1)) {
        if (mCollection.last().isEmpty()) {
            mCollection.removeLast();
        } else {
            break;
        }
    }
}


void MapRouteCollection::clear()
{
    mCollection.clear();
}

void MapRouteCollection::append(MapRoute &maproute)
{
    mCollection.append(maproute);
}


int MapRouteCollection::size()
{
    return mCollection.size();
}

MapRoute& MapRouteCollection::getCurrent()
{
    return mCollection[mRouteNow];
}

QList<MapRoute>::const_iterator MapRouteCollection::begin() const
{
    return mCollection.begin();
}

QList<MapRoute>::const_iterator MapRouteCollection::end() const
{
    return mCollection.end();
}

MapRoute& MapRouteCollection::at(int i)
{
    return mCollection[i];
}

MapRoute& MapRouteCollection::first()
{
    return mCollection.first();
}

MapRoute& MapRouteCollection::last()
{
    return mCollection.last();
}

void MapRouteCollection::prepend(const MapRoute &value)
{
    mCollection.prepend(value);
}

void MapRouteCollection::removeAt(int i)
{
    mCollection.removeAt(i);
}
