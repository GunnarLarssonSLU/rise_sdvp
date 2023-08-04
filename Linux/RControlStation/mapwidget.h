/*
    Copyright 2012 - 2019 Benjamin Vedder	benjamin@vedder.se

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

#ifndef MAPWIDGET_H
#define MAPWIDGET_H

#include <QWidget>
#include <QPainter>
#include <QPaintEvent>
#include <QBrush>
#include <QFont>
#include <QPen>
#include <QPalette>
#include <QList>
#include <QInputDialog>
#include <QTimer>
#include <QPinchGesture>
#include <QImage>
#include <QTransform>
#include <QXmlStreamWriter>
#include <QXmlStreamReader>

#include "locpoint.h"
#include "carinfo.h"
#include "copterinfo.h"
#include "perspectivepixmap.h"
#include "osmclient.h"

class MapModule
{
public:
    virtual void processPaint(QPainter &painter, int width, int height, bool highQuality,
                              QTransform drawTrans, QTransform txtTrans, double scale) = 0;
    virtual bool processMouse(bool isPress, bool isRelease, bool isMove, bool isWheel,
                              QPoint widgetPos, LocPoint mapPos, double wheelAngleDelta,
                              bool ctrl, bool shift, bool ctrlShift,
                              bool leftButton, bool rightButton, double scale) = 0;
};

class MapRoute;

class MapWidget : public QWidget
{
    Q_OBJECT

    friend class MapRoute;
public:
    typedef enum {
        InteractionModeDefault,
        InteractionModeCtrlDown,
        InteractionModeShiftDown,
        InteractionModeCtrlShiftDown
    } InteractionMode;

    explicit MapWidget(QWidget *parent = 0);
    CarInfo* getCarInfo(int car);
    void setFollowCar(int car);
    void setTraceCar(int car);
    void setSelectedCar(int car);
    void addCar(const CarInfo &car);
    bool removeCar(int carId);
    void clearCars();
    LocPoint* getAnchor(int id);
    void addAnchor(const LocPoint &anchor);
    bool removeAnchor(int id);
    void clearAnchors();
    QList<LocPoint> getAnchors();
    void setScaleFactor(double scale);
    double getScaleFactor();
    void setRotation(double rotation);
    void setXOffset(double offset);
    void setYOffset(double offset);
    void moveView(double px, double py);
    void clearTrace();
    void addRoutePoint(double px, double py, double speed = 0.0, qint32 time = 0);

    MapRoute getRoute(int ind = -1);
    QList<MapRoute> getRoutes();
    void setRoute(const MapRoute &route);
    void addRoute(const MapRoute &route);
    int getRouteNow() const;
    void setRouteNow(int routeNow);
    int getRouteNum();
    void clearRoute();
    void clearAllRoutes();

    MapRoute getField(int ind = -1);
    QList<MapRoute> getFields();
    void setField(const MapRoute &field);
    void addField(const MapRoute &field);
    int getFieldNow() const;
    void setFieldNow(int fieldNow);
    int getFieldNum();
    void clearField();
    void clearAllFields();

    void setRoutePointSpeed(double speed);
    void addInfoPoint(LocPoint &info, bool updateMap = true);
    void clearInfoTrace();
    void clearAllInfoTraces();
    void addPerspectivePixmap(PerspectivePixmap map);
    void clearPerspectivePixmaps();
    QPoint getMousePosRelative();
    void setAntialiasDrawings(bool antialias);
    void setAntialiasOsm(bool antialias);
    bool getDrawOpenStreetmap() const;
    void setDrawOpenStreetmap(bool drawOpenStreetmap);
    void setEnuRef(double lat, double lon, double height);
    void getEnuRef(double *llh);
    double getOsmRes() const;
    void setOsmRes(double osmRes);
    double getInfoTraceTextZoom() const;
    void setInfoTraceTextZoom(double infoTraceTextZoom);
    OsmClient *osmClient();
    int getInfoTraceNum();
    int getInfoPointsInTrace(int trace);
    int setNextEmptyOrCreateNewInfoTrace();
    void setAnchorMode(bool anchorMode);
    bool getAnchorMode();
    void setAnchorId(int id);
    void setAnchorHeight(double height);
    void removeLastRoutePoint();
    void zoomInOnRoute(int id, double margins, double wWidth = -1, double wHeight = -1);

    int getOsmMaxZoomLevel() const;
    void setOsmMaxZoomLevel(int osmMaxZoomLevel);

    int getOsmZoomLevel() const;

    bool getDrawGrid() const;
    void setDrawGrid(bool drawGrid);

    bool getDrawOsmStats() const;
    void setDrawOsmStats(bool drawOsmStats);

    qint32 getRoutePointTime() const;
    void setRoutePointTime(const qint32 &routePointTime);

    double getTraceMinSpaceCar() const;
    void setTraceMinSpaceCar(double traceMinSpaceCar);

    double getTraceMinSpaceGps() const;
    void setTraceMinSpaceGps(double traceMinSpaceGps);

    int getInfoTraceNow() const;
    void setInfoTraceNow(int infoTraceNow);

    /*
    void printPdf(QString path, int width = 0, int height = 0);
    void printPng(QString path, int width = 0, int height = 0);
*/

    bool getDrawRouteText() const;
    void setDrawRouteText(bool drawRouteText);

    bool getDrawUwbTrace() const;
    void setDrawUwbTrace(bool drawUwbTrace);

    void setLastCameraImage(const QImage &lastCameraImage);

    double getCameraImageWidth() const;
    void setCameraImageWidth(double cameraImageWidth);

    double getCameraImageOpacity() const;
    void setCameraImageOpacity(double cameraImageOpacity);

    MapWidget::InteractionMode getInteractionMode() const;
    void setInteractionMode(const MapWidget::InteractionMode &controlMode);

    void addMapModule(MapModule *m);
    void removeMapModule(MapModule *m);
    void removeMapModuleLast();

    quint32 getRoutePointAttributes() const;
    void setRoutePointAttributes(const quint32 &routePointAttributes);

    bool loadXMLRoute(QXmlStreamReader* stream,bool isBorder);
    void saveXMLRoutes(QXmlStreamWriter* stream,bool withId);

    std::array<double, 4> findExtremeValuesFieldBorders();
signals:
    void scaleChanged(double newScale);
    void offsetChanged(double newXOffset, double newYOffset);
    void posSet(quint8 id, LocPoint pos);
    void routePointAdded(LocPoint pos);
    void lastRoutePointRemoved(LocPoint pos);
    void infoTraceChanged(int traceNow);

    void mouseClickedInField(const int field);

private slots:
    void tileReady(OsmTile tile);
    void errorGetTile(QString reason);
    void timerSlot();

protected:
    void paintEvent(QPaintEvent *event) override;
    void mouseMoveEvent (QMouseEvent *e) override;
    void mousePressEvent(QMouseEvent *e) override;
    void mouseReleaseEvent(QMouseEvent *e) override;
    void wheelEvent(QWheelEvent *e) override;
    bool event(QEvent *event) override;

private:
    QList<CarInfo> mCarInfo;
    QVector<LocPoint> mCarTrace;
    QVector<LocPoint> mCarTraceGps;
    QVector<LocPoint> mCarTraceUwb;
    QList<LocPoint> mAnchors;
    QList<MapRoute> mFields;
    QList<MapRoute> mRoutes;
    QList<MapRoute> mFieldborders;
    QList<QList<LocPoint>> mInfoTraces;
    //    QList<MapRoute> mInfoTraces;
    QList<LocPoint> mVisibleInfoTracePoints;
    QList<PerspectivePixmap> mPerspectivePixmaps;
    double mRoutePointSpeed;
    qint32 mRoutePointTime;
    quint32 mRoutePointAttributes;
    qint32 mAnchorId;
    double mAnchorHeight;
    double mScaleFactor;
    double mRotation;
    double mXOffset;
    double mYOffset;
    int mMouseLastX;
    int mMouseLastY;
    int mFollowCar;
    int mTraceCar;
    int mSelectedCar;
    double xRealPos;
    double yRealPos;
    bool mAntialiasDrawings;
    bool mAntialiasOsm;
    double mOsmRes;
    double mInfoTraceTextZoom;
    OsmClient *mOsm;
    int mOsmZoomLevel;
    int mOsmMaxZoomLevel;
    bool mDrawOpenStreetmap;
    bool mDrawOsmStats;
    double mRefLat;
    double mRefLon;
    double mRefHeight;
    LocPoint mClosestInfo;
    bool mDrawGrid;
    int mRoutePointSelected;
    int mAnchorSelected;
    int mRouteNow;
    int mFieldNow;
    int mInfoTraceNow;
    double mTraceMinSpaceCar;
    double mTraceMinSpaceGps;
    QList<QPixmap> mPixmaps;
    bool mAnchorMode;
    bool mDrawRouteText;
    bool mDrawUwbTrace;
    QImage mLastCameraImage;
    double mCameraImageWidth;
    double mCameraImageOpacity;
    InteractionMode mInteractionMode;
    QTimer *mTimer;
    QVector<MapModule*> mMapModules;

    void updateClosestInfoPoint();
    int drawInfoPoints(QPainter &painter, const QList<LocPoint> &pts,
                        QTransform drawTrans, QTransform txtTrans,
                       double xStart, double xEnd, double yStart, double yEnd,
                       double min_dist);
    int getClosestPoint(LocPoint p, QList<LocPoint> route, double &dist);
    void drawCircleFast(QPainter &painter, QPointF center, double radius, int type = 0);

    void paint(QPainter &painter, int width, int height, bool highQuality = false);
    void updateTraces();
    void paintTiles(QPainter &painter,const double cx, const double cy, const double view_w, const double view_h, bool highQuality);
    void paintGrid(QPainter &painter,QPen &pen, QTransform drawTrans, QTransform txtTrans, int width,int height, double stepGrid,QString txt, QPointF pt_txt,const QColor textColor, const QColor zeroAxisColor,const QColor firstAxisColor,const QColor secondAxisColor,const double zeroAxisWidth);
    void paintCar(CarInfo &carInfo,QPainter &painter,QPen &pen, QTransform drawTrans, QTransform txtTrans,QString &txt, QPointF &pt_txt,const QColor &textColor,QRectF &rect_txt);
    void paintAnchor(LocPoint &anchor,QPainter &painter,QPen &pen, QTransform drawTrans, QTransform txtTrans,QString &txt, QPointF &pt_txt,const QColor &textColor,QRectF &rect_txt);
    void paintTrace(QList<LocPoint> &itNow,QPainter &painter,QPen &pen, bool isActive, QTransform drawTrans, QTransform txtTrans,const double cx, const double cy, const double view_w, const double view_h,int& info_segments, int& info_points);
    void paintCarTraces(QPainter &painter,QPen &pen, QTransform drawTrans);
    void paintCamera(QPainter &painter,QTransform txtTrans, int width,double &start_txt);
    void paintUnitZoomGeneralinfo(QPainter &painter,QFont font, QTransform txtTrans, int width, double stepGrid,QString& txt, const QColor textColor, double& start_txt,const double txtOffset,const double txt_row_h,    int& info_segments, int& info_points);
    void paintClosestPoint(QPainter &painter,QPen &pen,QTransform drawTrans, QTransform txtTrans, QPointF& pt_txt,QRectF &rect_txt );
};

class MapRoute
{

public:
//    MapRoute(QList<LocPoint> route, QList<LocPoint> infotrace);
    void append(LocPoint point);
    void append(const QList<LocPoint> &points);
    void append(const MapRoute &mr);
    int size();
    bool isEmpty() const;
    void clear();
    QList<LocPoint>::const_iterator begin() const;
    QList<LocPoint>::const_iterator end() const;
    LocPoint&	first();
    LocPoint&	last();
    void 	prepend(const LocPoint &value);
    void 	removeAt(int i);
    void 	removeLast();
    LocPoint &	operator[](int i);
    QList<LocPoint> mRoute;
    QList<LocPoint> mInfoTrace;

    double getArea();

    void paint(MapWidget* mapWidget, QPainter &painter, QPen &pen, bool isSelected, double mScaleFactor, QTransform drawtrans, QString txt, QPointF pt_txt, QRectF rect_txt, QTransform txtTrans, bool highQuality = false);
    void paintPath(MapWidget* mapWidget, QPainter &painter, QPen &pen, bool isSelected, double mScaleFactor, QTransform drawtrans, QString txt, QPointF pt_txt, QRectF rect_txt, QTransform txtTrans, bool highQuality = false);
    void paintBorder(QPainter &painter, QPen &pen, bool isSelected, double mScaleFactor, QTransform drawtrans);
    void routeinfo(MapWidget* mapWidget, QPainter &painter,double start_txt,const double txtOffset,const double txt_row_h, int width, QString txt);
    void setIsBorder(bool border);
    bool getIsBorder() const;

private:
    bool isBorder;

};


#endif // MAPWIDGET_H
