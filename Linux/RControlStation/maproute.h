#ifndef MAPROUTE_H
#define MAPROUTE_H

#include <QList>
#include <QXmlStreamWriter>
#include <QPainter>
#include <QPen>
#include <QPalette>
#include <QTransform>

#include "locpoint.h"
#include "mapwidget.h"

class MapWidget;

class MapRoute
{

public:
    MapRoute();
    MapRoute(const MapRoute &other);             // Copy constructor
    MapRoute(MapRoute &&other) noexcept;         // Move constructor
    MapRoute &operator=(const MapRoute &other); // Copy assignment
    MapRoute &operator=(MapRoute &&other) noexcept; // Move assignment
    ~MapRoute();                                // Destructor


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
    void insert(int i,LocPoint &value);
    void 	prepend(const LocPoint &value);
    void 	removeAt(int i);
    void 	removeLast();
    LocPoint& operator[](int i);
    LocPoint& at(int i);
    QList<LocPoint> mRoute;
    QList<LocPoint> mInfoTrace;

    void saveXMLRoute(QXmlStreamWriter* stream,bool withId, int i);
    double getArea();
    double getLength(bool bForAnalysis);
    //    void paint(MapWidget* mapWidget, QPainter &painter, QPen &pen, bool isSelected, double mScaleFactor, QTransform drawtrans, QString txt, QPointF pt_txt, QRectF rect_txt, QTransform txtTrans, bool highQuality = false);
    void paintPath(QPainter &painter, QPen &pen, bool isSelected, double mScaleFactor, QTransform drawtrans, QString txt, QPointF pt_txt, QRectF rect_txt, QTransform txtTrans, bool mDrawRouteText, bool highQuality = false);
    void paintBorder(QPainter &painter, QPen &pen, bool isSelected, double mScaleFactor, QTransform drawtrans);
    void paintLine(int i, bool isSelected, bool isSAnalyzed, QPainter &painter, QPen &pen, Qt::GlobalColor defaultDarkColor, Qt::GlobalColor defaultColor);
    void paintPoint(QPointF p, quint32 attr, bool isSelected, bool isSAnalyzed, QPainter &painter, QPen &pen, double mScaleFactor, QTransform drawTrans, bool highQuality);
    void paintInfoText(bool mDrawRouteText, int i, QPointF p, bool isSelected, QPainter &painter, QPen &pen, double mScaleFactor, QTransform drawTrans, QString txt, QPointF pt_txt, QRectF rect_txt, QTransform txtTrans);

    void routeinfo(QPainter &painter,double start_txt,const double txtOffset,const double txt_row_h, int width, QString txt);
    void setIsBorder(bool border);
    bool getIsBorder() const;
    int iAnalysisStart,iAnalysisEnd;
    bool bAnalysisShowable;
    void setAnalysisFocus();
    void updateAnalysis();
    void drawCircleFast(QPainter &painter, QPointF center, double radius, int type = 0);
    void transform(double moveX,double moveY,double rotate);

private:
    bool isBorder;
    static QList<QPixmap> mPixmaps;
    static const QList<QPixmap>& getPixmaps();
    static void initPixmaps();
};

#endif // MAPROUTE_H
