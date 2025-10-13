#ifndef SHAPEFILE_H
#define SHAPEFILE_H

#include <QString>
#include <ogrsf_frmts.h>
#include "mapwidget.h"
class ShapeFile

{
public:
    ShapeFile();
    bool load(QString fileName,double illh[3],MapWidget* mapWidget);
private:
    void extractPolygonPoints(OGRPolygon* poly, MapWidget* mapWidget, double* illh);
};

#endif // SHAPEFILE_H
