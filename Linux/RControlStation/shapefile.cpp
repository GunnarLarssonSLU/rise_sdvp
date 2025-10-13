#include <iostream>
#include "shapefile.h"
#include "utility.h"

ShapeFile::ShapeFile() {}

bool ShapeFile::load(QString fileName,double illh[3],MapWidget* mapWidget)
{
    QByteArray utf8Path = fileName.toUtf8();     // store the QByteArray
    const char* shapefile = utf8Path.constData(); // safe pointer

    qDebug() << "loading shapefile: " << fileName;

    // 1️⃣ Register all OGR drivers
    GDALAllRegister();

    // 2️⃣ Open the shapefile (read-only)
    GDALDataset *poDS = (GDALDataset*) GDALOpenEx(shapefile, GDAL_OF_VECTOR, NULL, NULL, NULL);
    if (poDS == nullptr) {
        std::cerr << "Failed to open shapefile: " << shapefile << std::endl;
        return false;
    }

    // 3️⃣ Get the first layer
    OGRLayer* poLayer = poDS->GetLayer(0);
    if (poLayer == nullptr) {
        std::cerr << "Could not get layer from shapefile" << std::endl;
        GDALClose(poDS);
        return false;
    }

    // 4️⃣ Loop through features
    OGRFeature* poFeature;
    poLayer->ResetReading();
    while ((poFeature = poLayer->GetNextFeature()) != nullptr) {
        OGRGeometry* geom = poFeature->GetGeometryRef();
        if (geom != nullptr) {
            OGRwkbGeometryType geomType = wkbFlatten(geom->getGeometryType());
            if (geomType == wkbMultiPolygon) {
                OGRMultiPolygon* multiPoly = geom->toMultiPolygon();
                for (int i = 0; i < multiPoly->getNumGeometries(); i++) {
                    OGRPolygon* poly = multiPoly->getGeometryRef(i)->toPolygon();
                    extractPolygonPoints(poly, mapWidget, illh);
                }
            }
            else if (geomType == wkbPolygon) {
                // Handle Polygon (single polygon)
                OGRPolygon* poly = geom->toPolygon();
                extractPolygonPoints(poly, mapWidget, illh);
            }
            else {
                std::cerr << "Unsupported geometry type: " << OGRGeometryTypeToName(geom->getGeometryType()) << std::endl;
            }
        }
        OGRFeature::DestroyFeature(poFeature);
    }

    // 5️⃣ Close dataset
    GDALClose(poDS);
    return true;
}

void ShapeFile::extractPolygonPoints(OGRPolygon* poly, MapWidget* mapWidget, double* illh) {
    OGRLinearRing* outer = poly->getExteriorRing();
    MapRoute MR;
    if (outer) {
        int nPoints = outer->getNumPoints();
        for (int j = 0; j < nPoints; j++) {
            double llh[3];
            double xyh[3];
            llh[0] = outer->getY(j);
            llh[1] = outer->getX(j);
            llh[2] = 0;
            utility::llhToEnu(illh, llh, xyh);
            LocPoint lp(xyh[0], xyh[1]);
            MR.append(lp);
        }
    }
     mapWidget->addField(MR);
}
