#ifndef MAPROUTECOLLECTION_H
#define MAPROUTECOLLECTION_H

#include "maproute.h"

class MapRoute;

class MapRouteCollection
{
public:
    MapRouteCollection();
    MapRoute& getRoute(int ind = -1);
    QList<MapRoute>& getRoutes();
    void setRoute(const MapRoute &route);
    void addRoute(const MapRoute &route);
    int getRouteNow() const;
    void setRouteNow(int routeNow);
    int getRouteNum();
    void clearRoute();
    void clearAllRoutes();
    int mRouteNow;

    void clear();
    void append(MapRoute &maproute);
    int size();
    MapRoute& getCurrent();
    MapRoute* getCurrentP();
    QList<MapRoute>::const_iterator begin() const;
    QList<MapRoute>::const_iterator end() const;
    MapRoute&	first();
    MapRoute&	last();
    void 	prepend(const MapRoute &value);
    void 	removeAt(int i);

    MapRoute& at(int i);
    QList<MapRoute> mCollection;
private:

};

#endif // MAPROUTECOLLECTION_H
