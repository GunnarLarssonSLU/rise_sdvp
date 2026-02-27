#include <QCoreApplication>
#include <QFile>
#include <QXmlStreamWriter>
#include <cmath>
#include <vector>
#include "locpoint.h"

class RouteGenerator {
public:
    RouteGenerator(double c_plotLength_m, double c_plotWidth_m, double c_implementLength_m, double c_implementWidth_m,int plots_DrivingDirection, int plots_NonDrivingDirection ,double distancebetweenplots_drivingdirection_m ,double distancebetweenplots_nondrivingdirection_m, LocPoint p1,LocPoint p2,double speed_km__h,double c_turnDiameterX, int c_turnSteps, bool c_flipSide);
    void generateXmlFile();

private:
    double implementLength_m, implementWidth_m, plotAngle, startX, startY;
    double relPosX, relPosY, plotSizeX, plotSizeY;
    int noPlotsX, noPlotsY, turnSteps;
    double distanceBetweenPlotsX, distanceBetweenPlotsY, turnDiameterX, speed;
    double flipSide;
    bool valid;
    std::string xmlFileName;

    std::vector<double> xs, ys;
    std::vector<int> attributes;

    void generateCoordinates();
    void rotateGeometry2(std::vector<std::pair<double, double>>& matrix, double angle, double aroundx, double aroundy);
    void ydistance(double &ydist, int &direction, int newrow_no,int oldrow_no,double plotsize_m, double distancebetweenplotsy_m);
    std::vector<int> createYArray(int size);

    struct PlotSegment {
        double distance;
        int attribute;
    };
};
