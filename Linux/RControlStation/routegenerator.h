#include <QCoreApplication>
#include <QFile>
#include <QXmlStreamWriter>
#include <cmath>
#include <vector>
#include "locpoint.h"

class RouteGenerator {
public:
    RouteGenerator(double c_plotLength_m, double c_plotWidth_m, double c_implementLength_m, double c_implementWidth_m,int plots_DrivingDirection, int plots_NonDrivingDirection ,double distancebetweenplots_drivingdirection_m ,double distancebetweenplots_nondrivingdirection_m, LocPoint p1,LocPoint p2,double speed_km__h,double c_turnDiameterX, int c_turnSteps, bool c_flipSide,bool c_isFieldTrial, int c_iTask,bool pieces);
    void generateXmlFile();
    int checkFieldTrial(int x,int y,int task);

private:
    double implementLength_m, implementWidth_m, plotAngle, startX, startY;
    double relPosX, relPosY, plotSizeX, plotSizeY;
    int noPlotsX, noPlotsY, turnSteps;
    double distanceBetweenPlotsX, distanceBetweenPlotsY, turnDiameterX, speed;
    double flipSide;
    bool valid;
    bool isFieldTrial;
    bool pieces;
    int iTask;
    std::string xmlFileName;

    std::vector<std::vector<double>> all_xs, all_ys;
    std::vector<std::vector<int>> all_attributes;

    void generateCoordinates();
    void rotateGeometry2(std::vector<std::pair<double, double>>& matrix, double angle, double aroundx, double aroundy);
    void ydistance(double &ydist, int &direction, int newrow_no,int oldrow_no,double plotsize_m, double distancebetweenplotsy_m);
    std::vector<int> createYArray(int size);

    struct PlotSegment {
        double distance;
        int attribute;
    };
};
