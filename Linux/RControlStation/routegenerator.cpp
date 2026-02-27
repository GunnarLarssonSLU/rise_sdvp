#include <QDebug>
#include <QMessageBox>
#include "routegenerator.h"

RouteGenerator::RouteGenerator(double c_plotLength_m, double c_plotWidth_m, double c_implementLength_m, double c_implementWidth_m,int plots_DrivingDirection, int plots_NonDrivingDirection ,double distancebetweenplots_drivingdirection_m ,double distancebetweenplots_nondrivingdirection_m, LocPoint p1,LocPoint p2,double speed_km__h,double c_turnDiameterX, int c_turnSteps, bool c_flipSide)
{
    // Initialize with example values
    implementLength_m = c_implementLength_m;
    implementWidth_m = c_implementWidth_m;

    startX = p2.getX();
    startY = p2.getY();
    relPosX = 0.0;
    relPosY = 0.0;
    noPlotsX = plots_DrivingDirection;
    noPlotsY = plots_NonDrivingDirection;
    distanceBetweenPlotsX = distancebetweenplots_drivingdirection_m;
    distanceBetweenPlotsY = distancebetweenplots_nondrivingdirection_m - c_implementWidth_m;
    plotSizeX=c_plotLength_m; // (LocPoint::calculateDistance(p1, p2)-distanceBetweenPlotsX*(noPlotsX-1))/noPlotsX;
    plotSizeY=c_plotWidth_m;
    plotAngle = LocPoint::calculateAngle(p1, p2);
    flipSide=c_flipSide;
    speed = speed_km__h /3.6;
    xmlFileName = "output.xml";
    turnDiameterX=c_turnDiameterX;
    turnSteps=c_turnSteps;
    valid=false; // valid set by generateCoordinates() if the path is valid
//    /*
    qDebug() << "startX: " << startX;
    qDebug() << "startY: " << startY;
    qDebug() << "relPosX: " << relPosX;
    qDebug() << "relPosY: " << relPosY;
    qDebug() << "noPlotsX: " << noPlotsX;
    qDebug() << "noPlotsY: " << noPlotsY;
    qDebug() << "distanceBetweenPlotsX: " << distanceBetweenPlotsX;
    qDebug() << "distanceBetweenPlotsY: " << distanceBetweenPlotsY;
    qDebug() << "plotSizeX: " << plotSizeX;
    qDebug() << "plotSizeY: " << plotSizeY;
    qDebug() << "plotAngle: " << plotAngle;
    qDebug() << "speed: " << speed;
    qDebug() << "turnDiameterX: " << turnDiameterX;
    qDebug() << "turnSteps: " << turnSteps;
    qDebug() << "implementLength_m: " << implementLength_m;
    qDebug() << "implementWidth_m: " << implementWidth_m; //*/
    generateCoordinates();
};

void RouteGenerator::generateXmlFile() {
    if (valid)  // Only produce a xml file if there is a valid path
    {
        QFile file(QString::fromStdString(xmlFileName));
        if (file.open(QIODevice::WriteOnly)) {
            QXmlStreamWriter xmlWriter(&file);
            xmlWriter.setAutoFormatting(true);
            xmlWriter.writeStartDocument();
            xmlWriter.writeStartElement("routes");
            xmlWriter.writeStartElement("route");

            double t = 0;
            for (size_t i = 0; i < xs.size(); ++i) {
                xmlWriter.writeStartElement("point");
                xmlWriter.writeTextElement("x", QString::number(xs[i]));
                xmlWriter.writeTextElement("y", QString::number(ys[i]));
                xmlWriter.writeTextElement("attributes", QString::number(attributes[i]));
                xmlWriter.writeTextElement("speed", QString::number(speed));
                xmlWriter.writeTextElement("time", QString::number(t));
                t += 2000;
                xmlWriter.writeEndElement(); // point
            }

            xmlWriter.writeEndElement(); // route
            xmlWriter.writeEndElement(); // routes
            xmlWriter.writeEndDocument();
            file.close();
        }
    } else
    {
        QMessageBox msgBox;
        msgBox.setText("No valid path");
        msgBox.exec();
    }
}

void RouteGenerator::generateCoordinates() {
    const int up = 8;
    const int down = 16;

    PlotSegment beforePlot = { distanceBetweenPlotsX - 2 * implementLength_m, up };
    PlotSegment startPlot = { implementLength_m, down };
    PlotSegment endPlot = { plotSizeX, down };
    PlotSegment afterPlot = { implementLength_m, up };

    std::vector<double> arrayDistance = { beforePlot.distance, startPlot.distance, endPlot.distance, afterPlot.distance   };
    std::vector<int> arrayAttribute = { beforePlot.attribute, startPlot.attribute, endPlot.attribute, afterPlot.attribute };
    int action = 2;
    int plotsDoneX = 0, plotsDoneY = 0;
    int directionUD = -1, directionLR = -1;
    bool inHeadRow = true;
    double currentTurnRadiusY = plotSizeY + distanceBetweenPlotsY;
    qDebug() << "currentTurnRadiusY: " << currentTurnRadiusY;
    qDebug() << "plotSizeY: " << plotSizeY;
    qDebug() << "distanceBetweenPlotsY: " << distanceBetweenPlotsY;
    int neededRepeats = plotSizeY / implementWidth_m;
    qDebug() << "neededRepeats*implementWidth_m: " << neededRepeats*implementWidth_m;
    qDebug() << "neededRepeats: " << neededRepeats;
    if (neededRepeats*implementWidth_m < plotSizeY)     // if the plot size and implements to not match return (i.e. there is no valid path)
    {
        return;
    }
    double x = implementLength_m, y = implementWidth_m / 2;
    double oldx=2,oldy=y;
    double lastXInHeader = 0, lastYInHeader = 0;
    int turnCounter = 0;
    int repeats = 0;
    int attribute=0;
    std::vector<int> yorder_array=createYArray(noPlotsY);        // The turn radius of machine might be too big to take the next row well, hence this is a way to make sure that the set turn radius is quite big (and hence more than enough for the machine to manage)
    int yorder_counter=1;
    int yorder=0;
    int oldyorder=0;
    float distancetoImplements=implementLength_m;
    float angleImplements=0;
    double dx=0,dy=0;
    double ddx=0,ddy=0;
    int counter=0;
    qDebug() << "neededRepeats: " << neededRepeats;
    while (repeats < neededRepeats) {
//        qDebug() << "inHeadRow: " << inHeadRow;
//        qDebug() << "action: " << action;
        if (inHeadRow) {
            x += arrayDistance[action-1] * directionLR;           // Move forward a set distance based on the current plotsegment
            attribute = arrayAttribute[action-1];                 // Set the right attribute based on the current plotsegment
            if (action == 3) {                                  // If startPlot move increase the count of done plots in the x (driving) direction
                plotsDoneX++;
            }

            if (plotsDoneX == noPlotsX) {                       // If done all items in the x (driving) direction move to the next plot in the y direction
                inHeadRow = false;                              // no longer in the headrow (as needs to turn)
                turnCounter = 0;                                // restart the counter for driving in the x (driving) direction
                plotsDoneY++;                                   // increase the counter in the y direction

                oldyorder=yorder;                               // keep history
                yorder_counter++;                               // count up the yorder counter (that is used to control the order in which the rows are taken)
                if (yorder_counter==(noPlotsY+1))
                {
                    yorder_counter=1;
                }
                yorder=yorder_array[yorder_counter-1];          // set the correct row based on the yorder counter
//                ydistance(currentTurnRadiusY,directionUD,yorder,oldyorder,plotSizeX,distanceBetweenPlotsY); // set currentTurnRadiusY (how far the machine turn in the y-direction and the direction it is driving in (in the main driving direction)
                ydistance(currentTurnRadiusY,directionUD,yorder,oldyorder,plotSizeY,distanceBetweenPlotsY); // set currentTurnRadiusY (how far the machine turn in the y-direction and the direction it is driving in (in the main driving direction)

                if (plotsDoneY == noPlotsY) {                   // if have done all plots in the y direction (and hence all plots for this repetition) various logics will be used to cover the entire field
                    currentTurnRadiusY -= implementWidth_m;
                    repeats++;
                    plotsDoneY = 0;
                };
                lastXInHeader = x;
                lastYInHeader = y;
            }

            action = (action + 1) % 4;
            if (action == 0) action = 4;
        } else {
            double angle = M_PI * (1.5 - turnCounter / static_cast<double>(turnSteps));
            attribute = up;
            dx = -(turnDiameterX * cos(angle)) * directionLR;
            x = lastXInHeader + dx + implementLength_m  * directionLR;
            dy = currentTurnRadiusY * directionUD * (1 + sin(angle)) / 2;
            qDebug() << "currentTurnRadiusY: " << currentTurnRadiusY;
            qDebug() << "dy: " << dy;
            y = lastYInHeader + dy;
            if (turnCounter == turnSteps) {
                inHeadRow = true;
                attribute = down;
                x=x-implementLength_m * directionLR; // to balance the expression a few lines above
                directionLR = -directionLR;
                action = 3;
                plotsDoneX = 0;
            }
            turnCounter++;
        }

        dy=y-oldy;
        dx=x-oldx;
        angleImplements=atan2(dy,dx);
        oldx=x;
        oldy=y;
        ddx=-cos(angleImplements)*distancetoImplements;
        ddy=0;
//        xs.push_back(x - directionLR * distancetoImplements);
//        ys.push_back(y);
        /*
        qDebug() << "======= " << (counter++) << " ========";
        qDebug() << "x: " << x+ddx;
        qDebug() << "y: " << y+ddy;
        */
        xs.push_back(x+ddx);
        ys.push_back(y+ddy);
        attributes.push_back(attribute);
    }

    if (flipSide)
    {
        for (size_t i = 0; i < xs.size(); ++i) {
            ys[i]=-ys[i];
        }
    }


    // Rotate geometry
    std::vector<std::pair<double, double>> matrix;
    for (size_t i = 0; i < xs.size(); ++i) {
        matrix.emplace_back(xs[i], ys[i]);
    }
    rotateGeometry2(matrix, plotAngle, 0, 0);

    // Update xs and ys after rotation
    for (size_t i = 0; i < matrix.size(); ++i) {
        xs[i] = matrix[i].first + relPosX + startX;
        ys[i] = matrix[i].second + relPosY + startY;
    }
    valid=true;
}

void RouteGenerator::rotateGeometry2(std::vector<std::pair<double, double>>& matrix, double angle, double aroundx, double aroundy) {
    for (auto& point : matrix) {
        double x = point.first - aroundx;
        double y = point.second - aroundy;

        // Apply rotation
        point.first = x * cos(angle) - y * sin(angle) + aroundx;
        point.second = x * sin(angle) + y * cos(angle) + aroundy;
    }
}


template<typename T>
int sign(T val)
{
    return (T(0) <val) - (val < T(0));
}

void RouteGenerator::ydistance(double &ydist, int &direction, int newrow_no,int oldrow_no, double plotsizey_m, double distancebetweenplotsy_m)
{
    int d=newrow_no-oldrow_no;
    ydist=(plotsizey_m+distancebetweenplotsy_m)*abs(d);
    direction=sign(d);
}

std::vector<int> RouteGenerator::createYArray(int size) {
    static const std::map<int, std::vector<int>> arrays = {
        {1, {0}},
        {2, {0, 1}},
        {3, {0, 1, 2}},
        {4, {1, 2, 0, 3}},
        {5, {0, 2, 4, 1, 3}},
        {6, {0, 2, 4, 1, 3, 5}}
    };

    auto it = arrays.find(size);
    if (it != arrays.end()) {
        return it->second; // Returns a std::vector<int>
    } else {
        QMessageBox msgBox;
        msgBox.setText("Unsupported array size!");
        msgBox.exec();
    }
}
