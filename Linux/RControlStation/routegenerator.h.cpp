#include <QCoreApplication>
#include <QFile>
#include <QXmlStreamWriter>
#include <cmath>
#include <vector>

class RouteGenerator {
public:
    RouteGenerator() {
        // Initialize with example values
        implementLength = 1.0;
        implementWidth = 0.5;
        plotAngle = 0.0;
        startX = 0.0;
        startY = 0.0;
        relPosX = 0.0;
        relPosY = 0.0;
        plotSizeX = 10.0;
        plotSizeY = 10.0;
        noPlotsX = 5;
        noPlotsY = 5;
        distanceBetweenPlotsX = 2.0;
        distanceBetweenPlotsY = 2.0;
        turnDiameterX = 2.0;
        turnSteps = 10;
        speed = 10.0;
        xmlFileName = "output.xml";
        generateCoordinates();
    }

    void generateXmlFile() {
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
    }

private:
    double implementLength, implementWidth, plotAngle, startX, startY;
    double relPosX, relPosY, plotSizeX, plotSizeY;
    int noPlotsX, noPlotsY, turnSteps;
    double distanceBetweenPlotsX, distanceBetweenPlotsY, turnDiameterX, speed;
    std::string xmlFileName;

    std::vector<double> xs, ys;
    std::vector<int> attributes;

    void generateCoordinates() {
        const int up = 8;
        const int down = 16;

        PlotSegment beforePlot = { distanceBetweenPlotsX - 2 * implementLength, up };
        PlotSegment startPlot = { implementLength, down };
        PlotSegment endPlot = { plotSizeX, down };
        PlotSegment afterPlot = { implementLength, up };

        std::vector<double> arrayDistance = { beforePlot.distance, startPlot.distance, endPlot.distance, afterPlot.distance };
        std::vector<int> arrayAttribute = { beforePlot.attribute, startPlot.attribute, endPlot.attribute, afterPlot.attribute };

        int action = 2;
        int plotsDoneX = 0, plotsDoneY = 0;
        int directionUD = -1, directionLR = -1;
        bool inHeadRow = true;
        double currentTurnRadiusY = plotSizeY + distanceBetweenPlotsY;
        double neededRepeats = plotSizeY / implementWidth;
        double x = implementLength, y = implementWidth / 2;
        double lastXInHeader = 0, lastYInHeader = 0;
        int turnCounter = 1;
        int repeats = 0;

        while (repeats <= neededRepeats) {
            int attribute;
            if (inHeadRow) {
                x += arrayDistance[action] * directionLR;
                attribute = arrayAttribute[action];
                if (action == 2) {
                    plotsDoneX++;
                }

                if (plotsDoneX == noPlotsX) {
                    inHeadRow = false;
                    turnCounter = 1;
                    plotsDoneY++;

                    if (plotsDoneY == noPlotsY) {
                        currentTurnRadiusY = (plotSizeY + distanceBetweenPlotsY) * (noPlotsY - 1) - implementWidth;
                        repeats++;
                        plotsDoneY = 0;
                        directionUD = 1;
                    } else {
                        currentTurnRadiusY = plotSizeY + distanceBetweenPlotsY;
                        directionUD = -1;
                    }

                    lastXInHeader = x;
                    lastYInHeader = y;
                }

                action = (action + 1) % 4;
                if (action == 0) action = 4;
            } else {
                double angle = M_PI * (1.5 - turnCounter / static_cast<double>(turnSteps));
                int attribute = 8;
                double dx = -turnDiameterX * directionLR * cos(angle);
                x = lastXInHeader + dx;
                double dy = currentTurnRadiusY * directionUD * (1 + sin(angle)) / 2;
                y = lastYInHeader + dy;

                if (turnCounter == turnSteps) {
                    inHeadRow = true;
                    attribute = 16;
                    directionLR = -directionLR;
                    action = 2;
                    plotsDoneX = 0;
                }

                turnCounter++;
            }

            xs.push_back(x - directionLR * 3.4);
            ys.push_back(y);
            attributes.push_back(attribute);
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
    }

    void rotateGeometry2(std::vector<std::pair<double, double>>& matrix, double angle, double aroundx, double aroundy) {
        for (auto& point : matrix) {
            double x = point.first - aroundx;
            double y = point.second - aroundy;

            // Apply rotation
            point.first = x * cos(angle) - y * sin(angle) + aroundx;
            point.second = x * sin(angle) + y * cos(angle) + aroundy;
        }
    }

    struct PlotSegment {
        double distance;
        int attribute;
    };
};

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    RouteGenerator routeGenerator;
    routeGenerator.generateXmlFile();

    return a.exec();
}



/*
#include <QCoreApplication>
#include <QFile>
#include <QXmlStreamWriter>
#include <cmath>
#include <vector>
#include <iostream>

// Function to rotate geometry
void rotateGeometry2(std::vector<std::pair<double, double>>& matrix, double angle, double aroundx, double aroundy) {
    for (auto& point : matrix) {
        double x = point.first - aroundx;
        double y = point.second - aroundy;

        // Apply rotation
        point.first = x * cos(angle) - y * sin(angle) + aroundx;
        point.second = x * sin(angle) + y * cos(angle) + aroundy;
    }
}

struct PlotSegment {
    double distance;
    int attribute;
};

int main(int argc, char *argv[]) {
    QCoreApplication a(argc, argv);

    // Define constants
    const int up = 8;
    const int down = 16;
    double implementLength = 1.0; // Example value
    double implementWidth = 0.5; // Example value
    double plotAngle = 0.0; // Example value
    double startX = 0.0, startY = 0.0; // Example values
    double relPosX = 0.0, relPosY = 0.0; // Example values
    double plotSizeX = 10.0, plotSizeY = 10.0; // Example values
    int noPlotsX = 5, noPlotsY = 5; // Example values
    double distanceBetweenPlotsX = 2.0, distanceBetweenPlotsY = 2.0; // Example values
    double turnDiameterX = 2.0; // Example value
    int turnSteps = 10; // Example value
    double speed = 10.0; // Example value
    std::string xmlFileName = "output_fieldpath.xml";

    // Initialize variables
    PlotSegment beforePlot = { distanceBetweenPlotsX - 2 * implementLength, up };
    PlotSegment startPlot = { implementLength, down };
    PlotSegment endPlot = { plotSizeX, down };
    PlotSegment afterPlot = { implementLength, up };

    std::vector<double> arrayDistance = { beforePlot.distance, startPlot.distance, endPlot.distance, afterPlot.distance };
    std::vector<int> arrayAttribute = { beforePlot.attribute, startPlot.attribute, endPlot.attribute, afterPlot.attribute };

    int action = 2;
    int plotsDoneX = 0, plotsDoneY = 0;
    int directionUD = -1, directionLR = -1;
    bool inHeadRow = true;
    double currentTurnRadiusY = plotSizeY + distanceBetweenPlotsY;
    double neededRepeats = plotSizeY / implementWidth;
    double x = implementLength, y = implementWidth / 2;
    double lastXInHeader = 0, lastYInHeader = 0;
    int turnCounter = 1;
    int repeats = 0;

    std::vector<double> xs, ys;
    std::vector<int> attributes;

    // Main logic loop
    while (repeats <= neededRepeats) {
        int attribute;
        if (inHeadRow) {
            x += arrayDistance[action] * directionLR;
            attribute = arrayAttribute[action];
            if (action == 2) {
                plotsDoneX++;
            }

            if (plotsDoneX == noPlotsX) {
                inHeadRow = false;
                turnCounter = 1;
                plotsDoneY++;

                if (plotsDoneY == noPlotsY) {
                    currentTurnRadiusY = (plotSizeY + distanceBetweenPlotsY) * (noPlotsY - 1) - implementWidth;
                    repeats++;
                    plotsDoneY = 0;
                    directionUD = 1;
                } else {
                    currentTurnRadiusY = plotSizeY + distanceBetweenPlotsY;
                    directionUD = -1;
                }

                lastXInHeader = x;
                lastYInHeader = y;
            }

            action = (action + 1) % 4;
            if (action == 0) action = 4; // Ensure action is between 1 and 4
        } else {
            double angle = M_PI * (1.5 - turnCounter / static_cast<double>(turnSteps));
            int attribute = 8;
            double dx = -turnDiameterX * directionLR * cos(angle);
            x = lastXInHeader + dx;
            double dy = currentTurnRadiusY * directionUD * (1 + sin(angle)) / 2;
            y = lastYInHeader + dy;

            if (turnCounter == turnSteps) {
                inHeadRow = true;
                attribute = 16;
                directionLR = -directionLR;
                action = 2;
                plotsDoneX = 0;
            }

            turnCounter++;
        }

        xs.push_back(x - directionLR * 3.4);
        ys.push_back(y);
        attributes.push_back(attribute);
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

    // Create XML file using QXmlStreamWriter
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

    return 0;
}
*/
