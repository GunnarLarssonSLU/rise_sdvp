#include "CompassWidget.h"

#ifndef COMPASSWIDGET_CPP
#define COMPASSWIDGET_CPP

CompassWidget::CompassWidget(QWidget *parent) : QWidget(parent), heading(0)
{

};

void CompassWidget::setHeading(qreal newHeading)
{
    heading = newHeading;
    update(); // Schedule a repaint
};

void CompassWidget::paintEvent(QPaintEvent *event)
{
    Q_UNUSED(event);

    int side = qMin(width(), height());
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    // Set the center and radius of the compass circle
    int centerX = width() / 2;
    int centerY = height() / 2;
    int radius = side / 2;

    // Draw the compass circle
    painter.setPen(Qt::black);
    painter.drawEllipse(centerX - radius, centerY - radius, side, side);

    // Draw the needle indicating the heading
    painter.setPen(Qt::red);
    painter.drawLine(centerX, centerY,
                     centerX + radius * qSin(qDegreesToRadians(heading)),
                     centerY - radius * qCos(qDegreesToRadians(heading)));
};

#endif
