#ifndef COMPASSWIDGET_H
#define COMPASSWIDGET_H

#include <QWidget>
#include <QPainter>
#include <QtMath>

class CompassWidget : public QWidget
{
    Q_OBJECT
public:
    explicit CompassWidget(QWidget *parent);

    void setHeading(qreal newHeading);
protected:
    void paintEvent(QPaintEvent *event) override;
private:
    qreal heading; // Current compass heading in degrees
};

#endif // COMPASSWIDGET_H
