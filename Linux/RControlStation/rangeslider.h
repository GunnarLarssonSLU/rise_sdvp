#ifndef RANGESLIDER_H
#define RANGESLIDER_H

#include <QSlider>
#include <QStylePainter>
#include <QMouseEvent>

class RangeSlider : public QSlider {
    Q_OBJECT
public:
    RangeSlider(QWidget *parent = nullptr);
    void paintEvent(QPaintEvent *event)  override;
    void mousePressEvent(QMouseEvent *event)  override;
    void mouseMoveEvent(QMouseEvent *event)  override;
signals:
    void lowerValueChanged(int value);
    void upperValueChanged(int value);

private:
    int m_lower, m_upper;
    bool m_draggingLower;
};


#endif // RANGESLIDER_H
