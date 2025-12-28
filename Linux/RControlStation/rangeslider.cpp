#include "rangeslider.h"

#include <QSlider>
#include <QStylePainter>
#include <QMouseEvent>

RangeSlider::RangeSlider(QWidget *parent) : QSlider(Qt::Horizontal, parent) {
    setRange(0, 100);
    m_lower = 20;
    m_upper = 80;
}

void RangeSlider::setLowerValue(int value)
{
    if (value >= 0 && value <= m_upper) {
        m_lower = value;
        emit lowerValueChanged(m_lower);
        update();
    }
}

void RangeSlider::setUpperValue(int value)
{
    if (value <= 100 && value >= m_lower) {
        m_upper = value;
        emit upperValueChanged(m_upper);
        update();
    }
}

void RangeSlider::paintEvent(QPaintEvent *event) {
    QStylePainter painter(this);
    // Draw the slider track
    painter.drawLine(10, height()/2, width()-10, height()/2);

    // Draw the lower and upper handles
    int lowerPos = QStyle::sliderPositionFromValue(minimum(), maximum(), m_lower, width()-20);
    int upperPos = QStyle::sliderPositionFromValue(minimum(), maximum(), m_upper, width()-20);
    painter.drawRect(lowerPos-5, height()/2-10, 10, 20);
    painter.drawRect(upperPos-5, height()/2-10, 10, 20);
}

void RangeSlider::mousePressEvent(QMouseEvent *event) {
    int pos = event->pos().x();
    int lowerPos = QStyle::sliderPositionFromValue(minimum(), maximum(), m_lower, width()-20);
    int upperPos = QStyle::sliderPositionFromValue(minimum(), maximum(), m_upper, width()-20);

    if (abs(pos - lowerPos) < abs(pos - upperPos)) {
        m_draggingLower = true;
    } else {
        m_draggingLower = false;
    }
    update();
}

void RangeSlider::mouseMoveEvent(QMouseEvent *event) {
    int pos = event->pos().x();
    int value = QStyle::sliderValueFromPosition(minimum(), maximum(), pos, width()-20);

    if (m_draggingLower) {
        if (value < m_upper) {
            m_lower = value;
            emit lowerValueChanged(m_lower);
        }
    } else {
        if (value > m_lower) {
            m_upper = value;
            emit upperValueChanged(m_upper);
        }
    }
    update();
}

