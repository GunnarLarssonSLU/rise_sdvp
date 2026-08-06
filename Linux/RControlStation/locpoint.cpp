/*
    Copyright 2012 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "locpoint.h"
#include <cmath>
#include <QDebug>

LocPoint::LocPoint(double x, double y, double height, double roll, double pitch, double yaw, double speed,
                   double radius, double sigma, QColor color, qint32 time, int id, bool drawLine, quint32 attributes) :
    mX(x),
    mY(y),
    mHeight(height),
    mRoll(roll),
    mPitch(pitch),
    mYaw(yaw),
    mSpeed(speed),
    mRadius(radius),
    mSigma(sigma),
    mInfo(""),  // Initialize mInfo to an empty QString
    mColor(color),
    mTime(time),
    mId(id),
    mDrawLine(drawLine),
    mAttributes(attributes)
{
}


LocPoint::LocPoint(const LocPoint &point)
    : mX(point.mX),
    mY(point.mY),
    mHeight(point.mHeight),
    mRoll(point.mRoll),
    mPitch(point.mPitch),
    mYaw(point.mYaw),
    mSpeed(point.mSpeed),
    mRadius(point.mRadius),
    mSigma(point.mSigma),
    mInfo(point.mInfo),  // QString is implicitly shared, so this is safe
    mColor(point.mColor), // QColor is safe to copy
    mTime(point.mTime),
    mId(point.mId),
    mDrawLine(point.mDrawLine),
    mAttributes(point.mAttributes),
    mControlStates(point.mControlStates) // Copy control states
{
    // No need for additional logic; all members are initialized directly
}


double LocPoint::getX() const
{
    return mX;
}

double LocPoint::getY() const
{
    return mY;
}

double LocPoint::getHeight() const
{
    return mHeight;
}

double LocPoint::getRoll() const
{
    return mRoll;
}

double LocPoint::getPitch() const
{
    return mPitch;
}

double LocPoint::getYaw() const
{
    return mYaw;
}

double LocPoint::getSpeed() const
{
    return mSpeed;
}

QPointF LocPoint::getPoint() const
{
    return QPointF(mX, mY);
}

QPointF LocPoint::getPointMm() const
{
    return QPointF(mX * 1000.0, mY * 1000.0);
}

double LocPoint::getRadius() const
{
    return mRadius;
}

double LocPoint::getSigma() const
{
    return mSigma;
}

void LocPoint::setX(double x)
{
    mX = x;
}

void LocPoint::setY(double y)
{
    mY = y;
}

void LocPoint::setHeight(double height)
{
    mHeight = height;
}

void LocPoint::setXY(double x, double y)
{
    mX = x;
    mY = y;
}

void LocPoint::setTime(const qint32 &time)
{
    mTime = time;
}

void LocPoint::setId(int id)
{
    mId = id;
}

QString LocPoint::getInfo() const
{
    return mInfo;
}

QColor LocPoint::getColor() const
{
    return mColor;
}

qint32 LocPoint::getTime() const
{
    return mTime;
}

int LocPoint::getId() const
{
    return mId;
}



bool LocPoint::getDrawLine() const
{
    return mDrawLine;
}

quint32 LocPoint::getAttributes() const
{
    return mAttributes;
}

double LocPoint::getDistanceTo(const LocPoint &point) const
{
    return sqrt((point.mX - mX) * (point.mX - mX) +
                (point.mY - mY) * (point.mY - mY));
}

double LocPoint::getDistanceTo3d(const LocPoint &point) const
{
    return sqrt((point.mX - mX) * (point.mX - mX) +
                (point.mY - mY) * (point.mY - mY) +
                (point.mHeight - mHeight) * (point.mHeight - mHeight));
}

bool LocPoint::operator ==(const LocPoint &point)
{
    if (    mX == point.mX &&
            mY == point.mY &&
            mHeight == point.mHeight &&
            mRoll == point.mRoll &&
            mPitch == point.mPitch &&
            mYaw == point.mYaw &&
            mSpeed == point.mSpeed &&
            mRadius == point.mRadius &&
            mSigma == point.mSigma &&
            mInfo == point.mInfo &&
            mColor == point.mColor &&
            mTime == point.mTime &&
            mId == point.mId &&
            mDrawLine == point.mDrawLine &&
            mAttributes == point.mAttributes &&
            mControlStates.size() == point.mControlStates.size()) {
        return true;
    } else {
        return false;
    }
}

bool LocPoint::operator !=(const LocPoint &point)
{
    return !(operator==(point));
}

LocPoint& LocPoint::operator=(const LocPoint& point)
{
    if (this != &point) { // Protect against self-assignment
        mX = point.mX;
        mY = point.mY;
        mHeight = point.mHeight;
        mRoll = point.mRoll;
        mPitch = point.mPitch;
        mYaw = point.mYaw;
        mSpeed = point.mSpeed;
        mRadius = point.mRadius;
        mSigma = point.mSigma;
        mInfo = point.mInfo;
        mColor = point.mColor;
        mTime = point.mTime;
        mId = point.mId;
        mDrawLine = point.mDrawLine;
        mAttributes = point.mAttributes;
        mControlStates = point.mControlStates; // Copy control states
    }
    return *this;
}

void LocPoint::setInfo(const QString &info)
{
    mInfo = info;
}

void LocPoint::setRoll(double roll)
{
    mRoll = roll;
}

void LocPoint::setPitch(double pitch)
{
    mPitch = pitch;
}

void LocPoint::setYaw(double alpha)
{
    mYaw = alpha;
}

void LocPoint::setSpeed(double speed)
{
    mSpeed = speed;
}

void LocPoint::setRadius(double radius)
{
    mRadius = radius;
}

void LocPoint::setSigma(double sigma)
{
    mSigma = sigma;
}

void LocPoint::setColor(const QColor &color)
{
    mColor = color;
}

void LocPoint::setDrawLine(bool drawLine)
{
    mDrawLine = drawLine;
}

void LocPoint::setAttributes(quint32 attributes)
{
    mAttributes = attributes;
}

// Control states methods for XML version 2
void LocPoint::setControlStates(const QList<ControlState>& states)
{
    mControlStates = states;
}

QList<ControlState> LocPoint::getControlStates() const
{
    return mControlStates;
}

void LocPoint::addControlState(int controlId, double targetValue)
{
    mControlStates.append({controlId, targetValue});
}

void LocPoint::clearControlStates()
{
    mControlStates.clear();
}

// XML point handling functions
void LocPoint::loadFromXML(LocPoint &point, QXmlStreamReader* stream)
{
    while (stream->readNextStartElement()) {
        QString name = stream->name().toString();

        if (name == "x") {
            point.setX(stream->readElementText().toDouble());
        } else if (name == "y") {
            point.setY(stream->readElementText().toDouble());
        } else if (name == "speed") {
            point.setSpeed(stream->readElementText().toDouble());
        } else if (name == "time") {
            point.setTime(stream->readElementText().toInt());
        } else if (name == "attributes") {
            point.setAttributes(stream->readElementText().toInt());
        } else if (name == "height") {
            point.setHeight(stream->readElementText().toDouble());
        } else if (name == "id") {
            point.setId(stream->readElementText().toInt());
        } else {
            qWarning() << ": Unknown XML element :" << name;
            stream->skipCurrentElement();
        }
    }
}

void LocPoint::saveToXML(QXmlStreamWriter* stream) const
{
    stream->writeStartElement("point");
    stream->writeTextElement("x", QString::number(getX()));
    stream->writeTextElement("y", QString::number(getY()));
    stream->writeTextElement("speed", QString::number(getSpeed()));
    stream->writeTextElement("time", QString::number(getTime()));
    stream->writeTextElement("attributes", QString::number(getAttributes()));
    stream->writeEndElement();
}

// XML version 2 functions (using control states instead of attributes)
void LocPoint::loadFromXMLV2(LocPoint &point, QXmlStreamReader* stream)
{
    while (stream->readNextStartElement()) {
        QString name = stream->name().toString();

        if (name == "x") {
            point.setX(stream->readElementText().toDouble());
        } else if (name == "y") {
            point.setY(stream->readElementText().toDouble());
        } else if (name == "speed") {
            point.setSpeed(stream->readElementText().toDouble());
        } else if (name == "time") {
            point.setTime(stream->readElementText().toInt());
        } else if (name == "height") {
            point.setHeight(stream->readElementText().toDouble());
        } else if (name == "id") {
            point.setId(stream->readElementText().toInt());
        } else if (name == "controlStates") {
            // Load control states
            point.clearControlStates();
            while (stream->readNextStartElement()) {
                QString controlName = stream->name().toString();
                if (controlName == "control") {
                    int controlId = -1;
                    double targetValue = 0.0;
                    
                    while (stream->readNextStartElement()) {
                        QString fieldName = stream->name().toString();
                        if (fieldName == "id") {
                            controlId = stream->readElementText().toInt();
                        } else if (fieldName == "value") {
                            targetValue = stream->readElementText().toDouble();
                        } else {
                            qWarning() << ": Unknown control field :" << fieldName;
                            stream->skipCurrentElement();
                        }
                    }
                    
                    if (controlId >= 0) {
                        point.addControlState(controlId, targetValue);
                    }
                } else {
                    qWarning() << ": Unknown element in controlStates :" << controlName;
                    stream->skipCurrentElement();
                }
            }
        } else {
            qWarning() << ": Unknown XML element :" << name;
            stream->skipCurrentElement();
        }
    }
}

void LocPoint::saveToXMLV2(QXmlStreamWriter* stream) const
{
    stream->writeStartElement("point");
    stream->writeTextElement("x", QString::number(getX()));
    stream->writeTextElement("y", QString::number(getY()));
    stream->writeTextElement("speed", QString::number(getSpeed()));
    stream->writeTextElement("time", QString::number(getTime()));
    
    // Save control states instead of attributes
    if (!getControlStates().isEmpty()) {
        stream->writeStartElement("controlStates");
        for (const ControlState &state : getControlStates()) {
            stream->writeStartElement("control");
            stream->writeTextElement("id", QString::number(state.controlId));
            stream->writeTextElement("value", QString::number(state.targetValue));
            stream->writeEndElement();
        }
        stream->writeEndElement();
    }
    
    stream->writeEndElement();
}

double LocPoint::calculateDistance(const LocPoint& p1, const LocPoint& p2) {
    double dx = p2.getX() - p1.getX();
    double dy = p2.getY() - p1.getY();
    return std::sqrt(dx * dx + dy * dy);
}

double LocPoint::calculateAngle(const LocPoint& p1, const LocPoint& p2) {
    double dx = p2.getX() - p1.getX();
    double dy = p2.getY() - p1.getY();
    return std::atan2(dy, dx); // Angle in radians
}
