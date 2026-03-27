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

/**
 * @file carinfo.cpp
 * @brief Implementation of the CarInfo class for storing and managing vehicle information
 * 
 * This file contains the implementation of the CarInfo class, which represents a vehicle
 * in the RControlStation application. The class stores all relevant information about a
 * vehicle including its position, dimensions, color, and state data.
 * 
 * CarInfo objects are used to track multiple vehicles simultaneously, with each vehicle
 * having its own unique ID, name, and visual representation on the map.
 */

#include "carinfo.h"
#include <QDebug>

/**
 * @brief Constructs a CarInfo object with default vehicle dimensions
 * 
 * Initializes a vehicle with default dimensions suitable for agricultural vehicles:
 * - Length: 0.8 meters
 * - Width: 0.335 meters  
 * - Corner radius: 0.02 meters
 * 
 * The vehicle is assigned a unique ID and color, and given a default name
 * in the format "Car <id>".
 * 
 * @param id The unique identifier for the vehicle
 * @param color The color used to represent the vehicle on the map
 */
CarInfo::CarInfo(int id, Qt::GlobalColor color)
{
    mId = id;
    mColor = color;
    mName = "";
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
//    mName = QString("Car %1").arg(mId, 0, 'd', 0);
    mName = QString("Car %1").arg(QString::number(mId));

#else
    mName.sprintf("Car %d", mId);
#endif

    mTime = 0;
    mLength = 0.8;
    mWidth = 0.335;
    mCornerRadius = 0.02;
}

/**
 * @brief Gets the unique identifier for the vehicle
 * 
 * @return The vehicle ID
 */
int CarInfo::getId()
{
    return mId;
}

/**
 * @brief Sets the unique identifier for the vehicle
 * 
 * Optionally updates the vehicle name to match the new ID in the format
 * "Car <id>".
 * 
 * @param id The new vehicle ID
 * @param changeName Whether to update the vehicle name to match the new ID
 */
void CarInfo::setId(int id, bool changeName)
{
    mId = id;

    if (changeName) {
        mName = "";
#if (QT_VERSION >= QT_VERSION_CHECK(6, 0, 0))
        mName = QString("Car %1").arg(QString::number(mId));
#else
        mName.sprintf("Car %d", mId);
#endif

    }
}

/**
 * @brief Gets the display name of the vehicle
 * 
 * @return The vehicle name
 */
QString CarInfo::getName() const
{
    return mName;
}

/**
 * @brief Sets the display name of the vehicle
 * 
 * @param name The new name for the vehicle
 */
void CarInfo::setName(QString name)
{
    mName = name;
}

/**
 * @brief Sets the vehicle's location (primary position)
 * 
 * This is the main location used for displaying the vehicle on the map.
 * 
 * @param point The new location for the vehicle
 */
void CarInfo::setLocation(LocPoint &point)
{
    mLocation = point;
}

/**
 * @brief Gets the vehicle's GPS location
 * 
 * @return Reference to the GPS location LocPoint
 */
LocPoint &CarInfo::getLocationGps()
{
    return mLocationGps;
}

/**
 * @brief Sets the vehicle's GPS location
 * 
 * Stores the GPS-derived position of the vehicle, which may be used
 * for positioning when GPS is available.
 * 
 * @param point The GPS location to set
 */
void CarInfo::setLocationGps(LocPoint &point)
{
//    qDebug() << "GPS location: " << point.getX() << ":::" << point.getY();
    mLocationGps = point;
}

/**
 * @brief Gets the color used to represent the vehicle on the map
 * 
 * @return The vehicle's color as a Qt::GlobalColor enum value
 */
Qt::GlobalColor CarInfo::getColor() const
{
    return mColor;
}

/**
 * @brief Sets the color used to represent the vehicle on the map
 * 
 * @param color The color to use for the vehicle (from Qt::GlobalColor enum)
 */
void CarInfo::setColor(Qt::GlobalColor color)
{
    mColor = color;
}

/**
 * @brief Gets the autopilot goal position
 * 
 * This is the target position that the vehicle's autopilot is trying to reach.
 * 
 * @return Reference to the autopilot goal LocPoint
 */
LocPoint &CarInfo::getApGoal()
{
    return mApGoal;
}

/**
 * @brief Sets the autopilot goal position
 * 
 * This is the target position that the vehicle's autopilot should navigate to.
 * 
 * @param apGoal The target position for the autopilot
 */
void CarInfo::setApGoal(const LocPoint &apGoal)
{
    mApGoal = apGoal;
}

/**
 * @brief Gets the vehicle's timestamp
 * 
 * This is typically the GPS time or system time associated with the vehicle's
 * current position.
 * 
 * @return The timestamp in milliseconds
 */
qint32 CarInfo::getTime() const
{
    return mTime;
}

/**
 * @brief Sets the vehicle's timestamp
 * 
 * Updates the timestamp associated with the vehicle's current position.
 * 
 * @param time The timestamp in milliseconds
 */
void CarInfo::setTime(const qint32 &time)
{
    mTime = time;
}

/**
 * @brief Gets the vehicle's length dimension
 * 
 * @return The vehicle length in meters
 */
double CarInfo::getLength() const
{
    return mLength;
}

/**
 * @brief Sets the vehicle's length dimension
 * 
 * @param length The vehicle length in meters
 */
void CarInfo::setLength(double length)
{
    mLength = length;
}

/**
 * @brief Gets the vehicle's width dimension
 * 
 * @return The vehicle width in meters
 */
double CarInfo::getWidth() const
{
    return mWidth;
}

/**
 * @brief Sets the vehicle's width dimension
 * 
 * @param width The vehicle width in meters
 */
void CarInfo::setWidth(double width)
{
    mWidth = width;
}

/**
 * @brief Gets the vehicle's corner radius
 * 
 * This is used for drawing rounded corners when rendering the vehicle on the map.
 * 
 * @return The corner radius in meters
 */
double CarInfo::getCornerRadius() const
{
    return mCornerRadius;
}

/**
 * @brief Sets the vehicle's corner radius
 * 
 * This value is used when rendering the vehicle to create rounded corners
 * instead of sharp edges.
 * 
 * @param cornerRadius The corner radius in meters
 */
void CarInfo::setCornerRadius(double cornerRadius)
{
    mCornerRadius = cornerRadius;
}

/**
 * @brief Gets the vehicle's UWB (Ultra-Wideband) location
 * 
 * This is the position derived from UWB positioning systems when available.
 * 
 * @return Reference to the UWB location LocPoint
 */
LocPoint &CarInfo::getLocationUwb()
{
    return mLocationUwb;
}

/**
 * @brief Sets the vehicle's UWB (Ultra-Wideband) location
 * 
 * Stores the position derived from UWB positioning systems.
 * 
 * @param locationUwb The UWB location to set
 */
void CarInfo::setLocationUwb(const LocPoint &locationUwb)
{
    mLocationUwb = locationUwb;
}

/**
 * @brief Gets the vehicle's primary location
 * 
 * This is the main location used for displaying and tracking the vehicle.
 * 
 * @return Reference to the primary location LocPoint
 */
LocPoint &CarInfo::getLocation()
{
    return mLocation;
}
