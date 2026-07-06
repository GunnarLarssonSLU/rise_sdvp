#include "colourutils.h"
#include <QDebug>
#include <cmath>

QColor ColourUtils::hexToQColor(const QString &hexColour)
{
    if (hexColour.isEmpty()) {
        return QColor("#CCCCCC"); // Default grey colour
    }
    
    // Remove # if present
    QString cleanHex = hexColour;
    if (cleanHex.startsWith('#')) {
        cleanHex = cleanHex.mid(1);
    }
    
    // Handle different formats
    if (cleanHex.length() == 3) {
        // Short format like "ABC" -> "AABBCC"
        cleanHex = QString("%1%1%2%2%3%3").arg(cleanHex.at(0)).arg(cleanHex.at(1)).arg(cleanHex.at(2));
    } else if (cleanHex.length() != 6 && cleanHex.length() != 8) {
        // Invalid format, return default
        qWarning() << "Invalid hex colour format:" << hexColour << "using default colour";
        return QColor("#CCCCCC");
    }
    
    // Convert to RGB
    bool ok;
    int red, green, blue, alpha = 255;
    
    if (cleanHex.length() == 6) {
        red = cleanHex.mid(0, 2).toInt(&ok, 16);
        green = cleanHex.mid(2, 2).toInt(&ok, 16);
        blue = cleanHex.mid(4, 2).toInt(&ok, 16);
    } else {
        // 8 characters: RRGGBBAA
        red = cleanHex.mid(0, 2).toInt(&ok, 16);
        green = cleanHex.mid(2, 2).toInt(&ok, 16);
        blue = cleanHex.mid(4, 2).toInt(&ok, 16);
        alpha = cleanHex.mid(6, 2).toInt(&ok, 16);
    }
    
    if (!ok) {
        qWarning() << "Invalid hex colour:" << hexColour << "using default colour";
        return QColor("#CCCCCC");
    }
    
    return QColor(red, green, blue, alpha);
}

QString ColourUtils::qColorToHex(const QColor &colour)
{
    if (!colour.isValid()) {
        return "#CCCCCC"; // Default grey colour
    }
    
    // Return in #RRGGBB format (without alpha for simplicity)
    return colour.name(QColor::HexRgb);
}

QColor ColourUtils::generateColourForAction(int actionId)
{
    // Generate a distinct colour using HSV colour space for better visual distinction
    // Use golden ratio conjugation for good distribution
    int hue = (actionId * 137) % 360;  // Golden ratio (≈137.5°) for good distribution
    
    // Convert HSV to RGB
    QColor colour;
    colour.setHsv(hue, 200, 255);  // Saturate and brighten for vibrant colours
    
    return colour;
}

QColor ColourUtils::getContrastingTextColour(const QColor &backgroundColour)
{
    if (!backgroundColour.isValid()) {
        return Qt::black;
    }
    
    // Calculate luminance to determine if background is light or dark
    // Using the formula from WCAG: 0.2126*R + 0.7152*G + 0.0722*B
    double luminance = 0.2126 * backgroundColour.redF() +
                      0.7152 * backgroundColour.greenF() +
                      0.0722 * backgroundColour.blueF();
    
    // Return black for light backgrounds, white for dark backgrounds
    return luminance > 0.5 ? Qt::black : Qt::white;
}

QColor ColourUtils::lightenColour(const QColor &colour, int percent)
{
    if (!colour.isValid() || percent <= 0) {
        return colour;
    }
    
    // Convert to HSV, increase value (brightness), then convert back
    int hue, saturation, value, alpha;
    colour.getHsv(&hue, &saturation, &value, &alpha);
    
    // Increase value by the specified percentage
    value = qMin(255, value + (255 * percent / 100));
    
    QColor lightenedColour;
    lightenedColour.setHsv(hue, saturation, value, alpha);
    
    return lightenedColour;
}

QColor ColourUtils::darkenColour(const QColor &colour, int percent)
{
    if (!colour.isValid() || percent <= 0) {
        return colour;
    }
    
    // Convert to HSV, decrease value (brightness), then convert back
    int hue, saturation, value, alpha;
    colour.getHsv(&hue, &saturation, &value, &alpha);
    
    // Decrease value by the specified percentage
    value = qMax(0, value - (255 * percent / 100));
    
    QColor darkenedColour;
    darkenedColour.setHsv(hue, saturation, value, alpha);
    
    return darkenedColour;
}