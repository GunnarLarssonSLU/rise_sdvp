#ifndef COLOURUTILS_H
#define COLOURUTILS_H

#include <QColor>
#include <QString>

class ColourUtils
{
public:
    // Convert hex colour string to QColor
    static QColor hexToQColor(const QString &hexColour);
    
    // Convert QColor to hex string
    static QString qColorToHex(const QColor &colour);
    
    // Generate a colour for an action based on its ID
    static QColor generateColourForAction(int actionId);
    
    // Get a contrasting colour (black or white) for text on a given background
    static QColor getContrastingTextColour(const QColor &backgroundColour);
    
    // Lighten a colour by a given percentage
    static QColor lightenColour(const QColor &colour, int percent);
    
    // Darken a colour by a given percentage  
    static QColor darkenColour(const QColor &colour, int percent);
};

#endif // COLOURUTILS_H