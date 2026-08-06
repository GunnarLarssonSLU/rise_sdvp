#include "test_colour_utils.h"
#include "colourutils.h"
#include <QDebug>

void TestColourUtils::testHexToQColor()
{
    // Test valid hex colours
    QColor red = ColourUtils::hexToQColor("#FF0000");
    QVERIFY(red == QColor(255, 0, 0));
    
    QColor green = ColourUtils::hexToQColor("#00FF00");
    QVERIFY(green == QColor(0, 255, 0));
    
    QColor blue = ColourUtils::hexToQColor("#0000FF");
    QVERIFY(blue == QColor(0, 0, 255));
    
    // Test short format
    QColor yellow = ColourUtils::hexToQColor("#FF0");
    QVERIFY(yellow == QColor(255, 255, 0));
    
    // Test without #
    QColor purple = ColourUtils::hexToQColor("9933CC");
    QVERIFY(purple == QColor(153, 51, 204));
    
    // Test empty string (should return default)
    QColor defaultColour = ColourUtils::hexToQColor("");
    QVERIFY(defaultColour == QColor("#CCCCCC"));
    
    // Test invalid format (should return default)
    QColor invalidColour = ColourUtils::hexToQColor("INVALID");
    QVERIFY(invalidColour == QColor("#CCCCCC"));
}

void TestColourUtils::testQColorToHex()
{
    QColor red(255, 0, 0);
    QString hex = ColourUtils::qColorToHex(red);
    QVERIFY(hex == "#ff0000");
    
    // Test invalid colour (should return default)
    QColor invalid;
    QString defaultHex = ColourUtils::qColorToHex(invalid);
    QVERIFY(defaultHex == "#cccccc");
}

void TestColourUtils::testGenerateColourForAction()
{
    // Test that different action IDs generate different colours
    QColor colour1 = ColourUtils::generateColourForAction(0);
    QColor colour2 = ColourUtils::generateColourForAction(1);
    QColor colour3 = ColourUtils::generateColourForAction(2);
    
    QVERIFY(colour1 != colour2);
    QVERIFY(colour2 != colour3);
    QVERIFY(colour1 != colour3);
    
    // Test that the same ID always generates the same colour
    QColor colour1Again = ColourUtils::generateColourForAction(0);
    QVERIFY(colour1 == colour1Again);
    
    // Test that generated colours are valid
    QVERIFY(colour1.isValid());
    QVERIFY(colour2.isValid());
    QVERIFY(colour3.isValid());
}

void TestColourUtils::testContrastingTextColour()
{
    // Test dark background should get white text
    QColor darkBg(30, 30, 30);
    QColor textColour = ColourUtils::getContrastingTextColour(darkBg);
    QVERIFY(textColour == Qt::white);
    
    // Test light background should get black text
    QColor lightBg(230, 230, 230);
    textColour = ColourUtils::getContrastingTextColour(lightBg);
    QVERIFY(textColour == Qt::black);
    
    // Test invalid background (should return black)
    QColor invalidBg;
    textColour = ColourUtils::getContrastingTextColour(invalidBg);
    QVERIFY(textColour == Qt::black);
}

void TestColourUtils::testLightenDarken()
{
    QColor baseColour(128, 128, 128);
    
    // Test lighten
    QColor lightened = ColourUtils::lightenColour(baseColour, 20);
    QVERIFY(lightened != baseColour);
    QVERIFY(lightened.lightness() > baseColour.lightness());
    
    // Test darken
    QColor darkened = ColourUtils::darkenColour(baseColour, 20);
    QVERIFY(darkened != baseColour);
    QVERIFY(darkened.lightness() < baseColour.lightness());
    
    // Test with invalid colour
    QColor invalid;
    QColor lightenedInvalid = ColourUtils::lightenColour(invalid, 20);
    QVERIFY(lightenedInvalid == invalid);
    
    QColor darkenedInvalid = ColourUtils::darkenColour(invalid, 20);
    QVERIFY(darkenedInvalid == invalid);
    
    // Test with 0 percent (should return same colour)
    QColor sameLightened = ColourUtils::lightenColour(baseColour, 0);
    QVERIFY(sameLightened == baseColour);
    
    QColor sameDarkened = ColourUtils::darkenColour(baseColour, 0);
    QVERIFY(sameDarkened == baseColour);
}