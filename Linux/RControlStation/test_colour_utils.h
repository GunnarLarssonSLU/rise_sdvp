#ifndef TEST_COLOUR_UTILS_H
#define TEST_COLOUR_UTILS_H

#include <QObject>
#include <QtTest>

class TestColourUtils : public QObject
{
    Q_OBJECT

private slots:
    void testHexToQColor();
    void testQColorToHex();
    void testGenerateColourForAction();
    void testContrastingTextColour();
    void testLightenDarken();
};

#endif // TEST_COLOUR_UTILS_H