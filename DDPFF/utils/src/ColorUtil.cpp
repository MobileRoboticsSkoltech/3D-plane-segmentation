#include "utils/ColorUtil.h"
#include "utils/Statistics.h"
#include "utils/utilities.h"

#include <QtCore/qmath.h>
#include <QFile>

// The ColorUtil is a useful helper when used in the context
// of rendering on a QPainter. Mostly, it is a collection of
// QPen and QBrush objects that are already set up ready for
// use, but the ColorUtil also offers a color interpolation
// interface for a height map and a heat map.

ColorUtil colorUtil;

ColorUtil::ColorUtil()
{
    // A palette for mapping colors for height maps.
    heightMapPalette << QColor("#000077");
    heightMapPalette << QColor("#0000FF");
    heightMapPalette << QColor("#FF0000");
    heightMapPalette << QColor("#FFFF00");

    heatMapPalette << QColor("#FFFFFF");
    heatMapPalette << QColor("#AA0000");

}

// Maps a value v between min and max to a color in the height map palette.
// The height map is a blue-red-yellow space with yellow assigned to the max value.
QColor ColorUtil::getHeightMapColor(double v, double min, double max)
{
	if (min >= max - 0.000001)
		return heightMapPalette[0];

	v = qBound(min, v, max - 0.000001); // We subtract an epsilon to avoid hitting exactly the max value.
	double dblIndx = (heightMapPalette.size()-1) * (v-min)/(max-min);
	int paletteIndex = qFloor(dblIndx);
	double factor = dblIndx - paletteIndex;
	return QColor((1.0-factor)*heightMapPalette[paletteIndex].red() + factor*heightMapPalette[paletteIndex+1].red(),
                  (1.0-factor)*heightMapPalette[paletteIndex].green() + factor*heightMapPalette[paletteIndex+1].green(),
            (1.0-factor)*heightMapPalette[paletteIndex].blue() + factor*heightMapPalette[paletteIndex+1].blue());
}

// Maps a value v between min and max to a color in the heat map palette.
// The heat map is a transparent-to-red space where low values near the minimum will
// be transparent and high values will be opaque and red.
QColor ColorUtil::getHeatMapColor(double v, double min, double max)
{
    if (min >= max - 0.000001)
        return heatMapPalette[0];

    v = qBound(min, v, max - 0.000001); // We subtract an epsilon to avoid hitting exactly the max value.
    double dblIndx = (heatMapPalette.size()-1) * (v-min)/(max-min);
    int paletteIndex = qFloor(dblIndx);
    double factor = dblIndx - paletteIndex;
    return QColor((1.0-factor)*heatMapPalette[paletteIndex].red() + factor*heatMapPalette[paletteIndex+1].red(),
                  (1.0-factor)*heatMapPalette[paletteIndex].green() + factor*heatMapPalette[paletteIndex+1].green(),
                  (1.0-factor)*heatMapPalette[paletteIndex].blue() + factor*heatMapPalette[paletteIndex+1].blue(),
                  factor*255);
}

QColor ColorUtil::sampleUniformColor()
{
    double randh = Statistics::uniformSample();
//	double rands = Statistics::uniformSample();
    double randv = Statistics::uniformSample();

	int h = randh*359;
	int s = 255;
	int v = 128 + randv*127;

	return QColor::fromHsv(h,s,v);
}

RandomColorGenerator::RandomColorGenerator(
        const uint hSpread,
        const uint sSpread,
        const uint vSpread,
        const real_t seed,
        const QColor &initColor) :
    seed(seed),
    hSpread(hSpread),
    sSpread(sSpread),
    vSpread(vSpread),
    initColor(initColor),
    lastColor(initColor),
    hDist(0, hSpread),
    sDist(0, sSpread),
    vDist(0, vSpread)
{

}

RandomColorGenerator::RandomColorGenerator(
        HueSpread hSpread,
        SaturationSpread sSpread,
        ValueSpread vSpread,
        const real_t seed,
        const QColor &initColor) :
    RandomColorGenerator(
        hSpread(),
        sSpread(),
        vSpread(),
        seed,
        initColor
    )
{
    reset();
}

QColor RandomColorGenerator::next()
{
    int lastH = lastColor.hue();

    // Choose hue at 60 degree separation,
    // so that consecutively sampled
    // colors are distinguishable.
    int nextH = (lastH + 60) % 360;

    // Add a small random noise
    int generated = 0;
    while (true) {
        generated = hDist(generator);
        if (nextH + generated > 0 && nextH + generated < 359) {
            break;
        }
    }
    nextH += generated;

    // Randomly choose saturation between 0.5 and 1.0
    int nextS = utils::clip<int>(225 + sDist(generator), 200, 255);

    // Randomly choose value between 0.5 and 1.0
    int nextV = utils::clip<int>(100 + vDist(generator), 50, 150);

    QColor retVal;
    retVal.setHsv(nextH, nextS, nextV);
    retVal = retVal.toRgb(); // change the color spec to RGB!

    lastColor = retVal;
    return retVal;
}

void RandomColorGenerator::reset()
{
    generator.seed(seed);
    hDist.reset();
    sDist.reset();
    vDist.reset();
    lastColor = initColor;
}
