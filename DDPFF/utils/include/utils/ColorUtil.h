#ifndef COLORUTIL_H_
#define COLORUTIL_H_
#include <QPen>
#include <QBrush>
#include <QColor>
#include <QVector>
#include <random>
#include <type_traits>
#include <memory>

#include "globals/constants.h"

struct ColorUtil
{
	ColorUtil();
    ~ColorUtil(){}

    QVector<QColor> heightMapPalette;
    QVector<QColor> heatMapPalette;
    QVector<QColor> hsvRandomMap;

    QColor getHeightMapColor(double v, double min, double max);
    QColor getHeatMapColor(double v, double min, double max);

	QColor sampleUniformColor();
};

extern ColorUtil colorUtil;


/*
 * Parameter classes to avoid confusion during RandomColorGenerator construction.
*/

struct HueSpread {

    explicit HueSpread(const uint s = 10) : s(s) {
        assert(s < 359);
    }

    uint& operator()() {
        return s;
    }

private:

    uint s;

};

struct SaturationSpread {

    explicit SaturationSpread(const uint s = 10) : s(s) {
        assert(s < 255);
    }

    uint& operator()() {
        return s;
    }

private:

    uint s;

};

struct ValueSpread {

    explicit ValueSpread(const uint s = 10) : s(s) {
        assert(s < 255);
    }

    uint& operator()() {
        return s;
    }

private:

    uint s;

};

/* Parameter helper class. */
template<uint H, uint S, uint V>
struct HSVSpread {

    static_assert (H >=0 && H <= 359, "Hue out of range 0-359");
    static_assert (S >=0 && S <= 255, "Saturation out of range 0-255");
    static_assert (V >=0 && V <= 255, "Value out of range 0-255");

    uint hSpread() {
        return H;
    }

    uint sSpread() {
        return S;
    }

    uint vSpread() {
        return V;
    }
};


class RandomColorGenerator {

private:

    real_t seed;

    uint hSpread, sSpread, vSpread;

    QColor initColor, lastColor;

    std::default_random_engine generator;

    // Zero mean normal distributions.
    std::normal_distribution<real_t> hDist, sDist, vDist;

    RandomColorGenerator(
            const uint hSpread,
            const uint sSpread,
            const uint vSpread,
            const real_t seed = 42.0,
            const QColor &initColor = Qt::black);


public:

    RandomColorGenerator(HueSpread hSpread,
            SaturationSpread sSpread,
            ValueSpread vSpread,
            const real_t seed = 42.0,
            const QColor &initColor = Qt::black);

    ~RandomColorGenerator(){}

    QColor next();

    void reset();

    template<uint H, uint S, uint V>
    static std::shared_ptr<RandomColorGenerator> get(HSVSpread<H,S,V> spread, const real_t seed = 42.0, const QColor &initColor = Qt::red) {
        return std::make_shared<RandomColorGenerator>(RandomColorGenerator(spread.hSpread(), spread.sSpread(), spread.vSpread(), seed, initColor));
    }
};

struct QColorHasher
{
    uint operator()(const QColor& k) const
    {
        uint res = 17;
        res = res * 31 + std::hash<int>()(k.spec());
        res = res * 31 + std::hash<int>()(k.alpha());
        res = res * 31 + std::hash<int>()(k.red());
        res = res * 31 + std::hash<int>()(k.green());
        res = res * 31 + std::hash<int>()(k.blue());
        return res;
    }
};

#endif /* COLORUTIL_H_ */

