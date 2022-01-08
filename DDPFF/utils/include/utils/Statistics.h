#ifndef STATISTICS_H_
#define STATISTICS_H_

#include "vector"
#include "globals/constants.h"

template<typename T>
using Vector = std::vector<T>;

class Statistics
{
    static uint seed;
public:
	Statistics();
    ~Statistics(){}
    static void init();
    static void setSeed(uint s);
    static uint getSeed();
    static double min(const Vector<double>& values);
    static double max(const Vector<double>& values);
    static Vec2 minmax(const Vector<double>& values);
    static double softmin(const Vector<double>& values);
    static double softmax(const Vector<double>& values);
    static double mean(const Vector<double>& values);
    static Vec2 meanstddev(const Vector<double>& values);
    static double median(const Vector<double>& values);
    static Vector<int> histogram(const Vector<double>& list, int bins=100, double min=0, double max=0);
    static int randomInt(int low=0, int high=RAND_MAX);
    static double randomNumber();
    static double randomNumber(double low, double high);
    static double uniformSample();
    static double uniformSample(double low, double high);
    static double normalSample();
    static double normalSample(double mean, double stddev);
};

extern Statistics statistics;

#endif /* STATISTICS_H_ */

