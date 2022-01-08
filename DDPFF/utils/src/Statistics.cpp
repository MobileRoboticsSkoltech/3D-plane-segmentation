#include "utils/Statistics.h"
#include <QTime>
#include <algorithm>

Statistics statistics;
uint Statistics::seed;
Statistics::Statistics()
{
    init();
}

// Initializes the random number generator.
// Warning! The RNG is not thread safe! If you are using the Statistics package
// in a different thread from the main, you need to call Statistics::init() explicitly.
void Statistics::init()
{
    QTime time = QTime::currentTime();
    seed = (uint)time.msec();
    srand(seed);
}

// Seeds the random number generator.
void Statistics::setSeed(uint s)
{
    seed = s;
    srand(s);
}

// Returns the seed of the random number generator.
uint Statistics::getSeed()
{
    return seed;
}

// Returns the softmin of the values in the list.
double Statistics::softmin(const Vector<double>& list)
{
    if (list.empty()){
        return 0;
    }

    double softmin = 0;
    double totalWeight = 0;
    for (uint i=0; i<list.size(); i++)
    {
        double w = exp(-list[i]);
        totalWeight += w;
        softmin += w*list[i];
    }
    softmin /= totalWeight;
    return softmin;
}

// Returns the sofmax of the values in the list.
double Statistics::softmax(const Vector<double>& list)
{
    if (list.empty()){
        return 0;
    }

    double softmax = 0;
    double totalWeight = 0;
    for (uint i=0; i<list.size(); i++)
    {
        double w = exp(list[i]);
        totalWeight += w;
        softmax += w*list[i];
    }
    softmax /= totalWeight;
    return softmax;
}

// Returns the minimum of the values in the list.
double Statistics::min(const Vector<double>& list)
{
    if (list.empty()){
        return 0;
    }

    double min = list[0];
    for (uint i = 0; i < list.size(); i++)
        min = qMin(min, list[i]);
    return min;
}

// Returns the maximum of the values in the list.
double Statistics::max(const Vector<double>& list)
{
    if (list.empty()){
        return 0;
    }

    double max = list[0];
    for (uint i = 0; i < list.size(); i++)
        max = qMax(list[i], max);
    return max;
}

// Returns the minimum (in vec.x) and the maximum (in vec.y) of the values in the list.
Vec2 Statistics::minmax(const Vector<double>& list)
{
    if (list.empty()){
        return Vec2();
    }

	double min = list[0];
	double max = list[0];
    for (uint i = 0; i < list.size(); i++)
	{
		min = qMin(list[i], min);
		max = qMax(max, list[i]);
	}
    return Vec2(min, max);
}

// Returns the mean (in x) and the standard deviation (in y) of the values in the list.
Vec2 Statistics::meanstddev(const Vector<double>& list)
{
	double mean = 0;
	double stddev = 0;

    for (uint i = 0; i < list.size(); i++)
		mean += list[i];
	mean /= list.size();

    for (uint i = 0; i < list.size(); i++)
        stddev += (list[i]-mean)*(list[i]-mean);
    stddev = sqrt(stddev/(list.size()-1)); // http://www.visiondummy.com/2014/03/divide-variance-n-1/

    return Vec2(mean, stddev);
}

// Returns the mean of the values in the list.
double Statistics::mean(const Vector<double>& list)
{
	double mean = 0;
    for (uint i = 0; i < list.size(); i++)
		mean += list[i];
	mean /= list.size();
	return mean;
}

// Generates a 1D histogram over the list with bins number of bins.
// Psst, a histogram counts how many of the values from the list are in each bin.
Vector<int> Statistics::histogram(const Vector<double>& list, int bins, double min, double max)
{
    Vector<int> hist(bins);
	if (list.size() <= 0)
		return hist;

    if (min == max)
    {
        min = list[0];
        max = list[0];
        for (uint i = 1; i < list.size(); i++)
        {
            min = qMin(list[i], min);
            max = qMax(max, list[i]);
        }
    }

    for (uint i = 0; i < list.size(); i++)
	{
        int idx = qBound<int>(0, qRound((bins-1)*(list[i]-min)/(max-min)), hist.size()-1);
        hist[idx]++;
	}

    return hist;
}

// Returns the median of the values in the list.
double Statistics::median(const Vector<double>& list)
{
    Vector<double> myList = list;
    std::sort(myList.begin(), myList.end());
    return myList[myList.size()/2];
}

// Returns a random integer between 0 and RAND_MAX
int Statistics::randomInt(int low, int high)
{
    return low + rand() * double(high-low)/RAND_MAX;
}

// Returns a sample from a uniformly distributed open (0, 1) interval.
double Statistics::randomNumber() {return uniformSample();}
double Statistics::uniformSample()
{
    double r = ((double)rand())/((double)RAND_MAX);
    return r;
}

// Returns a sample from a uniformly distributed open (low, high) interval.
double Statistics::randomNumber(double low, double high) {return uniformSample(low, high);}
double Statistics::uniformSample(double low, double high)
{
	return low + uniformSample() * (high - low);
}

// Returns a sample from a zero mean, 1 standard deviation normal distribution.
// This method could be used more efficiently if returning both, the cos and
// the sin result, or it could be entirely replaced by the ziggurat algorithm.
double Statistics::normalSample()
{
  double u1=uniformSample();
  double u2=uniformSample();
  return cos(8.*atan(1.)*u2)*sqrt(-2.*log(u1));
  //return sin(8.*atan(1.)*u1)*sqrt(-2.*log(u2));
}

// Returns a sample from a Gaussian distribution with given mean and standard deviation.
double Statistics::normalSample(double mean, double stddev)
{
    return normalSample()*stddev+mean;
}
