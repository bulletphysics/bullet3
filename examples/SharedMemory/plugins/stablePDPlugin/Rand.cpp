#include "Rand.h"
#include <time.h>
#include <assert.h>
#include <algorithm>

cRand::cRand()
{
	unsigned long int seed = static_cast<unsigned long int>(time(NULL));
	mRandGen = std::default_random_engine(seed);
	mRandDoubleDist = std::uniform_real_distribution<double>(0, 1);
	mRandDoubleDistNorm = std::normal_distribution<double>(0, 1);
	mRandIntDist = std::uniform_int_distribution<int>(std::numeric_limits<int>::min() + 1, std::numeric_limits<int>::max()); // + 1 since there is one more neg int than pos int
	mRandUintDist = std::uniform_int_distribution<unsigned int>(std::numeric_limits<unsigned int>::min(), std::numeric_limits<unsigned int>::max());
}

cRand::cRand(unsigned long int seed) 
{
	Seed(seed);
}

cRand::~cRand()
{
}

double cRand::RandDouble()
{
	return mRandDoubleDist(mRandGen);
}

double cRand::RandDouble(double min, double max)
{
	if (min == max)
	{
		return min;
	}

	// generate random double in [min, max]
	double rand_double = mRandDoubleDist(mRandGen);
	rand_double = min + (rand_double * (max - min));
	return rand_double;
}

double cRand::RandDoubleExp(double lambda)
{
	std::exponential_distribution<double> dist(lambda);
	double rand_double = dist(mRandGen);
	return rand_double;
}

double cRand::RandDoubleNorm(double mean, double stdev)
{
	double rand_double = mRandDoubleDistNorm(mRandGen);
	rand_double = mean + stdev * rand_double;
	return rand_double;
}

int cRand::RandInt()
{
	return mRandIntDist(mRandGen);
}

int cRand::RandInt(int min, int max)
{
	if (min == max)
	{
		return min;
	}

	// generate random double in [min, max)
	int delta = max - min;
	int rand_int = std::abs(RandInt());
	rand_int = min + rand_int % delta;

	return rand_int;
}

int cRand::RandUint()
{
	return mRandUintDist(mRandGen);
}

int cRand::RandUint(unsigned int min, unsigned int max)
{
	if (min == max)
	{
		return min;
	}

	// generate random double in [min, max)
	int delta = max - min;
	int rand_int = RandUint();
	rand_int = min + rand_int % delta;

	return rand_int;
}

int cRand::RandIntExclude(int min, int max, int exc)
{
	int rand_int = 0;
	if (exc < min || exc >= max)
	{
		rand_int = RandInt(min, max);
	}
	else
	{
		int new_max = max - 1;
		if (new_max <= min)
		{
			rand_int = min;
		}
		else
		{
			rand_int = RandInt(min, new_max);
			if (rand_int >= exc)
			{
				++rand_int;
			}
		}
	}
	return rand_int;
}

void cRand::Seed(unsigned long int seed)
{
	mRandGen.seed(seed);
	mRandDoubleDist.reset();
	mRandDoubleDistNorm.reset();
	mRandIntDist.reset();
	mRandUintDist.reset();
}

int cRand::RandSign()
{
	return FlipCoin() ? -1 : 1;
}

bool cRand::FlipCoin(double p)
{
	return (RandDouble(0, 1) < p);
}
