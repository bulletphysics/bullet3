#ifndef B3_CLOCK_H
#define B3_CLOCK_H

///The b3Clock is a portable basic clock that measures accurate time in seconds, use for profiling.
class b3Clock
{
public:
	b3Clock();

	b3Clock(const b3Clock& other);
	b3Clock& operator=(const b3Clock& other);

	~b3Clock();

	/// Resets the initial reference time. If zeroReference is true, will set reference to absolute 0.
	void reset(bool zeroReference = false);

	/// Returns the time in ms since the last call to reset or since
	/// the b3Clock was created.
	unsigned long int getTimeMilliseconds();

	/// Returns the time in us since the last call to reset or since
	/// the Clock was created.
	unsigned long long int getTimeMicroseconds();

	/// Returns the time in seconds since the last call to reset or since
	/// the Clock was created.
	double getTimeInSeconds();

	///Sleep for 'microSeconds', to yield to other threads and not waste 100% CPU cycles.
	///Note that some operating systems may sleep a longer time.
	static void usleep(int microSeconds);

private:
	struct b3ClockData* m_data;
};

#endif  //B3_CLOCK_H
