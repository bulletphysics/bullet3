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

	/// Resets the initial reference time.
	void reset();

	/// Returns the time in ms since the last call to reset or since 
	/// the b3Clock was created.
	unsigned long int getTimeMilliseconds();

	/// Returns the time in us since the last call to reset or since 
	/// the Clock was created.
	unsigned long int getTimeMicroseconds();
private:
	struct b3ClockData* m_data;
};


#endif //B3_CLOCK_H
