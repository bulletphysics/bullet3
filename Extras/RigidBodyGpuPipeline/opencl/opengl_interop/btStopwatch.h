/*
Stopwatch for timing and profiling for the Bullet Physics Library, http://bulletphysics.org
Copyright (c) 2003-2011 Erwin Coumans

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef BT_STOPWATCH_H
#define BT_STOPWATCH_H

///The btStopwatch is a portable basic clock that measures real-time, use for profiling etc.
class btStopwatch
{
public:
	btStopwatch();

	btStopwatch(const btStopwatch& other);
	btStopwatch& operator=(const btStopwatch& other);

	~btStopwatch();

	/// Resets the initial reference time.
	void reset();

	/// Returns the time in ms since the last call to reset or since 
	/// the btStopwatch was created.
	float getTimeMilliseconds();

	/// Returns the time in us since the last call to reset or since 
	/// the Clock was created.
	unsigned long int getTimeMicroseconds();
private:
	struct btStopwatchData* m_data;
};


#endif //BT_STOPWATCH_H