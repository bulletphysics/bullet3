#ifndef B3_READ_WAV_FILE_H
#define B3_READ_WAV_FILE_H

#include "Bullet3Common/b3AlignedObjectArray.h"
#include <stdio.h>
#include <string.h>

struct b3WavTicker
{
	b3AlignedObjectArray<double> lastFrame_;
	bool finished_;
	double time_;
	double rate_;
};

class b3ReadWavFile
{
	bool byteswap_;
	bool wavFile_;
	unsigned long m_numFrames;
	unsigned long dataType_;
	double fileDataRate_;
	FILE *fd_;
	unsigned long dataOffset_;
	unsigned int channels_;
	bool m_machineIsLittleEndian;

public:
	b3ReadWavFile();
	virtual ~b3ReadWavFile();

	b3AlignedObjectArray<double> m_frames;

	bool getWavInfo(const char *fileName);

	void normalize(double peak);

	double interpolate(double frame, unsigned int channel) const;
	double tick(unsigned int channel, b3WavTicker *ticker);

	void resize();

	b3WavTicker createWavTicker(double sampleRate);

	bool read(unsigned long startFrame, bool doNormalize);

	int getNumFrames() const
	{
		return m_numFrames;
	}
};

#endif  //B3_READ_WAV_FILE_H
