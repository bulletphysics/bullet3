#ifndef B3_WRITE_WAV_FILE_H
#define B3_WRITE_WAV_FILE_H

// b3WriteWavFile is copied from Stk::FileWvOut/FileWrite
// See also https://github.com/thestk/stk
// by Perry R. Cook and Gary P. Scavone, 1995--2014.
#include <string>

class b3WriteWavFile
{
	void incrementFrame( void );
	void flush();

	struct b3WriteWavFileInternalData* m_data;

	void flushData(int bufferSize);

public:

	b3WriteWavFile();
	virtual ~b3WriteWavFile();

	bool setWavFile(std::string fileName, int sampleRate, int numChannels, bool useDoublePrecision=true);

	void closeWavFile();

	void tick( double* values, int numValues );
	

};

#endif  //B3_WRITE_WAV_FILE_H
