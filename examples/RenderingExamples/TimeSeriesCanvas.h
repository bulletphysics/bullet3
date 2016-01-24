#ifndef TIME_SERIES_CANVAS_H
#define TIME_SERIES_CANVAS_H

class TimeSeriesCanvas
{
protected:
	struct TimeSeriesInternalData* m_internalData;
	void shift1PixelToLeft();
	void grapicalPrintf(const char* str,	void* fontData, int rasterposx,int rasterposy,unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha);

public:
	
	TimeSeriesCanvas(struct Common2dCanvasInterface* canvasInterface, int width, int height, const char* windowTitle);
	virtual ~TimeSeriesCanvas();

	void setupTimeSeries(float yScale, int ticksPerSecond, int startTime);
	void addDataSource(const char* dataSourceLabel, unsigned char red,unsigned char green,unsigned char blue);
	void insertDataAtCurrentTime(float value, int dataSourceIndex, bool connectToPrevious);
	float getCurrentTime() const;

	virtual void nextTick();
	
};

#endif//TIME_SERIES_CANVAS_H