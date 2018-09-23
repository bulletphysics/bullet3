#ifndef TIME_SERIES_CANVAS_H
#define TIME_SERIES_CANVAS_H

class TimeSeriesCanvas
{
protected:
	struct TimeSeriesInternalData* m_internalData;
	void shift1PixelToLeft();

public:
	TimeSeriesCanvas(struct Common2dCanvasInterface* canvasInterface, int width, int height, const char* windowTitle);
	virtual ~TimeSeriesCanvas();

	void setupTimeSeries(float yScale, int ticksPerSecond, int startTime, bool clearCanvas = true);
	void addDataSource(const char* dataSourceLabel, unsigned char red, unsigned char green, unsigned char blue);
	void insertDataAtCurrentTime(float value, int dataSourceIndex, bool connectToPrevious);
	float getCurrentTime() const;
	void grapicalPrintf(const char* str, void* fontData, int rasterposx, int rasterposy, unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha);

	virtual void nextTick();
};

#endif  //TIME_SERIES_CANVAS_H