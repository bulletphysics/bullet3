#include "TimeSeriesCanvas.h"
#include "../CommonInterfaces/Common2dCanvasInterface.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "TimeSeriesFontData.h"
#include "LinearMath/btVector3.h"
#include <stdio.h>

struct DataSource
{
	unsigned char m_red;
	unsigned char m_green;
	unsigned char m_blue;
	float m_lastValue;
	bool m_hasLastValue;
	DataSource()
		:m_hasLastValue(false)
	{
	}
};

struct TimeSeriesInternalData
{
	btAlignedObjectArray<DataSource> m_dataSources;

	struct Common2dCanvasInterface* m_canvasInterface;
	int	m_canvasIndex;
	int m_width;
	int m_height;

	float m_pixelsPerUnit;
	float m_zero;
	int m_timeTicks;
	int m_ticksPerSecond;
	float m_yScale;
	int m_bar;

	unsigned char m_backgroundRed;
	unsigned char m_backgroundGreen;
	unsigned char m_backgroundBlue;
	unsigned char m_backgroundAlpha;

	unsigned char m_textColorRed;
	unsigned char m_textColorGreen;
	unsigned char m_textColorBlue;
	unsigned char m_textColorAlpha;

	float getTime() 
	{
		return m_timeTicks/(float)m_ticksPerSecond;
	}

	TimeSeriesInternalData(int width, int height)
		:m_width(width),
		m_height(height),
		m_pixelsPerUnit(-100),
		m_zero(height/2.0),
		m_timeTicks(0),
		m_ticksPerSecond(100),
		m_yScale(1),
		m_bar(0),
		m_backgroundRed(255),
		m_backgroundGreen(255),
		m_backgroundBlue(255),
		m_backgroundAlpha(255),
		m_textColorRed(0),
		m_textColorGreen(0),
		m_textColorBlue(255),
		m_textColorAlpha(255)
	{
	}
};

TimeSeriesCanvas::TimeSeriesCanvas(struct Common2dCanvasInterface* canvasInterface, int width, int height, const char* windowTitle)
{
	m_internalData = new TimeSeriesInternalData(width,height);
	
	m_internalData->m_canvasInterface = canvasInterface;

	if (canvasInterface)
	{
		m_internalData->m_canvasIndex = m_internalData->m_canvasInterface->createCanvas(windowTitle,m_internalData->m_width,m_internalData->m_height,20,50);
	}
}

void TimeSeriesCanvas::addDataSource(const char* dataSourceLabel, unsigned char red,unsigned char green,unsigned char blue)
{
	DataSource dataSource;
	dataSource.m_red = red;
	dataSource.m_green = green;
	dataSource.m_blue = blue;
	dataSource.m_lastValue = 0;
	dataSource.m_hasLastValue = false;

	
	if (dataSourceLabel)
	{
		int numSources = m_internalData->m_dataSources.size();
		int row = numSources%3;
		int column = numSources/3;
		grapicalPrintf(dataSourceLabel, sTimeSeriesFontData, 50+200*column,m_internalData->m_height-48+row*16,
			red, green,blue,255);
	}

	m_internalData->m_dataSources.push_back(dataSource);

}
void TimeSeriesCanvas::setupTimeSeries(float yScale, int ticksPerSecond, int startTime, bool clearCanvas)
{
	if (0==m_internalData->m_canvasInterface)
		return;

	m_internalData->m_pixelsPerUnit = -(m_internalData->m_height/3.f)/yScale;
	m_internalData->m_ticksPerSecond = ticksPerSecond;
	m_internalData->m_yScale = yScale;
	m_internalData->m_dataSources.clear();
	
	if (clearCanvas)
	{
		for (int i=0;i<m_internalData->m_width;i++)
		{
			for (int j=0;j<m_internalData->m_height;j++)
			{
			
				m_internalData->m_canvasInterface->setPixel(m_internalData->m_canvasIndex,i,j,
					m_internalData->m_backgroundRed,
					m_internalData->m_backgroundGreen,
					m_internalData->m_backgroundBlue,
					m_internalData->m_backgroundAlpha);
			}
		}
	}
	
	float zeroPixelCoord = m_internalData->m_zero;
	float pixelsPerUnit = m_internalData->m_pixelsPerUnit;
	
	float yPos = zeroPixelCoord+pixelsPerUnit*yScale;
	float yNeg = zeroPixelCoord+pixelsPerUnit*-yScale;

	
	grapicalPrintf("0", sTimeSeriesFontData, 2,zeroPixelCoord,m_internalData->m_textColorRed,m_internalData->m_textColorGreen,m_internalData->m_textColorBlue,m_internalData->m_textColorAlpha);
	char label[1024];
	sprintf(label,"%2.1f", yScale);
	grapicalPrintf(label, sTimeSeriesFontData, 2,yPos,m_internalData->m_textColorRed,m_internalData->m_textColorGreen,m_internalData->m_textColorBlue,m_internalData->m_textColorAlpha);
	sprintf(label,"%2.1f", -yScale);
	grapicalPrintf(label, sTimeSeriesFontData, 2,yNeg,m_internalData->m_textColorRed,m_internalData->m_textColorGreen,m_internalData->m_textColorBlue,m_internalData->m_textColorAlpha);
		
	m_internalData->m_canvasInterface->refreshImageData(m_internalData->m_canvasIndex);

}

TimeSeriesCanvas::~TimeSeriesCanvas()
{
	if (m_internalData->m_canvasInterface && m_internalData->m_canvasIndex>=0)
	{
		m_internalData->m_canvasInterface->destroyCanvas(m_internalData->m_canvasIndex);
	}
	delete m_internalData;
}

float TimeSeriesCanvas::getCurrentTime() const
{
	return m_internalData->getTime();
}
void TimeSeriesCanvas::grapicalPrintf(const char* str,	void* fontData, int rasterposx,int rasterposy, unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha)
{
	unsigned char c;
	int x=0;
	int xx=0;
	
	while ((c = (unsigned char) *str++)) {
		
		x=xx;		
		unsigned char* fontPtr = (unsigned char*) fontData;
		char ch = c-32;
		
		int sx=ch%16;
		int sy=ch/16;
		
		
		for (int i=sx*16;i<(sx*16+16);i++)
		{
			int y=0;
			for (int j=sy*16;j<(sy*16+16);j++)
			{
				unsigned char packedColor = (fontPtr[i*3+255*256*3-(256*j)*3]);
				//float colorf = packedColor ? 0.f : 1.f;
				float colorf = packedColor/255.f;// ? 0.f : 1.f;
				btVector4 rgba(colorf,colorf,colorf,1.f);
				if (colorf)
				{
					if ((rasterposx+x>=0) && (rasterposx+x < m_internalData->m_width) &&
					(rasterposy+y>=0) && (rasterposy+y<m_internalData->m_height))
					{
					m_internalData->m_canvasInterface->setPixel(m_internalData->m_canvasIndex,rasterposx+x,rasterposy+y,
													   red,green,blue,alpha);
					}	
				}
				y++;
			}
			x++;
		}
		xx+=10;
	}
}

void TimeSeriesCanvas::shift1PixelToLeft()
{
	
	int resetVal = 10;
	int countdown = resetVal;

	//shift pixture one pixel to the left
	for (int j=50;j<m_internalData->m_height-48;j++)
	{
		for (int i=40;i<this->m_internalData->m_width;i++)
		{
			unsigned char red, green, blue, alpha;
			m_internalData->m_canvasInterface->getPixel(m_internalData->m_canvasIndex,i,j,red,green,blue,alpha);
			m_internalData->m_canvasInterface->setPixel(m_internalData->m_canvasIndex,i-1,j,red,green,blue,alpha);
		}
		if (!m_internalData->m_bar)
		{
			
			
			if (!countdown--)
			{
				countdown=resetVal;
				m_internalData->m_canvasInterface->setPixel(m_internalData->m_canvasIndex,m_internalData->m_width-1,j,0,0,0,255);
			} else
			{
				m_internalData->m_canvasInterface->setPixel(m_internalData->m_canvasIndex,m_internalData->m_width-1,j,255,255,255,255);
				
			}
		} else
		{
			m_internalData->m_canvasInterface->setPixel(m_internalData->m_canvasIndex,m_internalData->m_width-1,j,255,255,255,255);
			
		}
	}


		{
			int resetVal = 2;
			static int countdown = resetVal;
			if (!countdown--)
			{
				countdown=resetVal;
				m_internalData->m_canvasInterface->setPixel(m_internalData->m_canvasIndex,m_internalData->m_width-1,m_internalData->m_zero,0,0,0,255);
			}
		}

		{
			int resetVal = 10;
			static int countdown = resetVal;
			if (!countdown--)
			{
				countdown=resetVal;
				float zeroPixelCoord = m_internalData->m_zero;
				float pixelsPerUnit = m_internalData->m_pixelsPerUnit;

				float yPos = zeroPixelCoord+pixelsPerUnit*m_internalData->m_yScale;
				float yNeg = zeroPixelCoord+pixelsPerUnit*-m_internalData->m_yScale;

				m_internalData->m_canvasInterface->setPixel(m_internalData->m_canvasIndex,m_internalData->m_width-1,
					yPos,0,0,0,255);
				m_internalData->m_canvasInterface->setPixel(m_internalData->m_canvasIndex,m_internalData->m_width-1,
					yNeg,0,0,0,255);
			}
		}
	
	
	
	
	if (!m_internalData->m_bar)
	{
		char buf[1024];
		float time = m_internalData->getTime();
		sprintf(buf,"%2.0f",time);
		grapicalPrintf(buf, sTimeSeriesFontData, m_internalData->m_width-25,m_internalData->m_zero+3,0,0,0,255);

		m_internalData->m_bar=m_internalData->m_ticksPerSecond;
		
	}
	m_internalData->m_timeTicks++;

	m_internalData->m_bar--;

}

void TimeSeriesCanvas::insertDataAtCurrentTime(float orgV, int dataSourceIndex, bool connectToPrevious)
{
	if (0==m_internalData->m_canvasInterface)
		return;

	btAssert(dataSourceIndex < m_internalData->m_dataSources.size());

	float zero = m_internalData->m_zero;
	float amp = m_internalData->m_pixelsPerUnit;
	//insert some new value(s) in the right-most column
	{
	//	float time = m_internalData->getTime();
		
		float v = zero+amp*orgV;
		
		m_internalData->m_canvasInterface->setPixel(m_internalData->m_canvasIndex,m_internalData->m_width-1,v,
					m_internalData->m_dataSources[dataSourceIndex].m_red,
					m_internalData->m_dataSources[dataSourceIndex].m_green,
					m_internalData->m_dataSources[dataSourceIndex].m_blue,
					255
					);

		if (connectToPrevious && m_internalData->m_dataSources[dataSourceIndex].m_hasLastValue)
		{
			for (int value=m_internalData->m_dataSources[dataSourceIndex].m_lastValue;value<=v;value++)
			{
				if (value>=0 && value < float(m_internalData->m_height-1))
				{
					m_internalData->m_canvasInterface->setPixel(m_internalData->m_canvasIndex,m_internalData->m_width-1,value,
						m_internalData->m_dataSources[dataSourceIndex].m_red,
						m_internalData->m_dataSources[dataSourceIndex].m_green,
						m_internalData->m_dataSources[dataSourceIndex].m_blue,
						255
						);
				}
			}

		
	
		
			for (int value=v;value<=m_internalData->m_dataSources[dataSourceIndex].m_lastValue;value++)
			{
				if (value>=0 && value < float(m_internalData->m_height-1))
				{
					m_internalData->m_canvasInterface->setPixel(m_internalData->m_canvasIndex,m_internalData->m_width-1,value,
						m_internalData->m_dataSources[dataSourceIndex].m_red,
						m_internalData->m_dataSources[dataSourceIndex].m_green,
						m_internalData->m_dataSources[dataSourceIndex].m_blue,
						255);
				}
			}
			
		}
		m_internalData->m_dataSources[dataSourceIndex].m_lastValue = v;
			m_internalData->m_dataSources[dataSourceIndex].m_hasLastValue = true;
	}




}
void TimeSeriesCanvas::nextTick()
{
	shift1PixelToLeft();
	m_internalData->m_canvasInterface->refreshImageData(m_internalData->m_canvasIndex);
}
