#include "TinyVRGui.h"
#include "../CommonInterfaces/CommonGUIHelperInterface.h"
#include "../ExampleBrowser/GwenGUISupport/GraphingTexture.h"

#include "../CommonInterfaces/Common2dCanvasInterface.h"
#include "../RenderingExamples/TimeSeriesCanvas.h"
#include "../RenderingExamples/TimeSeriesFontData.h"
#include "../Importers/ImportMeshUtility/b3ImportMeshUtility.h"
#include "../OpenGLWindow/GLInstanceGraphicsShape.h"
#include "../Utils/b3BulletDefaultFileIO.h"

#include "../CommonInterfaces/CommonRenderInterface.h"
#include "../CommonInterfaces/CommonParameterInterface.h"

struct TestCanvasInterface2 : public Common2dCanvasInterface
{
	b3AlignedObjectArray<unsigned char>& m_texelsRGB;
	int m_width;
	int m_height;

	TestCanvasInterface2(b3AlignedObjectArray<unsigned char>& texelsRGB, int width, int height)
		: m_texelsRGB(texelsRGB),
		  m_width(width),
		  m_height(height)
	{
	}

	virtual ~TestCanvasInterface2()
	{
	}
	virtual int createCanvas(const char* canvasName, int width, int height, int posX, int posY)
	{
		return 0;
	}
	virtual void destroyCanvas(int canvasId)
	{
	}
	virtual void setPixel(int canvasId, int x, int y, unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha)
	{
		if (x >= 0 && x < m_width && y >= 0 && y < m_height)
		{
			m_texelsRGB[(x + y * m_width) * 3 + 0] = red;
			m_texelsRGB[(x + y * m_width) * 3 + 1] = green;
			m_texelsRGB[(x + y * m_width) * 3 + 2] = blue;
		}
	}
	virtual void getPixel(int canvasId, int x, int y, unsigned char& red, unsigned char& green, unsigned char& blue, unsigned char& alpha)
	{
		if (x >= 0 && x < m_width && y >= 0 && y < m_height)
		{
			red = m_texelsRGB[(x + y * m_width) * 3 + 0];
			green = m_texelsRGB[(x + y * m_width) * 3 + 1];
			blue = m_texelsRGB[(x + y * m_width) * 3 + 2];
		}
	}

	virtual void refreshImageData(int canvasId)
	{
	}
};

struct TinyVRGuiInternalData
{
	CommonRenderInterface* m_renderer;

	b3AlignedObjectArray<unsigned char> m_texelsRGB;
	TestCanvasInterface2* m_testCanvas;
	TimeSeriesCanvas* m_timeSeries;
	int m_src;
	int m_textureId;
	int m_gfxObjectId;

	TinyVRGuiInternalData()
		: m_renderer(0),
		  m_testCanvas(0),
		  m_timeSeries(0),
		  m_src(-1),
		  m_textureId(-1),
		  m_gfxObjectId(-1)
	{
	}
};

TinyVRGui::TinyVRGui(struct ComboBoxParams& params, struct CommonRenderInterface* renderer)
{
	m_data = new TinyVRGuiInternalData;
	m_data->m_renderer = renderer;
}

TinyVRGui::~TinyVRGui()
{
	delete m_data->m_timeSeries;
	delete m_data->m_testCanvas;
	delete m_data;
}

bool TinyVRGui::init()
{
	{
		int width = 256;
		int height = 256;
		m_data->m_texelsRGB.resize(width * height * 3);
		for (int i = 0; i < width; i++)
			for (int j = 0; j < height; j++)
			{
				m_data->m_texelsRGB[(i + j * width) * 3 + 0] = 155;
				m_data->m_texelsRGB[(i + j * width) * 3 + 1] = 155;
				m_data->m_texelsRGB[(i + j * width) * 3 + 2] = 255;
			}

		m_data->m_testCanvas = new TestCanvasInterface2(m_data->m_texelsRGB, width, height);
		m_data->m_timeSeries = new TimeSeriesCanvas(m_data->m_testCanvas, width, height, "time series");

		bool clearCanvas = false;

		m_data->m_timeSeries->setupTimeSeries(3, 100, 0, clearCanvas);
		m_data->m_timeSeries->addDataSource("Some sine wave", 255, 0, 0);
		m_data->m_timeSeries->addDataSource("Some cosine wave", 0, 255, 0);
		m_data->m_timeSeries->addDataSource("Delta Time (*10)", 0, 0, 255);
		m_data->m_timeSeries->addDataSource("Tan", 255, 0, 255);
		m_data->m_timeSeries->addDataSource("Some cosine wave2", 255, 255, 0);
		m_data->m_timeSeries->addDataSource("Empty source2", 255, 0, 255);

		m_data->m_textureId = m_data->m_renderer->registerTexture(&m_data->m_texelsRGB[0], width, height);

		{
			const char* fileName = "cube.obj";  //"textured_sphere_smooth.obj";
			//fileName = "cube.obj";

			int shapeId = -1;

			b3ImportMeshData meshData;
			b3BulletDefaultFileIO fileIO;
			if (b3ImportMeshUtility::loadAndRegisterMeshFromFileInternal(fileName, meshData,&fileIO))
			{
				shapeId = m_data->m_renderer->registerShape(&meshData.m_gfxShape->m_vertices->at(0).xyzw[0],
															meshData.m_gfxShape->m_numvertices,
															&meshData.m_gfxShape->m_indices->at(0),
															meshData.m_gfxShape->m_numIndices,
															B3_GL_TRIANGLES,
															m_data->m_textureId);

				float position[4] = {0, 0, 2, 1};
				float orn[4] = {0, 0, 0, 1};
				float color[4] = {1, 1, 1, 1};
				float scaling[4] = {.1, .1, .1, 1};

				m_data->m_gfxObjectId = m_data->m_renderer->registerGraphicsInstance(shapeId, position, orn, color, scaling);
				m_data->m_renderer->writeTransforms();

				meshData.m_gfxShape->m_scaling[0] = scaling[0];
				meshData.m_gfxShape->m_scaling[1] = scaling[1];
				meshData.m_gfxShape->m_scaling[2] = scaling[2];

				delete meshData.m_gfxShape;
				if (!meshData.m_isCached)
				{
					free(meshData.m_textureImage1);
				}
			}
		}
	}

	m_data->m_renderer->writeTransforms();
	return true;
}

void TinyVRGui::tick(b3Scalar deltaTime, const b3Transform& guiWorldTransform)
{
	float time = m_data->m_timeSeries->getCurrentTime();
	float v = sinf(time);
	m_data->m_timeSeries->insertDataAtCurrentTime(v, 0, true);
	v = cosf(time);
	m_data->m_timeSeries->insertDataAtCurrentTime(v, 1, true);
	v = tanf(time);
	m_data->m_timeSeries->insertDataAtCurrentTime(v, 3, true);
	m_data->m_timeSeries->insertDataAtCurrentTime(deltaTime * 10, 2, true);

	m_data->m_timeSeries->nextTick();

	m_data->m_renderer->updateTexture(m_data->m_textureId, &m_data->m_texelsRGB[0]);
	m_data->m_renderer->writeSingleInstanceTransformToCPU(guiWorldTransform.getOrigin(), guiWorldTransform.getRotation(), m_data->m_gfxObjectId);
	m_data->m_renderer->writeTransforms();
}

void TinyVRGui::clearTextArea()
{
	int width = 256;
	int height = 50;
	for (int i = 0; i < width; i++)
		for (int j = 0; j < height; j++)
		{
			m_data->m_texelsRGB[(i + j * width) * 3 + 0] = 155;
			m_data->m_texelsRGB[(i + j * width) * 3 + 1] = 155;
			m_data->m_texelsRGB[(i + j * width) * 3 + 2] = 255;
		}
}

void TinyVRGui::grapicalPrintf(const char* str, int rasterposx, int rasterposy, unsigned char red, unsigned char green, unsigned char blue, unsigned char alpha)
{
	m_data->m_timeSeries->grapicalPrintf(str, sTimeSeriesFontData, rasterposx, rasterposy, red, green, blue, alpha);
}
