

#ifndef GWEN_TEXTURE_WINDOW_H
#define GWEN_TEXTURE_WINDOW_H

struct MyGraphInput
{
	struct GwenInternalData* m_data;
	int m_xPos;
	int m_yPos;
	int m_width;
	int m_height;
	int m_borderWidth;
	const char* m_name;
	const char* m_texName;
	MyGraphInput(struct GwenInternalData* data)
		: m_data(data),
		  m_xPos(0),
		  m_yPos(0),
		  m_width(400),
		  m_height(400),
		  m_borderWidth(0),
		  m_name("GraphWindow"),
		  m_texName(0)
	{
	}
};
class MyGraphWindow* setupTextureWindow(const MyGraphInput& input);
void destroyTextureWindow(MyGraphWindow* window);

#endif  //GWEN_TEXTURE_WINDOW_H
