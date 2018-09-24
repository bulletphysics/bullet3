//
// Copyright (c) 2011 Andreas Krinke andreas.krinke@gmx.de
// Copyright (c) 2009 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

#ifndef FONTSTASH_H
#define FONTSTASH_H

#define MAX_ROWS 128
#define VERT_COUNT (16 * 128)
#define INDEX_COUNT (VERT_COUNT * 2)

struct vec2
{
	vec2(float x, float y)
	{
		p[0] = x;
		p[1] = y;
	}
	float p[2];
};

struct vec4
{
	vec4(float x, float y, float z, float w)
	{
		p[0] = x;
		p[1] = y;
		p[2] = z;
		p[3] = w;
	}

	float p[4];
};

typedef struct
{
	vec4 position;
	vec4 colour;
	vec2 uv;
} Vertex;

struct sth_quad
{
	float x0, y0, s0, t0;
	float x1, y1, s1, t1;
};

struct sth_row
{
	short x, y, h;
};

struct sth_glyph
{
	unsigned int codepoint;
	short size;
	struct sth_texture* texture;
	int x0_, y0, x1, y1;
	float xadv, xoff, yoff;
	int next;
};

struct sth_texture
{
	union {
		void* m_userData;
		int m_userId;
	};

	unsigned char* m_texels;

	// TODO: replace rows with pointer
	struct sth_row rows[MAX_ROWS];
	int nrows;
	int nverts;

	Vertex newverts[VERT_COUNT];
	struct sth_texture* next;
};

struct RenderCallbacks
{
	virtual ~RenderCallbacks() {}
	virtual void setColorRGBA(float color[4]) = 0;
	virtual void setWorldPosition(float pos[3]) = 0;
	virtual void setWorldOrientation(float orn[4]) = 0;
	virtual void updateTexture(sth_texture* texture, sth_glyph* glyph, int textureWidth, int textureHeight) = 0;
	virtual void render(sth_texture* texture) = 0;
};

struct sth_stash* sth_create(int cachew, int cacheh, RenderCallbacks* callbacks);

int sth_add_font(struct sth_stash* stash, const char* path);
int sth_add_font_from_memory(struct sth_stash* stash, unsigned char* buffer);

int sth_add_bitmap_font(struct sth_stash* stash, int ascent, int descent, int line_gap);
/*void sth_add_glyph(struct sth_stash* stash, int idx, unsigned int uid, const char* s,
                  short size, short base, int x, int y, int w, int h,
                  float xoffset, float yoffset, float xadvance);
				  */

void sth_begin_draw(struct sth_stash* stash);
void sth_end_draw(struct sth_stash* stash);

void sth_draw_texture(struct sth_stash* stash,
					  int idx, float size,
					  float x, float y,
					  int screenwidth, int screenheight,
					  const char* s, float* dx, float colorRGBA[4]);

void sth_flush_draw(struct sth_stash* stash);

void sth_draw_text3D(struct sth_stash* stash,
					 int idx, float fontSize,
					 float x, float y, float z,
					 const char* s, float* dx, float textScale, float colorRGBA[4], int bla);

void sth_draw_text(struct sth_stash* stash,
				   int idx, float size,
				   float x, float y, const char* string, float* dx, int screenwidth, int screenheight, int measureOnly, float retinaScale, float colorRGBA[4]);

inline void sth_draw_text(struct sth_stash* stash,
						  int idx, float size,
						  float x, float y, const char* string, float* dx, int screenwidth, int screenheight, int measureOnly = false, float retinaScale = 1.)
{
	float colorRGBA[4] = {1, 1, 1, 1};
	sth_draw_text(stash, idx, size, x, y, string, dx, screenwidth, screenheight, measureOnly, retinaScale, colorRGBA);
}

void sth_dim_text(struct sth_stash* stash, int idx, float size, const char* string,
				  float* minx, float* miny, float* maxx, float* maxy);

void sth_vmetrics(struct sth_stash* stash,
				  int idx, float size,
				  float* ascender, float* descender, float* lineh);

void sth_delete(struct sth_stash* stash);

#endif  // FONTSTASH_H
