
//-----------------------------------------------------------------------------------------------
//
// The remainder of this file shows how to use the spring network class with backward integration
// in order to implement a cloth system within a 3D game environment.
// The cloth class extends the springnetwork class in order to provide
// import/export, rendering support, and hooks into the game.
//

#include "Cloth.h"

Array<Cloth *> cloths;

Cloth::Cloth(const char *_name, int _n) : SpringNetwork(_n),
										  color(0, 0.5f, 1.0f)
{
	cloths.Add(this);
}
Cloth::~Cloth()
{
	cloths.Remove(this);
}

//
// I/O support for serialization of our springnetwork and cloth objects.
//

int cloth_showbbox = 0;         // for debug visualization  shows bounding box.
float cloth_showvert = 0.025f;  // size of box to put around current vert selected, 0 turns off

Cloth *ClothCreate(int w, int h, float size)
{
	// simple cloth generation routine that creates a typical square cloth.
	// better to use a real pipeline to generate these, this is just for testing.
	int i, j;
	Cloth *cloth = new Cloth("cloth", w * h);
	cloth->w = w;
	cloth->h = h;  // later for rendering.
	for (i = 0; i < h; i++)
		for (j = 0; j < w; j++)
		{
			cloth->X[i * w + j] = (float3(-0.5f, -0.5f, 0) + float3((float)j / (w - 1.0f), 1.0f - (float)i / (h - 1.0f), 0)) * size;
		}
	for (i = 0; i < h; i++)
		for (j = 0; j < w; j++)
		{
			if (i < h - 1) cloth->CreateSpring(SPRING_STRUCT, i * w + j, (i + 1) * w + j);                    // structural
			if (j < w - 1) cloth->CreateSpring(SPRING_STRUCT, i * w + j, i * w + (j + 1));                    // structural
			if (j < w - 1 && i < h - 1) cloth->CreateSpring(SPRING_SHEAR, i * w + j, (i + 1) * w + (j + 1));  // shear
			if (j > 0 && i < h - 1) cloth->CreateSpring(SPRING_SHEAR, i * w + j, (i + 1) * w + (j - 1));      // shear
			if (i < h - 2) cloth->CreateSpring(SPRING_BEND, i * w + j, (i + 2) * w + j);                      // benders
			if (j < w - 2) cloth->CreateSpring(SPRING_BEND, i * w + j, i * w + (j + 2));                      // benders
		}
	cloth->UpdateLimits();
	return cloth;
}

int cloth_tess = 20;
float3 cloth_spawnpoint(0, 3, 5.0f);

/*
static void ClothDrawSprings(Cloth *cloth)
{
       static const float3 color[3]={float3(1,1,0),float3(1,0,1),float3(0,1,1)};
       float3N &X = cloth->X;
       for(int i=0;i<cloth->springs.count;i++)
       {
               SpringNetwork::Spring &s = cloth->springs[i];
               extern void Line(const float3 &,const float3 &,const float3 &color_rgb);
               Line(X[s.a],X[s.b],color[s.type]);
       }
}
*/
int cloth_showsprings = 0;

void DoCloths()
{
	int i;
	for (i = 0; i < cloths.count; i++)
	{
		Cloth *cloth = cloths[i];

		//  cloth->Simulate((cloth->cloth_step<0)?DeltaT:cloth->cloth_step);
		//if(cloth_showsprings)
		//        ClothDrawSprings(cloth); // debug visualization
	}
}
