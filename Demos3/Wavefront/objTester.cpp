// Obj_loader.cpp : Defines the entry point for the console application.
//
#include <stdio.h>
#include "objLoader.h"

void printVector(obj_vector *v)
{
	printf("%.2f,", v->e[0] );
	printf("%.2f,", v->e[1] );
	printf("%.2f  ", v->e[2] );
}

int main(int argc, char **argv)
{
	objLoader *objData = new objLoader();
	objData->load("test.obj");

	printf("Number of vertices: %i\n", objData->vertexCount);
	printf("Number of vertex normals: %i\n", objData->normalCount);
	printf("Number of texture coordinates: %i\n", objData->textureCount);
	printf("\n");

	printf("Number of faces: %i\n", objData->faceCount);
	for(int i=0; i<objData->faceCount; i++)
	{
		obj_face *o = objData->faceList[i];
		printf(" face ");
		for(int j=0; j<3; j++)
		{
			printVector(objData->vertexList[ o->vertex_index[j] ]);
		}
		printf("\n");
	}

	printf("\n");

	printf("Number of spheres: %i\n", objData->sphereCount);
	for(int i=0; i<objData->sphereCount; i++)
	{
		obj_sphere *o = objData->sphereList[i];
		printf(" sphere ");
		printVector(objData->vertexList[ o->pos_index ]);
		printVector(objData->normalList[ o->up_normal_index ]);
		printVector(objData->normalList[ o->equator_normal_index ]);
		printf("\n");
	}

	printf("\n");

	printf("Number of planes: %i\n", objData->planeCount);
	for(int i=0; i<objData->planeCount; i++)
	{
		obj_plane *o = objData->planeList[i];
		printf(" plane ");
		printVector(objData->vertexList[ o->pos_index ]);
		printVector(objData->normalList[ o->normal_index]);
		printVector(objData->normalList[ o->rotation_normal_index]);
		printf("\n");
	}

	printf("\n");

	printf("Number of point lights: %i\n", objData->lightPointCount);
	for(int i=0; i<objData->lightPointCount; i++)
	{
		obj_light_point *o = objData->lightPointList[i];
		printf(" plight ");
		printVector(objData->vertexList[ o->pos_index ]);
		printf("\n");
	}

	printf("\n");

	printf("Number of disc lights: %i\n", objData->lightDiscCount);
	for(int i=0; i<objData->lightDiscCount; i++)
	{
		obj_light_disc *o = objData->lightDiscList[i];
		printf(" dlight ");
		printVector(objData->vertexList[ o->pos_index ]);
		printVector(objData->normalList[ o->normal_index ]);
		printf("\n");
	}

	printf("\n");

	printf("Number of quad lights: %i\n", objData->lightQuadCount);
	for(int i=0; i<objData->lightQuadCount; i++)
	{
		obj_light_quad *o = objData->lightQuadList[i];
		printf(" qlight ");
		printVector(objData->vertexList[ o->vertex_index[0] ]);
		printVector(objData->vertexList[ o->vertex_index[1] ]);
		printVector(objData->vertexList[ o->vertex_index[2] ]);
		printVector(objData->vertexList[ o->vertex_index[3] ]);
		printf("\n");
	}

	printf("\n");

	if(objData->camera != NULL)
	{
		printf("Found a camera\n");
		printf(" position: ");
		printVector(objData->vertexList[ objData->camera->camera_pos_index ]);
		printf("\n looking at: ");
		printVector(objData->vertexList[ objData->camera->camera_look_point_index ]);
		printf("\n up normal: ");
		printVector(objData->normalList[ objData->camera->camera_up_norm_index ]);
		printf("\n");
	}

	printf("\n");

	printf("Number of materials: %i\n", objData->materialCount);
	for(int i=0; i<objData->materialCount; i++)
	{
		obj_material *mtl = objData->materialList[i];
		printf(" name: %s", mtl->name);
		printf(" amb: %.2f ", mtl->amb[0]);
		printf("%.2f ", mtl->amb[1]);
		printf("%.2f\n", mtl->amb[2]);

		printf(" diff: %.2f ", mtl->diff[0]);
		printf("%.2f ", mtl->diff[1]);
		printf("%.2f\n", mtl->diff[2]);

		printf(" spec: %.2f ", mtl->spec[0]);
		printf("%.2f ", mtl->spec[1]);
		printf("%.2f\n", mtl->spec[2]);
		
		printf(" reflect: %.2f\n", mtl->reflect);
		printf(" trans: %.2f\n", mtl->trans);
		printf(" glossy: %i\n", mtl->glossy);
		printf(" shiny: %i\n", mtl->shiny);
		printf(" refact: %.2f\n", mtl->refract_index);

		printf(" texture: %s\n", mtl->texture_filename);
		printf("\n");
	}

	printf("\n");

	//vertex, normal, and texture test
	if(objData->textureCount > 2 && objData->normalCount > 2 && objData->faceCount > 2)
	{
		printf("Detailed face data:\n");

		for(int i=0; i<3; i++)
		{
			obj_face *o = objData->faceList[i];
			printf(" face ");
			for(int j=0; j<3; j++)
			{
				printf("%i/", o->vertex_index[j] );
				printf("%i/", o->texture_index[j] );
				printf("%i ", o->normal_index[j] );
			}
			printf("\n");
		}
	}

	return 0;

}



