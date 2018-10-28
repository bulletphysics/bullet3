/// obj2sdf will load a Wavefront .obj file that may contain many parts/materials
/// it will split into separate obj files for each part/material and
/// create an sdf file with visuals/collisions pointing to the new obj files
/// this will make it easier to load complex obj files into pybullet
/// see for example export in data/kitchens/fathirmutfak.sdf

#include <string.h>
#include <stdio.h>
#include <assert.h>
#define ASSERT_EQ(a, b) assert((a) == (b));
#include "Wavefront/tiny_obj_loader.h"
#include <vector>
#include "Bullet3Common/b3FileUtils.h"
#include "../Utils/b3ResourcePath.h"
#include "Bullet3Common/b3CommandLineArgs.h"
#include "Bullet3Common/b3HashMap.h"
#include "../Utils/b3BulletDefaultFileIO.h"

struct ShapeContainer
{
	std::string m_matName;
	std::string m_shapeName;
	tinyobj::material_t material;
	std::vector<float> positions;
	std::vector<float> normals;
	std::vector<float> texcoords;
	std::vector<unsigned int> indices;

	b3AlignedObjectArray<int> m_shapeIndices;
};

b3HashMap<b3HashString, ShapeContainer> gMaterialNames;

#define MAX_PATH_LEN 1024

std::string StripExtension(const std::string& sPath)
{
	for (std::string::const_reverse_iterator i = sPath.rbegin(); i != sPath.rend(); i++)
	{
		if (*i == '.')
		{
			return std::string(sPath.begin(), i.base() - 1);
		}

		// if we find a slash there is no extension
		if (*i == '\\' || *i == '/')
			break;
	}

	// we didn't find an extension
	return sPath;
}

int main(int argc, char* argv[])
{
	b3CommandLineArgs args(argc, argv);
	char* fileName;
	args.GetCmdLineArgument("fileName", fileName);
	if (fileName == 0)
	{
		printf("required --fileName=\"name\"");
		exit(0);
	}
	std::string matLibName = StripExtension(fileName);

	printf("fileName = %s\n", fileName);
	if (fileName == 0)
	{
		printf("Please use --fileName=\"pathToObj\".");
		exit(0);
	}
	bool mergeMaterials = args.CheckCmdLineFlag("mergeMaterials");

	char fileNameWithPath[MAX_PATH_LEN];
	bool fileFound = (b3ResourcePath::findResourcePath(fileName, fileNameWithPath, MAX_PATH_LEN,0)) > 0;
	char materialPrefixPath[MAX_PATH_LEN];
	b3FileUtils::extractPath(fileNameWithPath, materialPrefixPath, MAX_PATH_LEN);

	std::vector<tinyobj::shape_t> shapes;

	b3BulletDefaultFileIO fileIO;
	std::string err = tinyobj::LoadObj(shapes, fileNameWithPath, materialPrefixPath,&fileIO);

	char sdfFileName[MAX_PATH_LEN];
	sprintf(sdfFileName, "%s%s.sdf", materialPrefixPath, "newsdf");
	FILE* sdfFile = fopen(sdfFileName, "w");
	if (sdfFile == 0)
	{
		printf("Fatal error: cannot create sdf file %s\n", sdfFileName);
		exit(0);
	}

	fprintf(sdfFile, "<sdf version='1.6'>\n\t<world name='default'>\n\t<gravity>0 0 -9.8</gravity>\n");

	for (int s = 0; s < (int)shapes.size(); s++)
	{
		tinyobj::shape_t& shape = shapes[s];
		tinyobj::material_t mat = shape.material;

		b3HashString key = mat.name.length() ? mat.name.c_str() : "";
		if (!gMaterialNames.find(key))
		{
			ShapeContainer container;
			container.m_matName = mat.name;
			container.m_shapeName = shape.name;
			container.material = mat;
			gMaterialNames.insert(key, container);
		}

		ShapeContainer* shapeC = gMaterialNames.find(key);
		if (shapeC)
		{
			shapeC->m_shapeIndices.push_back(s);

			int curPositions = shapeC->positions.size() / 3;
			int curNormals = shapeC->normals.size() / 3;
			int curTexcoords = shapeC->texcoords.size() / 2;

			int faceCount = shape.mesh.indices.size();
			int vertexCount = shape.mesh.positions.size();
			for (int v = 0; v < vertexCount; v++)
			{
				shapeC->positions.push_back(shape.mesh.positions[v]);
			}
			int numNormals = int(shape.mesh.normals.size());
			for (int vn = 0; vn < numNormals; vn++)
			{
				shapeC->normals.push_back(shape.mesh.normals[vn]);
			}
			int numTexCoords = int(shape.mesh.texcoords.size());
			for (int vt = 0; vt < numTexCoords; vt++)
			{
				shapeC->texcoords.push_back(shape.mesh.texcoords[vt]);
			}

			for (int face = 0; face < faceCount; face += 3)
			{
				if (face < 0 && face >= int(shape.mesh.indices.size()))
				{
					continue;
				}

				shapeC->indices.push_back(shape.mesh.indices[face] + curPositions);
				shapeC->indices.push_back(shape.mesh.indices[face + 1] + curPositions);
				shapeC->indices.push_back(shape.mesh.indices[face + 2] + curPositions);
			}
		}
	}

	printf("unique materials=%d\n", gMaterialNames.size());

	if (mergeMaterials)
	{
		for (int m = 0; m < gMaterialNames.size(); m++)
		{
			if (gMaterialNames.getAtIndex(m)->m_shapeIndices.size() == 0)
				continue;

			ShapeContainer* shapeCon = gMaterialNames.getAtIndex(m);

			printf("object name = %s\n", shapeCon->m_shapeName.c_str());

			char objSdfPartFileName[MAX_PATH_LEN];
			sprintf(objSdfPartFileName, "part%d.obj", m);

			char objFileName[MAX_PATH_LEN];
			if (strlen(materialPrefixPath) > 0)
			{
				sprintf(objFileName, "%s/part%d.obj", materialPrefixPath, m);
			}
			else
			{
				sprintf(objFileName, "part%d.obj", m);
			}

			FILE* f = fopen(objFileName, "w");
			if (f == 0)
			{
				printf("Fatal error: cannot create part obj file %s\n", objFileName);
				exit(0);
			}
			fprintf(f, "# Exported using automatic converter by Erwin Coumans\n");
			if (matLibName.length())
			{
				fprintf(f, "mtllib %s.mtl\n", matLibName.c_str());
			}
			else
			{
				fprintf(f, "mtllib bedroom.mtl\n");
			}

			int faceCount = shapeCon->indices.size();
			int vertexCount = shapeCon->positions.size();
			tinyobj::material_t mat = shapeCon->material;
			if (shapeCon->m_matName.length())
			{
				const char* objName = shapeCon->m_matName.c_str();
				printf("mat.name = %s\n", objName);
				fprintf(f, "#object %s\n\n", objName);
			}
			for (int v = 0; v < vertexCount / 3; v++)
			{
				fprintf(f, "v %f %f %f\n", shapeCon->positions[v * 3 + 0], shapeCon->positions[v * 3 + 1], shapeCon->positions[v * 3 + 2]);
			}

			if (mat.name.length())
			{
				fprintf(f, "usemtl %s\n", mat.name.c_str());
			}
			else
			{
				fprintf(f, "usemtl wire_028089177\n");
			}

			fprintf(f, "\n");
			int numNormals = int(shapeCon->normals.size());

			for (int vn = 0; vn < numNormals / 3; vn++)
			{
				fprintf(f, "vn %f %f %f\n", shapeCon->normals[vn * 3 + 0], shapeCon->normals[vn * 3 + 1], shapeCon->normals[vn * 3 + 2]);
			}

			fprintf(f, "\n");
			int numTexCoords = int(shapeCon->texcoords.size());
			for (int vt = 0; vt < numTexCoords / 2; vt++)
			{
				fprintf(f, "vt %f %f\n", shapeCon->texcoords[vt * 2 + 0], shapeCon->texcoords[vt * 2 + 1]);
			}

			fprintf(f, "s off\n");

			for (int face = 0; face < faceCount; face += 3)
			{
				if (face < 0 && face >= int(shapeCon->indices.size()))
				{
					continue;
				}
				fprintf(f, "f %d/%d/%d %d/%d/%d %d/%d/%d\n",
						shapeCon->indices[face] + 1, shapeCon->indices[face] + 1, shapeCon->indices[face] + 1,
						shapeCon->indices[face + 1] + 1, shapeCon->indices[face + 1] + 1, shapeCon->indices[face + 1] + 1,
						shapeCon->indices[face + 2] + 1, shapeCon->indices[face + 2] + 1, shapeCon->indices[face + 2] + 1);
			}
			fclose(f);

			float kdRed = mat.diffuse[0];
			float kdGreen = mat.diffuse[1];
			float kdBlue = mat.diffuse[2];
			float transparency = mat.transparency;

			fprintf(sdfFile,
					"\t\t<model name='%s'>\n"
					"\t\t\t<static>1</static>\n"
					"\t\t\t<pose frame=''>0 0 0 0 0 0</pose>\n"
					"\t\t\t<link name='link_d%d'>\n"
					"\t\t\t<inertial>\n"
					"\t\t\t<mass>0</mass>\n"
					"\t\t\t<inertia>\n"
					"\t\t\t<ixx>0.166667</ixx>\n"
					"\t\t\t<ixy>0</ixy>\n"
					"\t\t\t<ixz>0</ixz>\n"
					"\t\t\t<iyy>0.166667</iyy>\n"
					"\t\t\t<iyz>0</iyz>\n"
					"\t\t\t<izz>0.166667</izz>\n"
					"\t\t\t</inertia>\n"
					"\t\t\t</inertial>\n"
					"\t\t\t<collision concave='yes' name='collision_%d'>\n"
					"\t\t\t<geometry>\n"
					"\t\t\t<mesh>\n"
					"\t\t\t<scale>1 1 1</scale>\n"
					"\t\t\t\t<uri>%s</uri>\n"
					"\t\t\t</mesh>\n"
					"\t\t\t</geometry>\n"
					"\t\t\t  </collision>\n"
					"\t\t\t<visual name='visual'>\n"
					"\t\t\t\t<geometry>\n"
					"\t\t\t\t<mesh>\n"
					"\t\t\t\t\t<scale>1 1 1</scale>\n"
					"\t\t\t\t\t<uri>%s</uri>\n"
					"\t\t\t\t</mesh>\n"
					"\t\t\t\t</geometry>\n"
					"\t\t\t<material>\n"
					"\t\t\t\t<ambient>1 0 0 1</ambient>\n"
					"\t\t\t\t<diffuse>%f %f %f %f</diffuse>\n"
					"\t\t\t\t<specular>0.1 0.1 0.1 1</specular>\n"
					"\t\t\t\t<emissive>0 0 0 0</emissive>\n"
					"\t\t\t </material>\n"
					"\t\t\t </visual>\n"
					"\t\t\t </link>\n"
					"\t\t\t</model>\n",
					objSdfPartFileName, m, m,
					objSdfPartFileName, objSdfPartFileName,
					kdRed, kdGreen, kdBlue, transparency);
		}
	}
	else
	{
		for (int s = 0; s < (int)shapes.size(); s++)
		{
			tinyobj::shape_t& shape = shapes[s];

			if (shape.name.length())
			{
				printf("object name = %s\n", shape.name.c_str());
			}

			char objFileName[MAX_PATH_LEN];
			if (strlen(materialPrefixPath) > 0)
			{
				sprintf(objFileName, "%s/part%d.obj", materialPrefixPath, s);
			}
			else
			{
				sprintf(objFileName, "part%d.obj", s);
			}
			FILE* f = fopen(objFileName, "w");
			if (f == 0)
			{
				printf("Fatal error: cannot create part obj file %s\n", objFileName);
				exit(0);
			}
			fprintf(f, "# Exported using automatic converter by Erwin Coumans\n");
			if (matLibName.length())
			{
				fprintf(f, "mtllib %s.mtl\n", matLibName.c_str());
			}
			else
			{
				fprintf(f, "mtllib bedroom.mtl\n");
			}

			int faceCount = shape.mesh.indices.size();
			int vertexCount = shape.mesh.positions.size();
			tinyobj::material_t mat = shape.material;
			if (shape.name.length())
			{
				const char* objName = shape.name.c_str();
				printf("mat.name = %s\n", objName);
				fprintf(f, "#object %s\n\n", objName);
			}
			for (int v = 0; v < vertexCount / 3; v++)
			{
				fprintf(f, "v %f %f %f\n", shape.mesh.positions[v * 3 + 0], shape.mesh.positions[v * 3 + 1], shape.mesh.positions[v * 3 + 2]);
			}

			if (mat.name.length())
			{
				fprintf(f, "usemtl %s\n", mat.name.c_str());
			}
			else
			{
				fprintf(f, "usemtl wire_028089177\n");
			}

			fprintf(f, "\n");
			int numNormals = int(shape.mesh.normals.size());

			for (int vn = 0; vn < numNormals / 3; vn++)
			{
				fprintf(f, "vn %f %f %f\n", shape.mesh.normals[vn * 3 + 0], shape.mesh.normals[vn * 3 + 1], shape.mesh.normals[vn * 3 + 2]);
			}

			fprintf(f, "\n");
			int numTexCoords = int(shape.mesh.texcoords.size());
			for (int vt = 0; vt < numTexCoords / 2; vt++)
			{
				fprintf(f, "vt %f %f\n", shape.mesh.texcoords[vt * 2 + 0], shape.mesh.texcoords[vt * 2 + 1]);
			}

			fprintf(f, "s off\n");

			for (int face = 0; face < faceCount; face += 3)
			{
				if (face < 0 && face >= int(shape.mesh.indices.size()))
				{
					continue;
				}
				fprintf(f, "f %d/%d/%d %d/%d/%d %d/%d/%d\n",
						shape.mesh.indices[face] + 1, shape.mesh.indices[face] + 1, shape.mesh.indices[face] + 1,
						shape.mesh.indices[face + 1] + 1, shape.mesh.indices[face + 1] + 1, shape.mesh.indices[face + 1] + 1,
						shape.mesh.indices[face + 2] + 1, shape.mesh.indices[face + 2] + 1, shape.mesh.indices[face + 2] + 1);
			}
			fclose(f);

			float kdRed = mat.diffuse[0];
			float kdGreen = mat.diffuse[1];
			float kdBlue = mat.diffuse[2];
			float transparency = mat.transparency;
			char objSdfPartFileName[MAX_PATH_LEN];
			sprintf(objSdfPartFileName, "part%d.obj", s);
			fprintf(sdfFile,
					"\t\t<model name='%s'>\n"
					"\t\t\t<static>1</static>\n"
					"\t\t\t<pose frame=''>0 0 0 0 0 0</pose>\n"
					"\t\t\t<link name='link_d%d'>\n"
					"\t\t\t<inertial>\n"
					"\t\t\t<mass>0</mass>\n"
					"\t\t\t<inertia>\n"
					"\t\t\t<ixx>0.166667</ixx>\n"
					"\t\t\t<ixy>0</ixy>\n"
					"\t\t\t<ixz>0</ixz>\n"
					"\t\t\t<iyy>0.166667</iyy>\n"
					"\t\t\t<iyz>0</iyz>\n"
					"\t\t\t<izz>0.166667</izz>\n"
					"\t\t\t</inertia>\n"
					"\t\t\t</inertial>\n"
					"\t\t\t<collision name='collision_%d'>\n"
					"\t\t\t<geometry>\n"
					"\t\t\t<mesh>\n"
					"\t\t\t<scale>1 1 1</scale>\n"
					"\t\t\t\t<uri>%s</uri>\n"
					"\t\t\t</mesh>\n"
					"\t\t\t</geometry>\n"
					"\t\t\t  </collision>\n"
					"\t\t\t<visual name='visual'>\n"
					"\t\t\t\t<geometry>\n"
					"\t\t\t\t<mesh>\n"
					"\t\t\t\t\t<scale>1 1 1</scale>\n"
					"\t\t\t\t\t<uri>%s</uri>\n"
					"\t\t\t\t</mesh>\n"
					"\t\t\t\t</geometry>\n"
					"\t\t\t<material>\n"
					"\t\t\t\t<ambient>1 0 0 1</ambient>\n"
					"\t\t\t\t<diffuse>%f %f %f %f</diffuse>\n"
					"\t\t\t\t<specular>0.1 0.1 0.1 1</specular>\n"
					"\t\t\t\t<emissive>0 0 0 0</emissive>\n"
					"\t\t\t </material>\n"
					"\t\t\t </visual>\n"
					"\t\t\t </link>\n"
					"\t\t\t</model>\n",
					objSdfPartFileName, s, s,
					objSdfPartFileName, objSdfPartFileName,
					kdRed, kdGreen, kdBlue, transparency);
		}
	}
	fprintf(sdfFile, "\t</world>\n</sdf>\n");

	fclose(sdfFile);

	return 0;
}