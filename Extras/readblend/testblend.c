#include <stdlib.h>
#include <stdio.h>

#include "abs-file.h"

#include "readblend.h"
#include "blendtype.h"

#define MAX_MESHES 10
bMesh	gMeshes[MAX_MESHES];
int gNumMesh = 0;

#define MAX_OBJECTS 10
bObj gObjects[MAX_OBJECTS];
int gNumObjects = 0;

void crawl(BlendFile* blend_file,
		   BlendObject obj)
{
	BlendObject data_obj;
	BlendObject data_obj2;
	
	BlendBlock* tmpBlock=0;

	{
		const char* type_name = blend_file->types[obj.type].name;
		if (strcmp(type_name,"Object")==0)
		{
			if (gNumObjects<MAX_OBJECTS)
			{
				blend_acquire_obj_from_obj(blend_file,&obj,&gObjects[gNumObjects],0);
				gNumObjects++;
			}
		}

		if (strcmp(type_name,"Mesh")==0)
		{
			printf("object type_name = %s\n",type_name);
			if (gNumMesh<MAX_MESHES)
			{
				blend_acquire_mesh_from_obj(blend_file, &obj, &gMeshes[gNumMesh]);
				gNumMesh++;
			}
		}
		
	}

	///todo: compress those 4 lines into a one-liner, using a C++ wrapper
	if (blend_object_structure_getfield(blend_file, &data_obj, obj, "curscene")) {
		if (blend_object_type(blend_file, data_obj) == BLEND_OBJ_POINTER) {
			if (blend_object_getdata(blend_file, &tmpBlock, data_obj)) {
				BlendObject  blscene= blend_block_get_object(blend_file,tmpBlock,0);

				if (blend_object_structure_getfield(blend_file, &data_obj,blscene , "id")) 
				{
					if (blend_object_type(blend_file, data_obj) == BLEND_OBJ_STRUCT) 
					{
						if (blend_object_structure_getfield(blend_file, &data_obj2,data_obj , "name")) 
						{
							char dest[19];
							int max_chars=20;
							
							if (blend_object_getstring(blend_file, data_obj2,
                                   dest, max_chars)) 
							{
								printf("dest=%s\n",dest);
							}

						}

					}

				}

				if (blend_object_structure_getfield(blend_file, &data_obj,blscene , "world")) {
					if (blend_object_type(blend_file, data_obj) == BLEND_OBJ_POINTER) {
						if (blend_object_getdata(blend_file, &tmpBlock, data_obj)) {
							BlendObject blworld= blend_block_get_object(blend_file,tmpBlock,0);

							if (blend_object_structure_getfield(blend_file, &data_obj, blworld, "gravity")) 
							{
								printf("Scene with world and gravity\n");
								if (blend_object_type(blend_file, data_obj) == BLEND_OBJ_FLOAT) {
									float data1 = 123456.7;
									if (blend_object_getdata(blend_file, &data1, data_obj))
										fprintf(stderr, "gravity=(%f) ", data1);
								} else {
									fprintf(stderr, "FETCH ERROR\n");
								}

							}

						}
					}
				}


			}

		}
	}
}

static BLENDBLOCKCALLBACK_RETURN
blockiter(BLENDBLOCKCALLBACK_ARGS) {
	int i;
	int entry_count = blend_block_get_entry_count(blend_file, block);
#if 0
	const char* tagname = blend_block_get_tagname(blend_file, block);
	const char* typename = blend_block_get_typename(blend_file, block);
	fprintf(stderr, "block: tag=%s type=%s entries=%d ",
		tagname, typename, entry_count);
#endif
	for (i=0; i<entry_count; ++i) {
		BlendObject obj = blend_block_get_object(blend_file, block, i);
		crawl(blend_file, obj);
	}
	//fprintf(stderr, "\n");
	return 1;
}


int
main(int argc, char *argv[])
{
	MY_FILETYPE *file;
	BlendFile *bf;
	int rtn = 0;

	if (argc <= 1) {
		fprintf(stderr, "specify filename on command line.\n");
		return -1;
	}

	file = MY_OPEN_FOR_READ(argv[1]);

	if (!file) {
		fprintf(stderr, "couldn't open file. :(\n");
		return -1;
	}

	bf = blend_read(file);

	if (!bf) {
		fprintf(stderr, "couldn't read blender file. :(\n");
		rtn = -1;
		goto closeit;
	}

	 blend_dump_typedefs(bf);

	//  blend_dump_blocks(bf);

	fflush(stdout);

	blend_foreach_block(bf, blockiter, NULL);


	{
		BlendObject obj;
		char want_name[] = "IMmetalrock.jpg";
		if (blend_object_get_by_IDname(bf,&obj, want_name)) {
			fprintf(stderr, "got %s.\n", want_name);
		}
	}


	blend_free(bf);

closeit:
	MY_CLOSE(file);
	return rtn;
}
