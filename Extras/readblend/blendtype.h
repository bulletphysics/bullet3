#ifndef BLEND_TYPE_H
#define BLEND_TYPE_H
#include "abs-file.h"


#ifdef __cplusplus
extern "C" { 
#endif

typedef struct _BlendField BlendField;
struct _BlendField {
  char* field_bytes;
  int field_bytes_count;

  /* the offset into field_bytes at which each field of a
     structure begins.  this is so we can keep the data aligned
     correctly for various architectures.  a non-structure type
     is equivalent to a structure with a single field.
  */
  int* field_offsets;
  int field_offsets_count;
};
struct _BlendBlock {
  char tag[5];
  uint32_t blender_pointer;
  /*void* fixed_pointer;*/

  int type_index;
  /* a block is simply an array of its type as defined by type_index.
     array_entries is an array of pointers to each entry in the block's
     array.     
   */
  BlendField *array_entries;
  int array_entries_count;
  void* customData; /* for Link blocks, with custom data such as .jpg pictures etc */
  int customDataSize;
};


typedef struct _BlendBlock BlendBlock;

/* the types extracted from the Blender file */
typedef struct _BlendType BlendType;
struct _BlendType {
	char* name;
	int size;

	int is_struct;

	/* if is_struct... this defines the types of each of the structure fields */
	int fieldtypes_count;
	int* fieldtypes; /* type indices */
	int fieldnames_count;
	int* fieldnames; /* name indices */
};






/* the opaque BlendFile structure */
typedef struct _BlendFile{
	BlendType* types;
	int types_count;

	char* *names;
	int names_count;

	int* strc_indices;
	int strc_indices_count;

	BlendBlock *blocks;
	int blocks_count;

	int name_undef; /* index of the name we add specially for top blocks */

} BlendFile;




#ifdef __cplusplus
}
#endif

#endif //BLEND_TYPE_H