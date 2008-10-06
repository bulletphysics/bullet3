/***************************************************************
 * readblend.h --
 * ReadBlend, a data extraction API for Blender's .blend files
 *
 * (c) 2003 Adam D. Moss <adam@gimp.org> <aspirin@icculus.org>
 *
 */

/* VERSION HISTORY
 * 2003-04-05 : v1.0
 * 2008-10-05 : v1.1-beta
 */

/* Blender files are 'curiously' structured and this is mirrored in
 * the low-level data access API (the section labelled 'Low-level
 * querying functions').
 *
 * The mid-level data access API (the section labelled 'Mid-level
 * querying functions') tries to provide some handy utilities to
 * ease your data-extraction pain, and finally the (partially written)
 * high-level data access API incorporates semantic knowledge of Blender's
 * useful high-level types such as Meshes and Materials to shield you
 * from the full horror.
 */
#ifndef _READBLEND_H
#define _READBLEND_H

#include <stdlib.h>
//#include <stdint.h>
#include <math.h>
#include <stdlib.h>//size_t for MSVC 6.0

//#include <cfloat>
#include <float.h>

#include "abs-file.h"

/* TODO: Doxygen me. */


/* don't worry yourself about this. */
#ifndef BlendFile
#define BlendFile void
#endif




/**************************************/
/* .blend handle load/free functions */
/*************************************/


/* Pass in an already-opened file handle; this function will then
   examine the file to check that it is a Blender file and scan the
   entire file to extract all vital information (quite expensive)
   before any query operations can be performed.  The resulting
   .blend data handle is returned.
*/

BlendFile* blend_read(MY_FILETYPE* file);


/* Free all of the given BlendFile data (note: obviously, do not attempt
   to extract data from a BlendFile which has been freed!) */

void blend_free(BlendFile* blend_file);


/********************************/
/* Public data querying types   */
/********************************/


/* heed this enum well. */
typedef enum {
  /* you're unlikely to encounter these in the wild. */
  BLEND_OBJ_NULL,
  BLEND_OBJ_OPAQUE,

  /* these object types are fetchable as-is */
  BLEND_OBJ_UCHAR8,
  BLEND_OBJ_CHAR8,
  BLEND_OBJ_USHORT16,
  BLEND_OBJ_SHORT16,
  BLEND_OBJ_ULONG32,
  BLEND_OBJ_LONG32,
  BLEND_OBJ_FLOAT,
  BLEND_OBJ_DOUBLE,
  BLEND_OBJ_POINTER,

  /* you'll need to extract the fields from these individually */
  BLEND_OBJ_STRUCT
} BlendObjType;

typedef void* BlendBlockPointer;

/* note: treat this as an opaque type and you'll live longer. */
typedef struct {
  long type;   long name;   BlendBlockPointer block;
  long entry_index;   long field_index;
} BlendObject;

/* The callback type for passing to blend_foreach_block() (the callback
   should return zero if it doesn't want to see any more blocks.) */
#define BLENDBLOCKCALLBACK_RETURN int
#define BLENDBLOCKCALLBACK_ARGS BlendBlockPointer block, \
				BlendFile* blend_file, \
				void* userdata
typedef BLENDBLOCKCALLBACK_RETURN(BlendBlockCallback)
				  (BLENDBLOCKCALLBACK_ARGS);


/********************************/
/* File info dumping functions  */
/********************************/

/* these functions simply print dumps of information about the given
   BlendFile.  These are vital for familiarising yourself with the
   structure of Blender's various scene objects if you're using the
   low/mid-level APIs to construct queries to extract information about
   specific types of objects that haven't been conveniently abstracted by
   the high-level API yet.  (That is, most of them right now, particularly
   the more obscure or deprecated ones.) */

/* print out the size, field-types and field-names of all of the varieties
   of types (atomic and struct) represented within this BlendFile.   The
   sizes are the in-file representations; the in-memory representations
   are potentially subject to additional padding and conversions.
*/
void blend_dump_typedefs(BlendFile* bf);

/* For every top-level block of data in the file, print the block's
   tag, its in-memory address, its type, and the number of entries of that
   type.  (Every pointer in a structure should point to one of these
   in-memory block addresses because we convert from in-file pointers to
   in-memory pointers automagically; a few in-file pointers within structures
   do actually point somewhere into the middle of a block or nowhere, which we
   don't currently support and silently convert to NULL.)  Also, any structs
   at the top-level with an id->name have that name printed out. */
void blend_dump_blocks(BlendFile* bf);


/********************************/
/* Low-level querying functions */
/********************************/


/* Calls the user-supplied callback function for every top-level
   data block in the .blend file.  You can use this to count blocks
   (which is pretty pointless) or search for a particular block you're
   interested in. */
void blend_foreach_block(BlendFile* blend_file,
			 BlendBlockCallback* func,
			 void* userdata);

/* Returns the tag-name ('IM', 'DATA', 'ME', etc) for the given
   top-level data block. */
const char* blend_block_get_tagname(BlendFile* blend_file,
				    BlendBlockPointer block);

/* Returns the type of the given top-level data block in raw
   string form ('uchar', 'float', 'MFace', 'Mesh', etc). */
const char* blend_block_get_typename(BlendFile* blend_file,
				     BlendBlockPointer block);

/* Translate a pointer from the file's raw data into a BlendBlockPointer
   that you can query via the API.  You will usually NOT need to ever
   use this unless you're manually extracting pointers from opaque raw data
   types.  Returns NULL on failure or if the input pointer is really NULL. */
BlendBlockPointer blend_block_from_blendpointer(BlendFile *blend_file,
						uint32_t blendpointer);

/* Returns the number of entries there are in the given top-level
   data block (a top-level data block is like an array of entries of a specific
   Blender type, this type usually being one of Blender's structs). */
int blend_block_get_entry_count(BlendFile* blend_file,
				BlendBlockPointer block);

/* This gets an object handle on the Nth piece of data (in the range
   0..TOTAL-1, where TOTAL is the figure returned by
   blend_block_get_entry_count() ) in the given top-level block. */
BlendObject blend_block_get_object(BlendFile* blend_file,
				   BlendBlockPointer block,
				   int entry_index);

/* Returns a BlendObjType enum handy for checking that the general type
   of the object you have a handle on is what you're expecting. */
BlendObjType blend_object_type(BlendFile* blend_file,
			       BlendObject obj);

/* Given a BlendObject of type BLEND_OBJ_STRUCT in 'obj', fill in the
   BlendObject pointed to by 'result' with a handle to the named
   field of that structure (note that the resulting data object might
   itself be a structure!).  0 is returned on failure (i.e. this structure
   does not have a field of the requested name, or you supplied an
   object which is not a structure). */
int blend_object_structure_getfield(BlendFile* blend_file,
				    BlendObject *result,
				    BlendObject obj,
				    const char* field_name);

/* Gets the size (in elements) of an object which is a 1D or 2D array.
   A non-array is equivalent to an array of size 1x1.  A 1D array will
   always have dim1 == 1. */
void blend_object_array_getdims(BlendFile* blend_file,
				BlendObject obj,
				int* dim1, int* dim2);

/* This fetches a piece of data from the .blend file in a format
   suitable for your architecture (i.e. ints will be of proper size
   and endianness, pointers are transformed to valid BlendBlockPointers
   or NULL, etc) into the address pointed to by dest.  It's up to you to
   check that the data you're asking for will fit into the type you're
   trying to put it in (use blend_object_type() and see the BlendObjType
   enum to check that the object's type is the one you're expecting.) 

   Composite structures (BLEND_OBJ_STRUCT) are not handled atomically;
   use blend_object_structure_getfield() to extract each named field from
   a structure individually.

   1 is returned on success.  Failure occurs when you try to extract
   data from a structure (see blend_object_structure_getfield()) or
   an array (use blend_object_array_getdata()).
*/
int blend_object_getdata(BlendFile* blend_file,
			 void* dest, BlendObject obj);

/* This operates like blend_object_getdata() except that it is happy
   to copy an item of data out of an array.  The array is always treated
   as two-dimensional (data[dimension1][dimension2]); if you're accessing
   a one-dimensional array then simply specify dim_index_1 as 0.
   (Indices are in the range 0..DIMSIZE-1.)

   Similarly, plain non-array data can be fetched by specifying
   dim_index_1 == dim_index_2 == 0.  This makes
   blend_object_array_getdata(bf, dest, obj, 0, 0) equivalent to
   blend_object_getdata(bf, dest, obj) except that it is happy to
   be called with an array, from which it will extract the first
   element.

   Like blend_object_getdata(), it will not fetch a structure (from
   an array of structures; fortunately I've not seen a .blend file
   featuring an array of structures so we'll bridge that API gap
   when we come to it).
 */
int blend_object_array_getdata(BlendFile* blend_file,
			       void* dest, BlendObject obj,
			       int dim_index_1, int dim_index_2);

/********************************/
/* Mid-level querying functions */
/********************************/

typedef unsigned long BlendLayerMask;

/* extract a string from a char-array or uchar-array object, up to
   max_chars in length including the \0 terminator.  If the object is
   a two-dimensional array then the first string is extracted.  Returns
   0 on failure.
*/
int blend_object_getstring(BlendFile* blend_file,
			   BlendObject obj,
			   char *dest, int max_chars);

/* Searches the file for a top-level object with the given ID name.
   Typical ID names are 'OBCamera', 'MECircle.003', 'MAplastic', 'TEgrass' etc.
   The first two characters of an ID name keep the namespaces separate,
   so that a material (MA) with the name 'metal' is distinguishable from
   a texture (TE) with the name 'metal'.  That's a foible of Blender itself.

   blend_object_get_by_IDname() returns 0 on failure.  It only returns one
   object of the given IDname; there should indeed be only one, as Blender
   enforces this uniqueness.
 */
int blend_object_get_by_IDname(BlendFile* blend_file,
			       BlendObject *result,
			       const char* IDname);
/* get string from [obj].ID.name -- caller allocs/frees */
int blend_object_get_IDname(BlendFile* blend_file,
                            BlendObject obj,
                            char *dest, int max_chars);

/* returns !0 if the blender Object whose handle is objobj is a top-level
   object -- that is, it's not a child of another Object. */
int blend_obj_is_rootobject(BlendFile *bf, BlendObject *objobj);
/* return the layers (bitmask) that an Object lives on. */
BlendLayerMask blend_obj_get_layermask(BlendFile *bf, BlendObject *objobj);

/* These functions can be a bit slow -- each one requires a linear scan
   of the file's blocks.  But they're handy... */
/* Return number of children that this Object has.  Can be 0. */
int blend_obj_get_childcount(BlendFile *bf, BlendObject *objobj);
/* Gets child number childnum of the Object.  childnum=0 returns
   the first, childnum=1 returns the second, etc. */
BlendBlockPointer blend_obj_get_child(BlendFile *bf, BlendObject *objobj,
                                      int childnum);

/*********************************/
/* High-level querying functions */
/*********************************/

typedef struct {
  int bone_id;
  float weight;
} bDeformWeight;

typedef struct {
  float xyz[3];
  float cnormal[3]; /* cosmetic normal */
  int mat;
  bDeformWeight* deform_weights;
  int deform_weights_count;
} bVert;
#define BVERT_HAS_CNORMAL(BV) ((BV)->cnormal[0] != 0.0f || \
			       (BV)->cnormal[1] != 0.0f || \
			       (BV)->cnormal[2] != 0.0f)

#define BFACE_FLAG_SMOOTH 0x01
typedef struct {
  int v[4];
  float rgba[4][4];  /* vertex colours */
  float rgba2[4][4]; /* texture face colours (errrr...?) */
  float uv[4][2];
  BlendBlockPointer image_id;
  int mat;
  unsigned char flags;
} bFace;
#define BFACE_HAS_TEXTURE(BF) ((BF)->image_id != NULL)
#define BFACE_IS_QUAD(BF) ((BF)->v[3] != 0)
#define BFACE_IS_TRI(BF) ((BF)->v[2] != 0 && (BF)->v[3] == 0)
#define BFACE_IS_LINE(BF) ((BF)->v[1] != 0 && (BF)->v[2] == 0)

typedef enum {
  BTEX_AFFECT_COLOUR  = 0x01,
  BTEX_AFFECT_ALPHA   = 0x02,
  BTEX_AFFECT_EMIT    = 0x04,
  BTEX_AFFECT_NORMAL  = 0x08,
  BTEX_AFFECT_NEGNORM = 0x10,
  BTEX_AFFECT_STENCIL = 0x20 /* not really an 'affect' in blender, but it is */
} bTexLayerAffects;

typedef enum {
  BTEX_BLEND_NORMAL, /* 'mix' */
  BTEX_BLEND_MULTIPLY, /* 'mul' -- modulate. */
  BTEX_BLEND_ADD,
  BTEX_BLEND_SUB
} bTexLayerBlendmode;

typedef enum {
  BTEX_COORDS_NONE,
  BTEX_COORDS_UV,
  BTEX_COORDS_REFLECT
} bTexLayerCoordsType;

typedef enum {
  BIMG_FLAG_INTERPOLATE = 0x01, /* upsample */
  BIMG_FLAG_ANTIALIAS   = 0x02, /* downsample */
  BIMG_FLAG_MIPMAP      = 0x04  /* use mipmaps */
} bTexImageFlags;

typedef struct {
  char *filename; /* image file name -- absolute path! -- NULL if not image. */
  bTexLayerAffects    affects_mask;
  bTexLayerBlendmode  blend_mode;
  bTexLayerCoordsType coords_type;
  /* Tex substruct */
  int is_st_clamped; /* otherwise, repeat... (GL texwrap mode.) */
  bTexImageFlags flags;
  short Nflags, Ntype; /* not decoded yet */
  short xrepeat, yrepeat; /* amounts to scale texcoords by, really */
} bTexLayer;

typedef enum {
  BMAT_FEATURE_VCOLLIGHT = 0x01,
  BMAT_FEATURE_VCOLPAINT = 0x02,
  BMAT_FEATURE_TEXFACE   = 0x04,
  BMAT_FEATURE_SHADELESS = 0x08,
  BMAT_FEATURE_WIRE      = 0x10 /* wireframe rendering */
} bMatFeatureMask;

#define BLENDER_MAX_TEX_LAYERS 8
typedef struct {
  bTexLayer *tex_layer[BLENDER_MAX_TEX_LAYERS];
  bMatFeatureMask feature_mask;
  float colour_rgba[4]; /* main material colour */
  float emit; /* emissive strength, 0..1 */
} bMaterial;

typedef float bMatrix[4][4];

typedef struct {
  bVert *vert;
  int vert_count;
  bFace *face;
  int face_count;
  bMaterial *material; /* or NULL if none */
} bMesh;

typedef enum {
  BOBJ_TYPE_UNKNOWN,
  BOBJ_TYPE_NULL, /* indicates object has no data associated with it! */
  BOBJ_TYPE_MESH
} bObjType;

typedef enum {
  BAQ_INCLUDE_CHILDREN = 0x0001
} bAcquireFlags;

typedef struct {
  bObjType type;
  char *name;

  bMatrix transform; /* local transformation matrix */
  bMatrix parentimat; /* parent's inverse transform matrix */
  float scaling[3]; /* scaling component of transform */
  float rotphr[3]; /* pitch/head/roll rotation component of transform (use for normals) */
  float location[3]; /* location component of transform */
  unsigned char transflags; /* NOT DECODED YET, RAW BYTE */

  union {
    void  *dummy;
    bMesh *mesh;
  } data;
} bObj;


bObj *blend_alloc_obj(void);
void blend_init_obj(bObj *obj);
void blend_free_obj(bObj *obj);
void blend_free_obj_inner(bObj *obj);
void blend_acquire_obj_from_obj(BlendFile *bf, BlendObject *objobj,
                                bObj *outobj,
                                /* malloc'd BlendObject will be written here if pointer if non-NULL: */ BlendObject **outblendobject);

bMesh *blend_alloc_mesh(void);
void blend_init_mesh(bMesh *mesh);
void blend_free_mesh_inner(bMesh *mesh);
void blend_free_mesh(bMesh *mesh);
void blend_acquire_mesh(const char *fname, const char *want_name, bMesh *mesh);
void blend_acquire_mesh_from_obj(BlendFile *bf,
                                 BlendObject *meobj,
                                 bMesh *mesh);
/* apply transformation found in 'obj' to 'mesh' */
void blend_transform_mesh_from_obj(bMesh *mesh, bObj *obj);

bMaterial *blend_alloc_material(void);
void blend_init_material(bMaterial *mat);
void blend_free_material(bMaterial *mat);
void blend_acquire_material_from_obj(BlendFile *bf, BlendObject *meobj,
                                     bMaterial *mat);

bTexLayer *blend_alloc_texlayer(void);
void blend_init_texlayer(bTexLayer *tl);
void blend_free_texlayer(bTexLayer *tl);
void blend_acquire_texlayer_from_obj(BlendFile *bf, BlendObject *tlobj,
                                     bTexLayer *tl);

#endif
