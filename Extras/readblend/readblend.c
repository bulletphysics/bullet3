
#include <stdlib.h>
//#include <stdint.h>
#include <string.h>
#include <math.h>

#include "abs-file.h"


#define B_DEBUG


#ifdef B_DEBUG
# ifndef dprintf
#  include <stdio.h>
#  define dprintf fprintf
# endif
#else
# define dprintf
#endif


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


typedef struct _BlendBlock BlendBlock;
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
};


/* the opaque BlendFile structure */
typedef struct {
  BlendType* types;
  int types_count;
  
  char* *names;
  int names_count;

  int* strc_indices;
  int strc_indices_count;

  BlendBlock *blocks;
  int blocks_count;

  int name_undef; /* index of the name we add specially for top blocks */
  
} IBlendFile;


#define BlendFile IBlendFile
#include "readblend.h"


/* endianness conversion macros */
#define BGETLEUINT16(p) ((uint16_t) ( \
                        ( ((uint16_t)*( 1 + (uint8_t*)(p) ))<<8 ) | \
                        ( ((uint16_t)*( 0 + (uint8_t*)(p) ))<<0 ) \
                        ) )
#define BGETLEINT16(p) ((int16_t) BGETLEUINT16(p))
#define BGETLEUINT32(p) ((uint32_t) ( \
                        ( ((uint32_t)*( 3 + (uint8_t*)(p) ))<<24 ) | \
                        ( ((uint32_t)*( 2 + (uint8_t*)(p) ))<<16 ) | \
                        ( ((uint32_t)*( 1 + (uint8_t*)(p) ))<<8 ) | \
                        ( ((uint32_t)*( 0 + (uint8_t*)(p) ))<<0 ) \
                        ) )
#define BGETLEINT32(p) ((int32_t) BGETLEUINT32(p))
#define BGETLEUINT64(p) ((uint64_t) ( \
                        ( ((uint64_t)*( 7 + (uint8_t*)(p) ))<<56 ) | \
                        ( ((uint64_t)*( 6 + (uint8_t*)(p) ))<<48 ) | \
                        ( ((uint64_t)*( 5 + (uint8_t*)(p) ))<<40 ) | \
                        ( ((uint64_t)*( 4 + (uint8_t*)(p) ))<<32 ) | \
                        ( ((uint64_t)*( 3 + (uint8_t*)(p) ))<<24 ) | \
                        ( ((uint64_t)*( 2 + (uint8_t*)(p) ))<<16 ) | \
                        ( ((uint64_t)*( 1 + (uint8_t*)(p) ))<<8 ) | \
                        ( ((uint64_t)*( 0 + (uint8_t*)(p) ))<<0 ) \
                        ) )
#define BGETLEINT64(p) ((int64_t) BGETLEUINT64(p))
static float BGETLEFLOAT32(char *p) {
  union { /* use type-punning, or aliasing optimization will kick our arse */
    uint32_t u32;
    float f32;
  } punner;
  punner.u32 = BGETLEUINT32(p);
  return punner.f32;
}
static double BGETLEDOUBLE64(char *p) {
  union { /* use type-punning, or aliasing optimization will kick our arse */
    uint64_t u64;
    float f64;
  } punner;
  punner.u64 = BGETLEUINT64(p);
  return punner.f64;
}


static BlendFile*
bf_new(void)
{
  BlendFile *rtn = malloc(sizeof(BlendFile));
  if (!rtn) {
    dprintf(stderr, "out of mem making bf handle\n");
    return NULL;
  }
  rtn->types = NULL;
  rtn->types_count = 0;
  rtn->names = NULL;
  rtn->names_count = 0;
  rtn->blocks = NULL;
  rtn->blocks_count = 0;
  rtn->strc_indices = NULL;
  rtn->strc_indices_count = 0;
  rtn->name_undef = -1;
  return rtn;
}


static long
seek_past_string(MY_FILETYPE* file, const char *str) {
  const int match_max = strlen(str);
  int match_now = 0;

  do {
    const char c = MY_GETC(file);
    if (c == str[match_now]) {
      ++match_now;
    } else {
      match_now = 0;
    }
  } while(match_now < match_max &&
	  !MY_ATEOF(file));

  if (MY_ATEOF(file)) {
    return -1;
  }

  return MY_TELL(file) - match_max;
}

#define EXPANDO_MULTIPLE(ARRAY,ADDITIONPTR,NUMBER) \
do { \
  (ARRAY) = realloc((ARRAY), sizeof((ARRAY)[0]) * (NUMBER +(ARRAY##_count))); \
  memcpy(&(ARRAY)[ARRAY##_count], ADDITIONPTR, sizeof((ARRAY)[0]) * NUMBER); \
  (ARRAY##_count) += NUMBER; \
} while(0)

#define EXPANDO(ARRAY,ADDITION) \
do { \
  (ARRAY) = realloc((ARRAY), sizeof((ARRAY)[0]) * (1 + (ARRAY##_count))); \
  (ARRAY)[ARRAY##_count] = (ADDITION); \
  ++(ARRAY##_count); \
} while(0)


static unsigned short
read_ushort(MY_FILETYPE* file) {
  unsigned char c[2];
  if (MY_READ(c, 2, 1, file) == 1) {
    return c[0] | (c[1]<<8);
  } else {
    return 0xFFFF;
  }
}


static long
read_long(MY_FILETYPE* file) {
  unsigned char c[4];
  if (MY_READ(c, 4, 1, file) == 1) {
    return ((unsigned long)c[0] | (c[1]<<8) | (c[2]<<16) | (c[3]<<24));
  } else {
    return 0xFFFFFFFF;
  }
}


static unsigned long
read_ulong(MY_FILETYPE* file) {
  unsigned char c[4];
  if (MY_READ(c, 4, 1, file) == 1) {
    return c[0] | (c[1]<<8) | (c[2]<<16) | (c[3]<<24);
  } else {
    return 0xFFFFFFFF;
  }
}


static int
name_is_pointer(char* name) {
  int len = strlen(name);
  /*fprintf(stderr,"[%s]",name);*/
  if (len >= 1) {
    if (name[0] == '*')
      return 1;
  }
  if (len >= 2) {
    if (name[1] == '*')
      return 1;
  }
  return 0;
}


/* note: we only deal with 1d or 2d arrays at the moment.  haven't
   seen any arrays of a higher order from Blender yet. */
static int
name_is_array(char* name, int* dim1, int* dim2) {
  int len = strlen(name);
  /*fprintf(stderr,"[%s]",name);*/
  /*if (len >= 1) {
    if (name[len-1] != ']')
      return 1;
  }
  return 0;*/
  char *bp;
  int num;
  if (dim1) {
    *dim1 = 1;
  }
  if (dim2) {
    *dim2 = 1;
  }
  bp = strchr(name, '[');
  if (!bp) {
    return 0;
  }
  num = 0;
  while (++bp < name+len-1) {
    const char c = *bp;
    if (c == ']') {
      break;
    }
    if (c <= '9' && c >= '0') {
      num *= 10;
      num += (c - '0');
    } else {
      dprintf(stderr, "array parse error.\n");
      return 0;
    }
  }
  if (dim2) {
    *dim2 = num;
  }

  /* find second dim, if any. */
  bp = strchr(bp, '[');
  if (!bp) {
    return 1; /* at least we got the first dim. */
  }
  num = 0;
  while (++bp < name+len-1) {
    const char c = *bp;
    if (c == ']') {
      break;
    }
    if (c <= '9' && c >= '0') {
      num *= 10;
      num += (c - '0');
    } else {
      dprintf(stderr, "array2 parse error.\n");
      return 1;
    }
  }
  if (dim1) {
    if (dim2) {
      *dim1 = *dim2;
      *dim2 = num;
    } else {
      *dim1 = num;
    }
  }

  return 1;
}


#define SIZE_ROUNDUP(SIZE) (((SIZE) + sizeof(int) - 1) & ~(sizeof(int) - 1))


static void
recursively_read_type(MY_FILETYPE* file, BlendFile* bf,
		      unsigned long section_type,
		      BlendField* field)
{
  char *new_data = NULL;
  int new_data_size = 0;
  int dim1, dim2;

  if (bf->types[section_type].is_struct) {
    int i;
    /*fprintf(stderr, "type%d(%s)is_struct(%d) ", section_type, bf->types[section_type].name, bf->types[section_type].size);*/
    for (i=0; i<bf->types[section_type].fieldtypes_count; ++i) {
      /*fprintf(stderr, "@%d ", i);*/
      int j,k;
#if 0
      if (name_is_array(bf->names[bf->types[section_type].fieldnames[i]],
			&j,&k) || 1) {
	dprintf(stderr, " %s/%s=[%d][%d] ",
		bf->types[bf->types[section_type].fieldtypes[i]].name,
		bf->names[bf->types[section_type].fieldnames[i]], j, k);
	}
#endif 
      if (name_is_pointer(bf->names[bf->types[section_type].fieldnames[i]])) {
	/*fprintf(stderr, "*(4) ");*/
	name_is_array(bf->names[bf->types[section_type].fieldnames[i]],
		      &dim1,&dim2);
	new_data_size = SIZE_ROUNDUP(4);
	new_data = malloc(new_data_size);
	/*fprintf(stderr, " ");*/
	for (j=0; j<dim1; ++j) {
	  for (k=0; k<dim2; ++k) {
	    /*fprintf(stderr, "*");*/
	    new_data[0] = MY_GETC(file);
	    new_data[1] = MY_GETC(file);
	    new_data[2] = MY_GETC(file);
	    new_data[3] = MY_GETC(file);
	    EXPANDO(field->field_offsets, field->field_bytes_count);
	    EXPANDO_MULTIPLE(field->field_bytes, new_data, new_data_size);
	    /*fprintf(stderr, "N*(%d) ", new_data_size);*/
	  }
	}
	free(new_data);
      } else {
	name_is_array(bf->names[bf->types[section_type].fieldnames[i]],
		      &dim1,&dim2);
	/*fprintf(stderr, " ");*/
	for (j=0; j<dim1; ++j) {
	  for (k=0; k<dim2; ++k) {
	    /*fprintf(stderr, "R");*/
	    recursively_read_type(file, bf,
				  bf->types[section_type].fieldtypes[i],
				  field);
	  }
	}
      }
    }
  } else {
    /*fprintf(stderr, "type%d(%s)plain(%d) ", section_type, bf->types[section_type].name, bf->types[section_type].size);     */
    new_data_size = SIZE_ROUNDUP(bf->types[section_type].size);
    /*fprintf(stderr, "%d... ", bf->types[section_type].size);
      if (bf->types[section_type].size > 4) {
      fprintf(stderr, "%d ", field->field_bytes_count);
      }*/
    if (new_data_size) {
      new_data = malloc(new_data_size);
      MY_READ(new_data, 1, bf->types[section_type].size, file);
      EXPANDO(field->field_offsets, field->field_bytes_count);
      EXPANDO_MULTIPLE(field->field_bytes, new_data, new_data_size);
      /*fprintf(stderr, "ND(%d) ", new_data_size); */
      free(new_data);
    } else {
      dprintf(stderr, " <NUL[%d]> ",
	      bf->types[section_type].size);
    }
  }

}


static BlendField
read_type(MY_FILETYPE* file, BlendFile* bf,
	  unsigned long section_type)
{
  BlendField rtn;

  rtn.field_bytes = NULL;
  rtn.field_bytes_count = 0;
  rtn.field_offsets = NULL;
  rtn.field_offsets_count = 0;
  
  recursively_read_type(file, bf, section_type, &rtn);

  return rtn;
}


static int
blend_read_data(MY_FILETYPE* file, BlendFile* bf)
{
  long next_block_start = 12;
  int finished_extracting = 0;
  char section_name[5] = {0,0, 0,0, 0};

  /* slurp up the whole file block by block! */

  do {
    unsigned long section_size;
    unsigned long section_pointer;
    unsigned long section_type;
    unsigned long section_ents;

    MY_SEEK(file, next_block_start);

    MY_READ(section_name, 4, 1, file);

    if (strcmp(section_name, "DNA1") != 0) {
      int i;
      BlendBlock block;

      section_size    = read_ulong(file);
      section_pointer = read_ulong(file);
      section_type    = read_ulong(file);
      section_type    = bf->strc_indices[section_type];
      section_ents    = read_ulong(file);

      memcpy(block.tag, section_name, 4);
      block.tag[4] = '\0';
      block.type_index = section_type;
      block.blender_pointer = section_pointer;
      /*block.fixed_pointer = NULL;*/
      block.array_entries = NULL;
      block.array_entries_count = 0;

      /*dprintf(stderr, "\nsizeof(%s)=%ld: %s[%ld]\n", section_name, section_size, bf->types[section_type].name, section_ents); */

      for (i=0; i<section_ents; ++i) {
	BlendField field;
	field = read_type(file, bf, section_type);
	EXPANDO(block.array_entries, field);
      }
       
      EXPANDO(bf->blocks, block);

      next_block_start += 4+4+4+4+4 + section_size;

#ifdef B_DEBUG
      if (MY_TELL(file) > next_block_start) {
	dprintf(stderr, " **OVER-READ(%ld,%ld)** ",
		MY_TELL(file), next_block_start);
	if (strcmp(bf->types[section_type].name, "Link") == 0 &&
	    MY_TELL(file) - next_block_start == 4) {
	  dprintf(stderr, "<- don't panic, known Link struct weirdness.");
	} else {
	  dprintf(stderr, "<- okay, PANIC!");
	}
	dprintf(stderr, "\n");
      } else if (MY_TELL(file) < next_block_start) {
	/*dprintf(stderr, " **under-READ(%ld,%ld)** ",
	  MY_TELL(file), next_block_start);*/
      } else {
	/*dprintf(stderr, " :) ");*/
      }
#endif
	
    } else {
      finished_extracting = 1;
    }

  } while (!finished_extracting);

  return 1;
}


BlendFile*
blend_read(MY_FILETYPE* file)
{
  char blender_mark[8] = {0,0,0,0, 0,0,0,0};
  BlendFile *bf;
  long sdnaname_offs, type_offs, tlen_offs, strc_offs, endb_offs;
  long sdnaname_size, type_size, tlen_size, strc_size;
  long sdnaname_ents, type_ents, tlen_ents, strc_ents;

  MY_REWIND(file);

  /* Check file signature */

  MY_READ(blender_mark, 1, 7, file);
  if (strcmp(blender_mark, "BLENDER") != 0) {
    dprintf(stderr, "Not a Blender file.\n");
    return NULL;
  }

  /* Alloc a handle to return */

  bf = bf_new();

  /* Scan the whole file (!) for file section markers */
  
  sdnaname_offs = seek_past_string(file, "SDNANAME");
  sdnaname_ents = read_long(file);
  type_offs     = seek_past_string(file, "TYPE");
  type_ents     = read_long(file);
  tlen_offs     = seek_past_string(file, "TLEN");
  tlen_ents     = type_ents;
  strc_offs     = seek_past_string(file, "STRC");
  strc_ents     = read_long(file);
  endb_offs     = seek_past_string(file, "ENDB");

  if (sdnaname_offs == -1 || type_offs == -1 || tlen_offs == -1 ||
      strc_offs == -1 || endb_offs == -1) {
    dprintf(stderr, "Couldn't find all necessary file markers. :(\n");
    return NULL;
  }

  /* Move marker offsets to point to the start of each one's block */

  sdnaname_offs += 8 + 4;
  sdnaname_size = type_offs - sdnaname_offs;

  type_offs += 4 + 4;
  type_size = tlen_offs - type_offs;

  tlen_offs += 4;
  tlen_size = strc_offs - tlen_offs;

  strc_offs += 4 + 4;
  strc_size = endb_offs - strc_offs;

  /* read the NAME table */

  MY_SEEK(file, sdnaname_offs);
  {
    long offs = 0;
    int i;
    char *top_block_name;

    for (i=0; i<sdnaname_ents; ++i) {
      int finished_name = 0;
      char *this_name_chars = NULL;
      int this_name_chars_count = 0;

      do {
	const char c = MY_GETC(file);
	++offs;
	if (offs == sdnaname_size ||
	    c == '\0') {
	  finished_name = 1;
	}
	
	EXPANDO(this_name_chars, c);
      } while (!finished_name);
      
      /*fprintf(stderr, "%s ", this_name_chars); */
      
      EXPANDO(bf->names, this_name_chars);
    }

    /* our top-block name */
#define BLEND_TOP_BLOCK_NAME "READBLEND_TOP_BLOCK"
    top_block_name = calloc(1, 1+strlen(BLEND_TOP_BLOCK_NAME));
    strcpy(top_block_name, BLEND_TOP_BLOCK_NAME);
    bf->name_undef = bf->names_count;
    EXPANDO(bf->names, top_block_name);
  }

  /* read the TYPE table */

  MY_SEEK(file, type_offs);
  {
    long offs = 0;
    int i;

    for (i=0; i<type_ents; ++i) {
      int finished_type = 0;
      char *this_type_chars = NULL;
      int this_type_chars_count = 0;
      BlendType bt;

      do {
	const char c = MY_GETC(file);
	++offs;
	if (offs == type_size ||
	    c == '\0') {
	  finished_type = 1;
	}
	
	EXPANDO(this_type_chars, c);
      } while (!finished_type);
      
      bt.name = this_type_chars;
      bt.size = -1;
      bt.is_struct = 0;
      bt.fieldtypes_count = 0;
      bt.fieldtypes = NULL;
      bt.fieldnames_count = 0;
      bt.fieldnames = NULL;

      /* fprintf(stderr, "(%d:%s)", bt.size, bt.name);*/

      EXPANDO(bf->types, bt);
    }
  }

  /* read the TLEN table */

  MY_SEEK(file, tlen_offs);
  {
    int i;
    for (i=0; i<bf->types_count; ++i) {
      unsigned short len = read_ushort(file);
      bf->types[i].size = len;
      /*fprintf(stderr, "sizeof(%s)=%d ", bf->types[i].name, len); */
    }
  }

  /* Read the STRC table */
  
  MY_SEEK(file, strc_offs);
  {
    int i,j;
    for (i=0; i<strc_ents; ++i) {
      unsigned short struc_type_index, num_fields;
      struc_type_index = read_ushort(file);
      num_fields = read_ushort(file);
      bf->types[struc_type_index].is_struct = 1;
      EXPANDO(bf->strc_indices, struc_type_index);
      /*dprintf(stderr, "\n%s: ", bf->types[struc_type_index].name); */
      for (j=0; j<num_fields; ++j) {
	unsigned short ftype = read_ushort(file);
	unsigned short fname = read_ushort(file);
	EXPANDO(bf->types[struc_type_index].fieldtypes, ftype);
	EXPANDO(bf->types[struc_type_index].fieldnames, fname);
	/*dprintf(stderr, "%s %s ,  ", bf->types[ftype].name, bf->names[fname]); */
      }
    }
  }

  blend_read_data(file, bf);

  /* Return the new handle */

  return bf;
}


static void free_btype_inner(BlendType *btype)
{
 if (btype->name) {
   free(btype->name);
 } else {
   dprintf(stderr, "null typename.\n");
 }

 if (btype->fieldtypes) {
   free(btype->fieldtypes);
 }

 if (btype->fieldnames) {
   free(btype->fieldnames);
 }
}


static void free_bfield_inner(BlendField *bfield)
{
  if (bfield->field_bytes) {
    free(bfield->field_bytes);
  }

  if (bfield->field_offsets) {
    free(bfield->field_offsets);
  }
}


static void free_bblock_inner(BlendBlock *bblock)
{
  int i;
  
  for (i=0; i<bblock->array_entries_count; ++i) {
    free_bfield_inner(&bblock->array_entries[i]);
  }
  free(bblock->array_entries);  
}


void
blend_free(BlendFile* blend_file)
{
  int i;

  for (i=0; i<blend_file->types_count; ++i) {
    free_btype_inner(&blend_file->types[i]);
  }
  if (blend_file->types) {
    free(blend_file->types);
  }

  for (i=0; i<blend_file->names_count; ++i) {
    if (blend_file->names[i]) {
      free(blend_file->names[i]);
    } else {
      dprintf(stderr, "null name.\n");
    }
  }
  if (blend_file->names) {
    free(blend_file->names);
  }

  for (i=0; i<blend_file->blocks_count; ++i) {
    free_bblock_inner(&blend_file->blocks[i]);
  }
  if (blend_file->blocks) {
    free(blend_file->blocks);
  }

  if (blend_file->strc_indices) {
    free(blend_file->strc_indices);
  }

  free(blend_file);
}


/********************************************************************
 * done with the reading/parsing logic; now for the querying logic  *
 ********************************************************************/
						


/********************************************************************
 * LOW-LEVEL                                                        *
 ********************************************************************/



const char*
blend_block_get_tagname(BlendFile* blend_file,
			BlendBlockPointer block)
{
  const BlendBlock *const bb = block;
  return bb->tag;
}


const char*
blend_block_get_typename(BlendFile* blend_file,
			 BlendBlockPointer block)
{
  const BlendBlock *const bb = block;
  return blend_file->types[bb->type_index].name;
}


int
blend_block_get_entry_count(BlendFile* blend_file,
			    BlendBlockPointer block)
{
  const BlendBlock *const bb = block;
  return bb->array_entries_count;;
}


void
blend_foreach_block(BlendFile* blend_file,
		    BlendBlockCallback* func,
		    void* userdata)
{
  int i;
  for (i=0; i<blend_file->blocks_count; ++i) {
    if (!func(&blend_file->blocks[i], blend_file, userdata)) return;
  }
}


static int
blend_type_basename_compare(const char *fancy, const char *raw) {
  const int flen = strlen(fancy);
  const int rlen = strlen(raw);
  int i, strcmp_result = 123;

  i = 0;
  while (i < flen && (fancy[i]=='*' || fancy[i]=='(')) {
    ++i;
  }

  strcmp_result = strncmp(&fancy[i], raw, rlen);

  if (strcmp_result == 0 && flen > rlen+i) {
    i = rlen + i;
    if (fancy[i] != ')' && fancy[i] != '(' && fancy[i] != '[') {
      strcmp_result = -1;
    }
  }

  return strcmp_result;
}


static BlendObjType
typestring_to_blendobj_type(BlendFile* blend_file,
			    const char* type_name) {
  if (blend_type_basename_compare(type_name, "char") == 0) {
    return BLEND_OBJ_CHAR8;
  } else if (blend_type_basename_compare(type_name, "uchar") == 0) {
    return BLEND_OBJ_UCHAR8;
  } else if (blend_type_basename_compare(type_name, "short") == 0) {
    return BLEND_OBJ_SHORT16;
  } else if (blend_type_basename_compare(type_name, "ushort") == 0) {
    return BLEND_OBJ_USHORT16;
  } else if (blend_type_basename_compare(type_name, "int") == 0) {
    return BLEND_OBJ_LONG32;
  } else if (blend_type_basename_compare(type_name, "long") == 0) {
    return BLEND_OBJ_LONG32;
  } else if (blend_type_basename_compare(type_name, "ulong") == 0) {
    return BLEND_OBJ_ULONG32;
  } else if (blend_type_basename_compare(type_name, "float") == 0) {
    return BLEND_OBJ_FLOAT;
  } else if (blend_type_basename_compare(type_name, "double") == 0) {
    return BLEND_OBJ_DOUBLE;
  } else if (blend_type_basename_compare(type_name, "void") == 0) {
    return BLEND_OBJ_OPAQUE;
  } else {
    return BLEND_OBJ_STRUCT; /* structure */
  }
}


static BlendObjType
typelong_to_blendobj_type(BlendFile* blend_file,
			  long btype, long bname)
{
  if (name_is_pointer(blend_file->names[bname])) {
    return BLEND_OBJ_POINTER;
  } else if (blend_file->types[btype].is_struct) {
    return BLEND_OBJ_STRUCT;
  } else {
    return typestring_to_blendobj_type(blend_file,
				       blend_file->types[btype].name);
  }
}


BlendObjType
blend_object_type(BlendFile* blend_file,
		  BlendObject obj) {
  return typelong_to_blendobj_type(blend_file,
				   obj.type,
				   obj.name);
}


BlendBlockPointer
blend_block_from_blendpointer(BlendFile *blend_file,
			      uint32_t blendpointer)
{
  int i;

  /* fprintf(stderr, "%04x: ", blendpointer);*/

  if (blendpointer != 0) {
    for (i=0; i<blend_file->blocks_count; ++i) {
      /*fprintf(stderr, "%04x? ", blend_file->blocks[i].blender_pointer); */
      if (blend_file->blocks[i].blender_pointer == blendpointer) {
	return &blend_file->blocks[i];
      }
    }
  }

  return NULL;
}


static BlendBlockPointer
blend_block_from_object(BlendFile *blend_file,
                        BlendObject *obj) {
  return obj->block;
}


int
blend_object_array_getdata(BlendFile* blend_file,
			   void* dest, BlendObject obj,
			   int dim_index_1, int dim_index_2)
{
  const char* type_name = blend_file->types[obj.type].name;
  const BlendBlock *const bb = obj.block;
  BlendField *bf = &bb->array_entries[obj.entry_index];
  void* data;
  int dim1, dim2;

  name_is_array(blend_file->names[obj.name], &dim1, &dim2);
  /*dprintf(stderr, "copying:'%s'[%d][%d] (of [%d][%d]) ", type_name, dim_index_1, dim_index_2, dim1, dim2);*/

  if (dim_index_1 >= dim1 ||
      dim_index_2 >= dim2) {
    dprintf(stderr, "Array index (%d,%d) out of bounds for dimensionality [%d][%d]\n", dim_index_1, dim_index_2, dim1, dim2);
    return 0;
  }

  data = &bf->field_bytes[bf->field_offsets[obj.field_index +
					   dim2*dim_index_1 + dim_index_2]];
  /*dprintf(stderr, "fi[%d]byteo[%d]", obj.field_index,
          bf->field_offsets[obj.field_index +
          dim2*dim_index_1 + dim_index_2]);*/

  if (name_is_pointer(blend_file->names[obj.name])) {
    *(BlendBlockPointer*)dest =
      blend_block_from_blendpointer(blend_file,
				    BGETLEUINT32(data));
    return 1;
  }

  /* FIXME: might be a good idea to do less-crappy word-size conversions
     here -- these might read beyond the end of malloc'd blocks if we
     ever change our field-padding policy.  There were endian problems
     too; these are believed fixed now. */
  /* The signed conversions look strange because they have to sign-expand
     negative results without relying on right-shifts which have undefined
     behaviour on negative data according to ANSI C. */
  if (blend_type_basename_compare(type_name, "char") == 0) {
    *(char*)dest = (*(char*)data) << (8*sizeof(char)-8) / (1<<(8*sizeof(char)-8));
  } else if (blend_type_basename_compare(type_name, "uchar") == 0) {
    *(unsigned char*)dest = *(unsigned char*)data;
  } else if (blend_type_basename_compare(type_name, "short") == 0) {
    *(int16_t*)dest = BGETLEINT16(data) << (8*sizeof(int16_t)-16) / (1<<(8*sizeof(int16_t)-16));
  } else if (blend_type_basename_compare(type_name, "ushort") == 0) {
    *(uint16_t*)dest = BGETLEUINT16(data);
  } else if (blend_type_basename_compare(type_name, "int") == 0) {
    *(int32_t*)dest = BGETLEINT32(data) << (8*sizeof(int32_t)-32) / (1<<(8*sizeof(int32_t)-32));
  } else if (blend_type_basename_compare(type_name, "long") == 0) {
    *(int32_t*)dest = BGETLEINT32(data) << (8*sizeof(int32_t)-32) / (1<<(8*sizeof(int32_t)-32));
  } else if (blend_type_basename_compare(type_name, "ulong") == 0) {
    *(uint32_t*)dest = BGETLEUINT32(data);
  } else if (blend_type_basename_compare(type_name, "float") == 0) {
    *(float*)dest = BGETLEFLOAT32(data);
    /*fprintf(stderr, "GOT{%f'%f} ", *(float*)dest, *(float*)data);*/
  } else if (blend_type_basename_compare(type_name, "double") == 0) {
    *(double*)dest = BGETLEDOUBLE64(data);
  } else if (blend_type_basename_compare(type_name, "void") == 0) {
    dprintf(stderr, "Tried to fetch a void.\n");
    return 0;
  } else {
    dprintf(stderr, "Tried to fetch a whole structure.\n");
    return 0;
  }
  return 1; /* success */
}


int
blend_object_getdata(BlendFile* blend_file,
		     void* dest, BlendObject obj)
{
  int dim1, dim2;

  if (name_is_array(blend_file->names[obj.name], &dim1, &dim2)) {
    if (dim1 != 1 || dim2 != 1) {
      dprintf(stderr, "Tried to fetch a whole array.\n");
      return 0;
    }
  }

  return (blend_object_array_getdata(blend_file, dest, obj, 0, 0));
}


/* recursively count the number of fields and array items in this
   structure, for the purposes of skipping in the field offset array */
static long
get_num_type_segments(BlendFile* blend_file,
		      BlendObject obj)
{
  int i;
  long rtn = 0;
  int dim1,dim2;

  name_is_array(blend_file->names[obj.name],
		&dim1, &dim2);

  if (name_is_pointer(blend_file->names[obj.name]) ||
      !blend_file->types[obj.type].is_struct) {
    return (1 * dim1 * dim2);
  }

  /* fprintf(stderr, "STRUCTYAYYY ");*/

  for (i=0; i<blend_file->types[obj.type].fieldnames_count; ++i) {
    BlendObject qo = obj;
    qo.type = blend_file->types[obj.type].fieldtypes[i];
    qo.name = blend_file->types[obj.type].fieldnames[i];
    qo.field_index = i;
    rtn += get_num_type_segments(blend_file, qo) * dim1 * dim2;
  }

  return (rtn);
}


int
blend_object_structure_getfield(BlendFile* blend_file,
				BlendObject *result,
				BlendObject obj,
				const char* field_name)
{
  if (blend_file->types[obj.type].is_struct) {
    int i;
    int field_index = 0;
    for (i=0; i<blend_file->types[obj.type].fieldnames_count; ++i) {
      if (blend_type_basename_compare(
	      blend_file->names[blend_file->types[obj.type].fieldnames[i]],
	      field_name)
	  == 0) {
	result->type = blend_file->types[obj.type].fieldtypes[i];
	result->name = blend_file->types[obj.type].fieldnames[i];
	result->block = obj.block;
	result->entry_index = obj.entry_index;
	result->field_index = field_index;
	return 1;
      }

      {
	BlendObject qo = obj;
	int fos;
	qo.type = blend_file->types[obj.type].fieldtypes[i];
	qo.name = blend_file->types[obj.type].fieldnames[i];
	qo.field_index = field_index;
	fos = get_num_type_segments(blend_file, qo);
	/*fprintf(stderr, ">>%s %s:%d ",
		blend_file->types[qo.type].name,
		blend_file->names[qo.name], fos);*/
	field_index += fos;
      }
    }
    return 0;
  } else {
    dprintf(stderr, "Indexed object isn't a structure!\n");
    return 0;
  }
}


void
blend_object_array_getdims(BlendFile* blend_file,
			   BlendObject obj,
			   int* dim1, int* dim2)
{
  name_is_array(blend_file->names[obj.name],
		dim1, dim2);
}


BlendObject
blend_block_get_object(BlendFile* blend_file,
		       BlendBlockPointer block,
		       int entry_index)
{
  BlendObject bo;
  const BlendBlock *const bb = block;
  /*BlendField *bf = &bb->array_entries[entry_index]; */

  bo.type = bb->type_index;
  bo.name = blend_file->name_undef;
  bo.block = block;
  bo.entry_index = entry_index;
  bo.field_index = 0;

  return bo;
}

						


/********************************************************************
 * MID-LEVEL                                                        *
 ********************************************************************/


/* general helpers */


int
blend_object_getstring(BlendFile* blend_file,
		       BlendObject obj,
		       char *dest, int max_chars)
{
  int strpos = 0;
  int dim1, dim2;
  int rtn = 1;
  BlendObjType bo_type;

  name_is_array(blend_file->names[obj.name],
		&dim1, &dim2);

  if (dim2 < max_chars) {
    max_chars = dim2;
  }

  bo_type = blend_object_type(blend_file, obj);
  if (! (bo_type==BLEND_OBJ_CHAR8 || bo_type==BLEND_OBJ_UCHAR8)) {
    dprintf(stderr, "tried to get string from an object that's not of type uchar/char. (is type %d)\n", bo_type);
    rtn = 0;
    goto done;
  }
      
  for (strpos=0; strpos<max_chars-1; ++strpos) {
    char c;
    if (!blend_object_array_getdata(blend_file, &c, obj, 0, strpos)) {
      dprintf(stderr, "unexpected error accessing array. :(\n");
      rtn = 0;
      goto done;
    }
    if (c == '\0') {
      goto done;
    }
    dest[strpos] = c;
  }

 done:
  dest[strpos] = '\0';
  return rtn;
}


/* ID-finding helpers */

int
blend_object_get_IDname(BlendFile* blend_file,
                        BlendObject obj,
                        char *dest, int max_chars) {
  if (BLEND_OBJ_STRUCT == blend_object_type(blend_file, obj)) {
    BlendObject idstruc_obj;
    /* See if this structure has an 'id' field */
    if (blend_object_structure_getfield(blend_file, &idstruc_obj,
                                        obj, "id")) {
      /* we have an 'id' field -- look for a 'name' field within it */
      BlendObject name_obj;
      /* An ID field should have a 'name' field, but we'll make sure... */
      if (BLEND_OBJ_STRUCT == blend_object_type(blend_file, idstruc_obj) &&
          blend_object_structure_getfield(blend_file, &name_obj,
                                          idstruc_obj, "name")) {
        /* great, get the string from the 'name' field. */
        if (blend_object_getstring(blend_file, name_obj,
                                   dest, max_chars)) {
          return 1;
        } else {
          dprintf(stderr, "oh no, couldn't extract string.\n");
        }
      } else {
        /* next entry please. */
      }
    } else {
      /* this block's type structure doesn't have an ID field at all */
    }
  } else {
    /* this block's type isn't a structure */
    dprintf(stderr, "non-objectstruct type of type %d\n",
            blend_object_type(blend_file, obj));
  }

  return 0; /* failure */
}

typedef struct {
  BlendObject found;
  int success;
  const char* IDname;
  int just_print_it;
} IDFinderData;


static BLENDBLOCKCALLBACK_RETURN
block_ID_finder(BLENDBLOCKCALLBACK_ARGS) {
  int entry_count = blend_block_get_entry_count(blend_file, block);
  int want_more = 1;
  IDFinderData *ifd = (IDFinderData*)userdata;
  int i;

  if (ifd->IDname) { /* don't freak out, but hmm, do we do the right thing? */
    const int strl = strlen(ifd->IDname);
    char *obj_string = malloc(3+ strl);
    
    for (i=0; i<entry_count; ++i) {
      BlendObject obj = blend_block_get_object(blend_file, block, i);
      blend_object_get_IDname(blend_file, obj,
                              obj_string, strl+1);
      
      if (strcmp(obj_string, ifd->IDname) == 0) {
        ifd->found = obj;
        ifd->success = 1;
        want_more = 0;
        goto done;
      } else {
        /* next entry please. */
      }
    }
    
  done:;
    free(obj_string);
  }

  return want_more;
}


int
blend_object_get_by_IDname(BlendFile* blend_file,
			   BlendObject *result,
			   const char* IDname)
{
  IDFinderData ifd;

  ifd.success = 0;
  ifd.IDname = IDname;
  ifd.just_print_it = 0;

  blend_foreach_block(blend_file, block_ID_finder, &ifd);

  if (!ifd.success) {
    return 0;
  }

  *result = ifd.found;
  return 1;
}


void
blend_dump_typedefs(BlendFile* bf)
{
  int i;

  /* dump out display of types and their sizes */
  for (i=0; i<bf->types_count; ++i) {
    /* if (!bf->types[i].is_struct)*/
    {
      printf("%3d: sizeof(%s%s)=%d",
	     i,
	     bf->types[i].is_struct ? "struct " : "atomic ",
	     bf->types[i].name, bf->types[i].size);
      if (bf->types[i].is_struct) {
	int j;
	printf(", %d fields: { ", bf->types[i].fieldtypes_count);
	for (j=0; j<bf->types[i].fieldtypes_count; ++j) {
	  printf("%s %s",
		  bf->types[bf->types[i].fieldtypes[j]].name,
		  bf->names[bf->types[i].fieldnames[j]]);
	  if (j == bf->types[i].fieldtypes_count-1) {
	    printf("; }\n");
	  } else {
	    printf("; ");
	  }
	}
      }

    }
  }
}


void
blend_dump_blocks(BlendFile* bf)
{
  int i;
  IDFinderData ifd;

  ifd.success = 0;
  ifd.IDname = NULL;
  ifd.just_print_it = 1;

  for (i=0; i<bf->blocks_count; ++i) {
    BlendBlock* bb = &bf->blocks[i];
    printf("tag='%s'\tptr=%p\ttype=%s\t[%4d]",
	   bb->tag, /*bb->blender_pointer,*/ bb,
	   bf->types[bb->type_index].name,
	   bb->array_entries_count);
    block_ID_finder(bb, bf, &ifd);
    printf("\n");
  }
}


int
blend_obj_is_rootobject(BlendFile *bf, BlendObject *objobj) {
  BlendObject obj;
  BlendBlockPointer block;
  if ((blend_object_structure_getfield(bf, &obj, *objobj, "parent") &&
       blend_object_getdata(bf, &block, obj))) {
    return (block == NULL);
  }
  return 0;
}


BlendLayerMask
blend_obj_get_layermask(BlendFile *bf, BlendObject *objobj) {
  BlendObject obj;
  int32_t ldata;
  if ((blend_object_structure_getfield(bf, &obj, *objobj, "lay") &&
       blend_object_getdata(bf, &ldata, obj))) {
    return (BlendLayerMask) ldata;
  }
  return 0;
}


/* Bizarrely, blender doesn't seem to carry a mapping of parent
   to children -- only of children to parent.  So, to find the children
   of an Object we have to scan all objects to see if their parent is
   the Object in question. */
typedef struct {
  BlendBlockPointer wanted_parent;
  BlendBlockPointer child;
  int just_counting;
  int num_so_far;
  int wanted_index;
} childfinderData;
static BLENDBLOCKCALLBACK_RETURN
block_childfinder(BLENDBLOCKCALLBACK_ARGS) {
  childfinderData *cfd = (childfinderData *)userdata;
  const char *tagname = blend_block_get_typename(blend_file, block);
  int entry_count = blend_block_get_entry_count(blend_file, block);
  int want_more = 1;
  int i;

  if (strcmp(tagname, "Object") == 0) {
    /* Is Object */
    for (i=0; i<entry_count; ++i) {
      BlendObject obj;
      BlendObject oobj = blend_block_get_object(blend_file, block, i);
      BlendBlockPointer datablock;
      if (blend_object_structure_getfield(blend_file, &obj, oobj, "parent") &&
          blend_object_getdata(blend_file, &datablock, obj)) {
        /* has 'parent' pointer... are we interested? */
        if (datablock == cfd->wanted_parent) {
          if ((cfd->num_so_far == cfd->wanted_index) &&
              !cfd->just_counting) {
            cfd->child = block;
            want_more = 0;
          }
          ++cfd->num_so_far;
        } else {
          cfd->child = NULL;
        }
      }
    }
  }

  return want_more;
}

int
blend_obj_get_childcount(BlendFile *bf, BlendObject *objobj) {
  childfinderData cfd;
  cfd.wanted_parent = blend_block_from_object(bf, objobj);
  cfd.child = NULL;
  cfd.just_counting = 1;
  cfd.num_so_far = 0;
  cfd.wanted_index = 0;
  blend_foreach_block(bf, block_childfinder, &cfd);
  return cfd.num_so_far;
}

BlendBlockPointer
blend_obj_get_child(BlendFile *bf, BlendObject *objobj, int childnum) {
  childfinderData cfd;
  cfd.wanted_parent = blend_block_from_object(bf, objobj);
  cfd.child = NULL;
  cfd.just_counting = 0;
  cfd.num_so_far = 0;
  cfd.wanted_index = childnum;
  blend_foreach_block(bf, block_childfinder, &cfd);
  return cfd.child;
}


/********************************************************************
 * HIGH-LEVEL                                                       *
 ********************************************************************/

#ifndef LEN3POW2
/* length squared */
#define LEN3POW2(xd,yd,zd) ((xd)*(xd) + (yd)*(yd) + (zd)*(zd))
#endif

#ifndef LEN3
/* length (expensive) */
#define LEN3(xd,yd,zd) (sqrt(LEN3POW2((xd),(yd),(zd))))
#endif

#ifndef NORMALIZE3
/* vector normalization (expensive) */
#define NORMALIZE3(xd,yd,zd) \
do { \
  const double norm3_macro_len3 = LEN3((xd),(yd),(zd)); \
  if (norm3_macro_len3 != 0.0F) \
    { \
      (xd) = (xd) / norm3_macro_len3; \
      (yd) = (yd) / norm3_macro_len3; \
      (zd) = (zd) / norm3_macro_len3; \
    } \
} while (0)
#endif


static void bMatIdentity(bMatrix mat) {
  int i,j;
  for (i=0; i<4; ++i) {
    for (j=0; j<4; ++j) {
      mat[i][j] = 0.0f;
    }
    mat[i][i] = 1.0f;
  }
}

static void bMatMultVec(float xyz[3], bMatrix mat) {
  int i;
  float r[3];
  for (i=0; i<3; ++i) {
    r[i] = 0.0f;
  }
  for (i=0; i<3; ++i) {
    r[i] += xyz[0] * mat[0][i];
    r[i] += xyz[1] * mat[1][i];
    r[i] += xyz[2] * mat[2][i];
    r[i] += 1.0f   * mat[3][i];
  }
  for (i=0; i<3; ++i) {
    xyz[i] = r[i];
  }
}

bMesh *blend_alloc_mesh(void) {
  return malloc(sizeof(bMesh));
}

void
blend_init_mesh(bMesh *mesh)
{
  int i,j;
  mesh->vert = NULL;
  mesh->vert_count = 0;
  mesh->face = NULL;
  mesh->face_count = 0;
  mesh->material = NULL;
}

void
blend_free_mesh_inner(bMesh *mesh)
{
  if (mesh->vert) {
    free(mesh->vert);
  }
  if (mesh->face) {
    free(mesh->face);
  }
  if (mesh->material) {
    blend_free_material(mesh->material);
  }
}

void
blend_free_mesh(bMesh *mesh)
{
  blend_free_mesh_inner(mesh);
  free(mesh);
}


bObj *
blend_alloc_obj(void) {
  return malloc(sizeof(bObj));
}

void
blend_init_obj(bObj *obj) {
  obj->type = BOBJ_TYPE_UNKNOWN;
  obj->name = NULL;
  bMatIdentity(obj->transform);
  bMatIdentity(obj->parentimat);
  obj->location[0] =
    obj->location[1] =
    obj->location[2] = 0.0f;
  obj->scaling[0] =
    obj->scaling[1] =
    obj->scaling[2] = 1.0f;
  obj->rotphr[0] =
    obj->rotphr[1] =
    obj->rotphr[2] = 0.0f;
  obj->data.dummy = NULL;
  obj->transflags = 0;
}

void
blend_free_obj_inner(bObj *obj) {
  if (obj->name) {
    free(obj->name);
  }
  switch (obj->type) {
  case BOBJ_TYPE_MESH:
    blend_free_mesh(obj->data.mesh);
  case BOBJ_TYPE_UNKNOWN:
  case BOBJ_TYPE_NULL:
  default:
    break;
  }
}

void
blend_free_obj(bObj *obj) {
  blend_free_obj_inner(obj);
  free(obj);
}

void
blend_acquire_obj_from_obj(BlendFile *bf, BlendObject *objobj,
                           bObj *outobj, BlendObject **mallocdoutblendobject) {
  
  BlendObject obj, dataobj;
  BlendBlockPointer block;
  short sdata = 12345;
#define B_IDNAME_MAX_SIZE 80
  char *idname = malloc(1 + B_IDNAME_MAX_SIZE);
  float fdata1, fdata2, fdata3;

  if (mallocdoutblendobject) *mallocdoutblendobject = NULL;

  blend_init_obj(outobj);

  if (!(blend_object_structure_getfield(bf, &obj, *objobj,
                                        "type") &&
        blend_object_getdata(bf, &sdata, obj))) {
    abort(); /* couldn't get type */
  } else {
    switch (sdata) {
    case 1: /* mesh */
      outobj->type = BOBJ_TYPE_MESH; break;
    case 10: /* lamp */
    case 11: /* camera */
    default:
      outobj->type = BOBJ_TYPE_UNKNOWN; break;
    }
  }
  
  if (blend_object_get_IDname(bf, *objobj, idname, B_IDNAME_MAX_SIZE)) {
    outobj->name = idname;
  } else {
    free(idname);
    abort(); /* couldn't get obj name */
  }
      
  /* now we override the mesh transform with the object's.  should
         we merge, instead???  - hm, dunno, don't think so. */

  if (blend_object_structure_getfield(bf, &obj, *objobj, "loc") &&
      blend_object_array_getdata(bf, &fdata1, obj, 0,0) &&
      blend_object_array_getdata(bf, &fdata2, obj, 0,1) &&
      blend_object_array_getdata(bf, &fdata3, obj, 0,2)) {
    outobj->location[0] = fdata1;
    outobj->location[1] = fdata2;
    outobj->location[2] = fdata3;
  } else {
    outobj->location[0] = 
      outobj->location[1] = 
      outobj->location[2] = 0.0f;
  }
      
  if (blend_object_structure_getfield(bf, &obj, *objobj, "size") &&
      blend_object_array_getdata(bf, &fdata1, obj, 0,0) &&
      blend_object_array_getdata(bf, &fdata2, obj, 0,1) &&
      blend_object_array_getdata(bf, &fdata3, obj, 0,2)) {
    outobj->scaling[0] = fdata1;
    outobj->scaling[1] = fdata2;
    outobj->scaling[2] = fdata3;
  } else {
    outobj->scaling[0] = 
      outobj->scaling[1] = 
      outobj->scaling[2] = 1.0f;
  }
      
  if (blend_object_structure_getfield(bf, &obj, *objobj, "rot") &&
      blend_object_array_getdata(bf, &fdata1, obj, 0,0) &&
      blend_object_array_getdata(bf, &fdata2, obj, 0,1) &&
      blend_object_array_getdata(bf, &fdata3, obj, 0,2)) {
    outobj->rotphr[0] = fdata1;
    outobj->rotphr[1] = fdata2;
    outobj->rotphr[2] = fdata3;
  } else {
    outobj->rotphr[0] = 
      outobj->rotphr[1] = 
      outobj->rotphr[2] = 0.0f;
  }

  if (blend_object_structure_getfield(bf, &obj, *objobj, "parentinv")) {
    int i,j;
    for (i=0; i<4; ++i) {
      for (j=0; j<4; ++j) {
        blend_object_array_getdata(bf, &fdata1, obj, i,j);
        outobj->parentimat[i][j] = fdata1;
      }
    }
  }
      
  if (blend_object_structure_getfield(bf, &obj, *objobj, "transflag")) {
    char cdata;
    /* TODO: decode what these flags precisely mean. */
    /* top bit is 'powertrack' */
    if (blend_object_getdata(bf, &cdata, obj)) {
      outobj->transflags = (unsigned char)cdata;
    }
  }
      
  if (blend_object_structure_getfield(bf, &obj, *objobj, "obmat")) {
    int i,j;
    for (i=0; i<4; ++i) {
      for (j=0; j<4; ++j) {
        blend_object_array_getdata(bf, &fdata1, obj, i,j);
        outobj->transform[i][j] = fdata1;
        dprintf(stderr, "%0.3f ", fdata1);
      }
      dprintf(stderr, "\n");
    }
  }

  /* get actual obj here */

  if (! (blend_object_structure_getfield(bf, &obj, *objobj, "data") &&
         blend_object_getdata(bf, &block, obj))) {
    abort();
  }

  if (block == NULL) {
    outobj->type = BOBJ_TYPE_NULL;
  } else {
    dataobj = blend_block_get_object(bf, block, 0);
    if (mallocdoutblendobject) {
      *mallocdoutblendobject = malloc(sizeof(BlendObject));
      **mallocdoutblendobject = dataobj;
    }
  }


  switch (outobj->type) {
  case BOBJ_TYPE_MESH:
    outobj->data.mesh = blend_alloc_mesh();
    blend_acquire_mesh_from_obj(bf, &dataobj, outobj->data.mesh);
    break;
  case BOBJ_TYPE_UNKNOWN:
  default:
  case BOBJ_TYPE_NULL:
    outobj->data.dummy = NULL;
    break;
  }
}



bTexLayer *
blend_alloc_texlayer(void) {
  return malloc(sizeof(bTexLayer));
}

void
blend_init_texlayer(bTexLayer *tl) {
  tl->filename = NULL;
  tl->affects_mask = 0;
  tl->blend_mode = BTEX_BLEND_NORMAL;
  tl->coords_type = BTEX_COORDS_NONE;
  tl->is_st_clamped = 0;
  tl->flags = 0;
  tl->Nflags = tl->Ntype = 0;
  tl->xrepeat = tl->yrepeat = 1;
}

static void
blend_free_texlayer_inner(bTexLayer *tl) {
  if (tl->filename) {
    free(tl->filename);
  }
}

void
blend_free_texlayer(bTexLayer *tl) {
  blend_free_texlayer_inner(tl);
  free(tl);
}


void /* MTex */
blend_acquire_texlayer_from_obj(BlendFile *bf, BlendObject *tlobj,
                                bTexLayer *tl) {
  BlendObject obj;
  BlendBlockPointer tex_block;
  short sdata = 12345;

  blend_init_texlayer(tl);

  if (!(blend_object_structure_getfield(bf, &obj, *tlobj,
                                        "mapto") &&
        blend_object_getdata(bf, &sdata, obj))) {
    abort();
  }
  if (sdata & 0x01) {
    tl->affects_mask |= BTEX_AFFECT_COLOUR;
  }
  if (sdata & 0x40) {
    tl->affects_mask |= BTEX_AFFECT_EMIT;
  }
  if (sdata & 0x80) {
    tl->affects_mask |= BTEX_AFFECT_ALPHA;
  }
  /* note: mapto not fully decoded. */

  if (!(blend_object_structure_getfield(bf, &obj, *tlobj,
                                        "texflag") &&
        blend_object_getdata(bf, &sdata, obj))) {
    abort();
  }
  if (sdata & 0x02) {
    tl->affects_mask |= BTEX_AFFECT_STENCIL;
  }

  if (!(blend_object_structure_getfield(bf, &obj, *tlobj,
                                        "texco") &&
        blend_object_getdata(bf, &sdata, obj))) {
    abort();
  }
  switch (sdata) {
  case 1:
  case 2:
    tl->coords_type = BTEX_COORDS_REFLECT;
    break;
  case 16:
    tl->coords_type = BTEX_COORDS_UV;
    break;
  default:
    /* I haven't seen this happen, but it probably does... */
    tl->coords_type = BTEX_COORDS_NONE;
    break;
  }

  if (!(blend_object_structure_getfield(bf, &obj, *tlobj,
                                        "blendtype") &&
        blend_object_getdata(bf, &sdata, obj))) {
    abort();
  }
  tl->blend_mode = sdata; /* not decoded yet :( */

  if (blend_object_structure_getfield(bf, &obj, *tlobj, "tex") &&
      blend_object_getdata(bf, &tex_block, obj) && tex_block) {
    BlendObject tobj = blend_block_get_object(bf, tex_block, 0);
    BlendBlockPointer im_block;
    BlendObject obj;
  
    if (!(blend_object_structure_getfield(bf, &obj, tobj, "extend") &&
          blend_object_getdata(bf, &sdata, obj))) {
      abort();
    }
    tl->is_st_clamped = !(sdata & 2 /*'repeat'*/);
    /*fprintf(stderr, "CLAMP=%d (was %d)\n", tl->is_st_clamped, sdata);*/
    if (!(blend_object_structure_getfield(bf, &obj, tobj, "xrepeat") &&
          blend_object_getdata(bf, &sdata, obj))) {
      tl->xrepeat = 1;
    } else {
      tl->xrepeat = sdata;
    }
    if (!(blend_object_structure_getfield(bf, &obj, tobj, "yrepeat") &&
          blend_object_getdata(bf, &sdata, obj))) {
      tl->yrepeat = 1;
    } else {
      tl->yrepeat = sdata;
    }
    if (!(blend_object_structure_getfield(bf, &obj, tobj, "flag") &&
          blend_object_getdata(bf, &sdata, obj))) {
      abort();
    } else {
      tl->flags = sdata;
    }
    if (!(blend_object_structure_getfield(bf, &obj, tobj, "imaflag") &&
          blend_object_getdata(bf, &sdata, obj))) {
      abort();
    } else {
      if (sdata & 0x0001) {
        tl->flags |= BIMG_FLAG_INTERPOLATE;
      }
      if (sdata & 0x0004) {
        tl->flags |= BIMG_FLAG_MIPMAP;
      }
      if (sdata & 0x0100) {
        tl->flags |= BIMG_FLAG_ANTIALIAS;
      }
    }
    if (!(blend_object_structure_getfield(bf, &obj, tobj, "type") &&
          blend_object_getdata(bf, &sdata, obj))) {
      abort();
    } else {
      tl->Ntype = sdata;
    }
    if (blend_object_structure_getfield(bf, &obj, tobj, "ima") &&
        blend_object_getdata(bf, &im_block, obj) && im_block) {
      BlendObject imobj = blend_block_get_object(bf, im_block, 0);
#define BF_IMAGE_FILENAME_MAXSIZE 160
      tl->filename = malloc(BF_IMAGE_FILENAME_MAXSIZE);
      if (!(blend_object_structure_getfield(bf, &obj, imobj, "name") &&
            blend_object_getstring(bf, obj,
                                   tl->filename, BF_IMAGE_FILENAME_MAXSIZE))) {
        abort();
      }
    }
  } else {
    abort();
  }
}


bMaterial *
blend_alloc_material(void) {
  return malloc(sizeof(bMaterial));
}

void
blend_init_material(bMaterial *mat) {
  int i;
  for (i=0; i<BLENDER_MAX_TEX_LAYERS; ++i) {
    mat->tex_layer[i] = NULL;
  }
  mat->feature_mask = 0;
  for (i=0; i<4; ++i) {
    mat->colour_rgba[i] = 1.0f;
  }
  mat->emit = 0.0f;
}

void
blend_free_material(bMaterial *mat) {
  int i;
  for (i=0; i<BLENDER_MAX_TEX_LAYERS; ++i) {
    if (mat->tex_layer[i]) {
      blend_free_texlayer(mat->tex_layer[i]);
    }
  }
}

void
blend_acquire_material_from_obj(BlendFile *bf, BlendObject *matobj,
                                bMaterial *mat) {
  BlendObject obj;
  int i;
  int32_t ldata = 123456;
  float fdata = 123456.0;

  blend_init_material(mat);

  if ((blend_object_structure_getfield(bf, &obj, *matobj, "r") &&
       blend_object_getdata(bf, &fdata, obj))) {
    mat->colour_rgba[0] = fdata;
  }
  if ((blend_object_structure_getfield(bf, &obj, *matobj, "g") &&
       blend_object_getdata(bf, &fdata, obj))) {
    mat->colour_rgba[1] = fdata;
  }
  if ((blend_object_structure_getfield(bf, &obj, *matobj, "b") &&
       blend_object_getdata(bf, &fdata, obj))) {
    mat->colour_rgba[2] = fdata;
  }
  if ((blend_object_structure_getfield(bf, &obj, *matobj, "alpha") &&
       blend_object_getdata(bf, &fdata, obj))) {
    mat->colour_rgba[3] = fdata;
  }
  if ((blend_object_structure_getfield(bf, &obj, *matobj, "emit") &&
       blend_object_getdata(bf, &fdata, obj))) {
    mat->emit = fdata;
  }

  if (!(blend_object_structure_getfield(bf, &obj, *matobj, "mode") &&
        blend_object_getdata(bf, &ldata, obj))) {
    abort();
  }
  if (ldata & 0x04) {
    mat->feature_mask |= BMAT_FEATURE_SHADELESS;
  }
  if (ldata & 0x08) {
    mat->feature_mask |= BMAT_FEATURE_WIRE;
  }
  if (ldata & 0x10) {
    mat->feature_mask |= BMAT_FEATURE_VCOLLIGHT;
  }
  if (ldata & 0x80) {
    mat->feature_mask |= BMAT_FEATURE_VCOLPAINT;
  }
  if (ldata & (1024 | 512 | 256)) { /* not sure about this, it's strange. */
    mat->feature_mask |= BMAT_FEATURE_TEXFACE;
  }

  for (i=0; i<BLENDER_MAX_TEX_LAYERS; ++i) {
    BlendBlockPointer texl_block;
    if (blend_object_structure_getfield(bf, &obj, *matobj, "mtex") &&
        blend_object_array_getdata(bf, &texl_block, obj, 0,i) && texl_block) {
      BlendObject tlobj = blend_block_get_object(bf, texl_block, 0);
      mat->tex_layer[i] = blend_alloc_texlayer();
      /*fprintf(stderr, "GETTING LAYER AT POS %d\n", i);*/
      blend_acquire_texlayer_from_obj(bf, &tlobj, mat->tex_layer[i]);
    } else {
      /*fprintf(stderr, "NOTHING FOUND AT POS %d\n", i);*/
    }
  }
}

void
blend_acquire_mesh_from_obj(BlendFile *bf, BlendObject *meobj, bMesh *mesh)
{
    {
      BlendObject obj;
      BlendBlockPointer vblock, fblock, cblock, tblock,
                        dblock, matlink;
      int i;
      int32_t ldata = 123456;
      float fdata1, fdata2, fdata3;

      blend_init_mesh(mesh);
      
      if (!(blend_object_structure_getfield(bf, &obj, *meobj,
					    "totvert") &&
	    blend_object_getdata(bf, &ldata, obj))) {
	abort();
      }
      mesh->vert_count = ldata;
      
      if (!(blend_object_structure_getfield(bf, &obj, *meobj,
					    "totface") &&
	    blend_object_getdata(bf, &ldata, obj))) {
	abort();
      }
      mesh->face_count = ldata;

      dprintf(stderr, "%d verts, %d faces...\n",
	      mesh->vert_count, mesh->face_count);
      
      if (!(blend_object_structure_getfield(bf, &obj, *meobj,
					    "mface") &&
	    blend_object_getdata(bf, &fblock, obj))) {
	abort();
      }
      /* null fblock is okay */

      if (blend_object_structure_getfield(bf, &obj, *meobj,
                                          "mat") &&
          blend_object_getdata(bf, &matlink, obj) && matlink) {
        /* found an indirect material link, follow it */
	BlendObject matlinkobj = blend_block_get_object(bf, matlink, 0);
        BlendBlockPointer matblock;
        if (blend_object_structure_getfield(bf, &obj, matlinkobj,
                                            "next") &&
            blend_object_getdata(bf, &matblock, obj)) {
          if (matblock) {
            BlendObject matobj = blend_block_get_object(bf, matblock, 0);
            mesh->material = blend_alloc_material();
            blend_acquire_material_from_obj(bf, &matobj, mesh->material);
          } else {
            /* um, okay, link went nowhere, leave mesh->material NULL */
          }
        } else {
          abort();
        }
      } else {
        /* no material -- mesh->material will remain NULL */
      }
      
      if (!(blend_object_structure_getfield(bf, &obj, *meobj,
					    "mvert") &&
	    blend_object_getdata(bf, &vblock, obj))) {
	abort();
      }
      /* null vblock is okay */
      
      if (!(blend_object_structure_getfield(bf, &obj, *meobj,
					    "tface") &&
	    blend_object_getdata(bf, &tblock, obj))) {
	abort();
      }
      
      if (!(blend_object_structure_getfield(bf, &obj, *meobj,
					    "mcol") &&
	    blend_object_getdata(bf, &cblock, obj))) {
	abort();
      }
      
      if (!(blend_object_structure_getfield(bf, &obj, *meobj,
					    "dvert") &&
	    blend_object_getdata(bf, &dblock, obj))) {
	/* sometimes there isn't a dvert block... */
        dblock = NULL;
      }

      mesh->vert = malloc(sizeof(bVert) * mesh->vert_count);

      for (i=0; i<mesh->vert_count; ++i) {
	BlendObject obj = blend_block_get_object(bf, vblock, i);
	BlendObject aobj;
	float fdata1, fdata2, fdata3;
	int32_t sdata1, sdata2, sdata3;
	char cdata;

	mesh->vert[i].xyz[0] =
          mesh->vert[i].xyz[1] =
          mesh->vert[i].xyz[2] = -12345;
	mesh->vert[i].cnormal[0] =
          mesh->vert[i].cnormal[1] =
	  mesh->vert[i].cnormal[2] = -12345;
	mesh->vert[i].mat = -1;

	if (!(blend_object_structure_getfield(bf, &aobj, obj, "co") &&
	      blend_object_array_getdata(bf, &fdata1, aobj, 0,0) &&
	      blend_object_array_getdata(bf, &fdata2, aobj, 0,1) &&
	      blend_object_array_getdata(bf, &fdata3, aobj, 0,2))) {
	  abort();
	}
	mesh->vert[i].xyz[0] = fdata1;
	mesh->vert[i].xyz[1] = fdata2;
	mesh->vert[i].xyz[2] = fdata3;

	if (!(blend_object_structure_getfield(bf, &aobj, obj, "no") &&
	      blend_object_array_getdata(bf, &sdata1, aobj, 0,0) &&
	      blend_object_array_getdata(bf, &sdata2, aobj, 0,1) &&
	      blend_object_array_getdata(bf, &sdata3, aobj, 0,2))) {
	  abort();
	}
	mesh->vert[i].cnormal[0] = sdata1;
	mesh->vert[i].cnormal[1] = sdata2;
	mesh->vert[i].cnormal[2] = sdata3;
	/*fprintf(stderr, "%f ", LEN3(mesh->vert[i].normal[0],
	  mesh->vert[i].normal[1],
	  mesh->vert[i].normal[2]));*/
	if (sdata1 != 0 || sdata2 != 0 || sdata3 != 0) {
	  NORMALIZE3(mesh->vert[i].cnormal[0],
		     mesh->vert[i].cnormal[1],
		     mesh->vert[i].cnormal[2]);
	}

	if (!(blend_object_structure_getfield(bf, &aobj, obj, "mat_nr") &&
	      blend_object_getdata(bf, &cdata, aobj))) {
	  abort();
	}
	mesh->vert[i].mat = cdata;
      }

      mesh->face = malloc(sizeof(bFace) * mesh->face_count);

      for (i=0; i<mesh->face_count; ++i) {
	int j,k;
	BlendObject obj = blend_block_get_object(bf, fblock, i);
	BlendObject aobj;
	char cdata;

	mesh->face[i].v[0] = mesh->face[i].v[1] =
	  mesh->face[i].v[2] = mesh->face[i].v[3] = -1;
	mesh->face[i].mat = -1;
	mesh->face[i].flags = 0;
	for (j=0; j<4; ++j) {
	  for (k=0; k<3; ++k) {
	    mesh->face[i].rgba[j][k] = 1.0;
	  }
	  mesh->face[i].rgba[j][3] = 1.0f;
	  mesh->face[i].uv[j][0] = 0.0f;
	  mesh->face[i].uv[j][1] = 0.0f;
	  mesh->face[i].image_id = NULL;
	}

        if (blend_object_structure_getfield(bf, &aobj, obj, "v1")) {
          if (0!=strcmp(bf->types[aobj.type].name,"int") &&
              0!=strcmp(bf->types[aobj.type].name,"ushort")) {
            dprintf(stderr, "Expected vertex-index type to be 'ushort' or 'int', got '%s'\n", bf->types[aobj.type].name);
            abort();
          }
          
          if (0==strcmp(bf->types[aobj.type].name,"int")) {
            /* index type is a 32bit int, generated by newish Blenders */
            int32_t idata;
            if (!(blend_object_structure_getfield(bf, &aobj, obj, "v1") &&
                  blend_object_getdata(bf, &idata, aobj))) {
              abort();
            }
            mesh->face[i].v[0] = idata;
            
            if (!(blend_object_structure_getfield(bf, &aobj, obj, "v2") &&
                  blend_object_getdata(bf, &idata, aobj))) {
              abort();
            }
            mesh->face[i].v[1] = idata;
            
            if (!(blend_object_structure_getfield(bf, &aobj, obj, "v3") &&
                  blend_object_getdata(bf, &idata, aobj))) {
              abort();
            }
            mesh->face[i].v[2] = idata;
            
            if (!(blend_object_structure_getfield(bf, &aobj, obj, "v4") &&
                  blend_object_getdata(bf, &idata, aobj))) {
              abort();
            }
            mesh->face[i].v[3] = idata;
          } else {
            /* index type is a 16bit ushort, generated by old Blenders */
            uint16_t usdata;
            if (!(blend_object_structure_getfield(bf, &aobj, obj, "v1") &&
                  blend_object_getdata(bf, &usdata, aobj))) {
              abort();
            }
            mesh->face[i].v[0] = usdata;
            
            if (!(blend_object_structure_getfield(bf, &aobj, obj, "v2") &&
                  blend_object_getdata(bf, &usdata, aobj))) {
              abort();
            }
            mesh->face[i].v[1] = usdata;
            
            if (!(blend_object_structure_getfield(bf, &aobj, obj, "v3") &&
                  blend_object_getdata(bf, &usdata, aobj))) {
              abort();
            }
            mesh->face[i].v[2] = usdata;
            
            if (!(blend_object_structure_getfield(bf, &aobj, obj, "v4") &&
                  blend_object_getdata(bf, &usdata, aobj))) {
              abort();
            }
            mesh->face[i].v[3] = usdata;            
          }
        } else {
          abort();
        }

	if (!(blend_object_structure_getfield(bf, &aobj, obj, "mat_nr") &&
	      blend_object_getdata(bf, &cdata, aobj))) {
	  abort();
	}
	mesh->face[i].mat = cdata;

	if (!(blend_object_structure_getfield(bf, &aobj, obj, "flag") &&
	      blend_object_getdata(bf, &cdata, aobj))) {
	  abort();
	}
	mesh->face[i].flags = cdata;
      }

      if (cblock) {
	/* we have vertex colours */
	for (i=0; i<mesh->face_count; ++i) {
	  int j;
	  unsigned char cdata;
	  BlendObject aobj;
	  for (j=0; j<4; ++j) {
	    BlendObject obj = blend_block_get_object(bf, cblock, i*4+j);
	    if (!(blend_object_structure_getfield(bf, &aobj, obj, "b") &&
		  blend_object_getdata(bf, &cdata, aobj))) {
	      abort();
	    }
	    mesh->face[i].rgba[j][0] = cdata / 255.0f;

	    if (!(blend_object_structure_getfield(bf, &aobj, obj, "g") &&
		  blend_object_getdata(bf, &cdata, aobj))) {
	      abort();
	    }
	    mesh->face[i].rgba[j][1] = cdata / 255.0f;

	    if (!(blend_object_structure_getfield(bf, &aobj, obj, "r") &&
		  blend_object_getdata(bf, &cdata, aobj))) {
	      abort();
	    }
	    mesh->face[i].rgba[j][2] = cdata / 255.0f;

	    /* alpha seems to be nonsense :( */
	    /*
	    if (!(blend_object_structure_getfield(bf, &aobj, obj, "a") &&
		  blend_object_getdata(bf, &cdata, aobj))) {
	      abort();
	    }
	    mesh->face[i].rgba[j][3] = cdata / 255.0f;
	    */
	    mesh->face[i].rgba[j][3] = 1.0f;
	  }
	}
      } else {
	/* !cblock (no vertex colours) */
	for (i=0; i<mesh->face_count; ++i) {
	  int j;
	  for (j=0; j<4; ++j) {
	    mesh->face[i].rgba[j][0] = 1.0f;
	    mesh->face[i].rgba[j][1] = 1.0f;
	    mesh->face[i].rgba[j][2] = 1.0f;
	    mesh->face[i].rgba[j][3] = 1.0f;
	  }
	}
      }

      if (tblock) {
	/* we have tex co-ords */
	for (i=0; i<mesh->face_count; ++i) {
	  int j,k;
	  void *pdata;
	  BlendObject aobj;
	  BlendObject obj = blend_block_get_object(bf, tblock, i);

	  if (!(blend_object_structure_getfield(bf, &aobj, obj, "tpage") &&
		blend_object_getdata(bf, &pdata, aobj))) {
	    abort();
	  }
	  mesh->face[i].image_id = pdata;

	  for (j=0; j<4; ++j) {
	    uint32_t uldata;
	    for (k=0; k<2; ++k) {
	      float fdata;	      
	      if (!(blend_object_structure_getfield(bf, &aobj, obj, "uv") &&
		    blend_object_array_getdata(bf, &fdata, aobj, j,k))) {
		abort();
	      }
	      mesh->face[i].uv[j][k] = fdata;
	    }
	    mesh->face[i].uv[j][1] = 1.0f - mesh->face[i].uv[j][1];      
	    /* texture face colour... not sure how this conceptually
	       differs from the face vertex colours, but it does. */
	    if (!(blend_object_structure_getfield(bf, &aobj, obj, "col") &&
		  blend_object_array_getdata(bf, &uldata, aobj, 0,j))) {
	      abort();
	    }
	    /* in its usual inconsistant style, blender packs this
	       RGBA value into the bytes of an unsigned long... */
	    mesh->face[i].rgba2[j][0] = ((uldata >> 24) & 0xFF) / 255.0f;
	    mesh->face[i].rgba2[j][1] = ((uldata >> 16) & 0xFF) / 255.0f;
	    mesh->face[i].rgba2[j][2] = ((uldata >> 8) & 0xFF) / 255.0f;
	    mesh->face[i].rgba2[j][3] = ((uldata >> 0) & 0xFF) / 255.0f;
	  }
	  /*mesh->face[i].uv[0][0]=0; mesh->face[i].uv[0][1]=0;
	  mesh->face[i].uv[1][0]=1; mesh->face[i].uv[1][1]=0;
	  mesh->face[i].uv[2][0]=1; mesh->face[i].uv[2][1]=1;
	  mesh->face[i].uv[3][0]=0; mesh->face[i].uv[3][1]=1;*/

	}
      } else {
	/* !tblock (no texture co-ords, no face tex-colours) */
	for (i=0; i<mesh->face_count; ++i) {
	  int j;
	  for (j=0; j<4; ++j) {
	    mesh->face[i].rgba2[j][0] = 1.0f;
	    mesh->face[i].rgba2[j][1] = 1.0f;
	    mesh->face[i].rgba2[j][2] = 1.0f;
	    mesh->face[i].rgba2[j][3] = 1.0f;
	  }
	}
      }

      if (dblock) {
	/* we have vertex deformation weights */
	for (i=0; i<mesh->vert_count; ++i) {
	  int j;
	  int32_t ldata;
	  float fdata;
	  BlendBlockPointer pdata;
	  BlendObject aobj;
	  BlendObject obj = blend_block_get_object(bf, dblock, i);

	  if (!(blend_object_structure_getfield(bf, &aobj, obj, "totweight") &&
		blend_object_getdata(bf, &ldata, aobj))) {
	    abort();
	  }
	  mesh->vert[i].deform_weights_count = ldata;
	  mesh->vert[i].deform_weights = malloc(ldata*sizeof(bDeformWeight));

	  if (!(blend_object_structure_getfield(bf, &aobj, obj, "dw") &&
		blend_object_getdata(bf, &pdata, aobj))) {
	    abort();
	  }
	  
	  for (j=0; j<mesh->vert[i].deform_weights_count; ++j) {
	    BlendObject dwobj = blend_block_get_object(bf, pdata, j);
	    
	    if (!(blend_object_structure_getfield(bf, &aobj, dwobj, "def_nr")
		  && blend_object_getdata(bf, &ldata, aobj))) {
	      abort();
	    }
	    mesh->vert[i].deform_weights[j].bone_id = ldata;
	    
	    if (!(blend_object_structure_getfield(bf, &aobj, dwobj, "weight")
		  && blend_object_getdata(bf, &fdata, aobj))) {
	      abort();
	    }
	    mesh->vert[i].deform_weights[j].weight = fdata;
	  }	  
	}
      } else {
	/* !dblock (no vertex deformation weights) */
	for (i=0; i<mesh->vert_count; ++i) {
	  mesh->vert[i].deform_weights = NULL;
	  mesh->vert[i].deform_weights_count = 0;
	}
      }
    }
}



void
blend_acquire_mesh(const char *fname, const char *want_name, bMesh *mesh)
{
  BlendFile* bf;
  MY_FILETYPE *fp;

  fp = MY_OPEN_FOR_READ(fname);

  if (!fp) {
    dprintf(stderr, "couldn't open file %s.\n", fname);
    abort();
  }

  bf = blend_read(fp);
  {
    BlendObject meobj;
    if (!blend_object_get_by_IDname(bf, &meobj, want_name)) {
      dprintf(stderr, "couldn't find %s.\n", want_name);
      abort();
    }
    blend_dump_blocks(bf);
    blend_acquire_mesh_from_obj(bf, &meobj, mesh);
  }
  blend_free(bf);

  MY_CLOSE(fp);
}


/* apply pitch, head, roll */
static void bRotPHR(float xyz[3], const float rot[3]) {
  float rx,ry,rz;
  float ix,iy,iz;
  float cosang, sinang;

  ix = xyz[0]; iy = xyz[1]; iz = xyz[2];

  cosang = cos(rot[0]);
  sinang = sin(rot[0]);
  /* pitch */
  rx = ix;
  ry = iy * cosang - iz * sinang;
  rz = iy * sinang + iz * cosang;

  ix = rx; iy = ry; iz = rz;

  cosang = cos(rot[1]);
  sinang = sin(rot[1]);
  /* head */
  rx =  ix * cosang + iz * sinang;
  ry =  iy;
  rz = -ix * sinang + iz * cosang;

  ix = rx; iy = ry; iz = rz;

  cosang = cos(rot[2]);
  sinang = sin(rot[2]);
  /* roll */
  rx = ix * cosang - iy * sinang;
  ry = ix * sinang + iy * cosang;
  rz = iz;

  xyz[0] = rx; xyz[1] = ry; xyz[2] = rz;
}


void
blend_transform_mesh_from_obj(bMesh *mesh, bObj *obj) {
  int i;
  for (i=0; i<mesh->vert_count; ++i) {
    /* this one looks good. */
    bMatMultVec(mesh->vert[i].xyz, obj->transform);
    bRotPHR(mesh->vert[i].cnormal, obj->rotphr);
  }
}

