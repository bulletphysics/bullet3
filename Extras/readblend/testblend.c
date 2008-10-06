#include <stdlib.h>
#include <stdio.h>

#include "abs-file.h"

#include "readblend.h"

void crawl(BlendFile* blend_file,
	   BlendObject obj)
{
  BlendObject data_obj;

  if (blend_object_structure_getfield(blend_file, &data_obj, obj, "totvert")) {
    if (blend_object_type(blend_file, data_obj) == BLEND_OBJ_LONG32) {
      long data1 = 123456;
      if (blend_object_getdata(blend_file, &data1, data_obj)) {
	fprintf(stderr, "TOTVERT=(%ld) ", data1);
      } else {
	fprintf(stderr, "FETCH ERROR\n");
      }
    }
  }

  if (blend_object_structure_getfield(blend_file, &data_obj, obj, "co")) {
    if (blend_object_type(blend_file, data_obj) == BLEND_OBJ_FLOAT) {
      float data1 = 123456.7;
      float data2 = 123456.8;
      float data3 = 123456.9;
      if (blend_object_array_getdata(blend_file, &data1, data_obj, 0,0) &&
	  blend_object_array_getdata(blend_file, &data2, data_obj, 0,1) &&
	  blend_object_array_getdata(blend_file, &data3, data_obj, 0,2)) {
	fprintf(stderr, "CO=(%f,%f,%f) ", data1, data2, data3);
      } else {
	fprintf(stderr, "FETCH ERROR\n");
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

  blend_dump_blocks(bf);

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
