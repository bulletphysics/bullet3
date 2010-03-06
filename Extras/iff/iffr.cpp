/*----------------------------------------------------------------------*
* IFFR.C  Support routines for reading IFF-85 files.          11/15/85
* (IFF is Interchange Format File.)
*
* By Jerry Morrison and Steve Shaw, Electronic Arts.
* This software is in the public domain.
*
* Original version was for the Commodore-Amiga computer.
* This version is compatible with PC, OSX, PS3, Wii, iPhone. 10/26/2008
*----------------------------------------------------------------------*/

#include "iff.h"
/*  #include "DF1:iff/gio.h" */
/*  #define OFFSET_BEGINNING OFFSET_BEGINING */

/**  Manx expects INTs as 16 bits,  This wont matter on LAttice ***/



/* ---------- Read -----------------------------------------------------*/

extern int PutID(); /**  Added as a diagnostic aid, will remove later ***/

/* ---------- OpenRIFF --------------------------------------------------*/
IFFP OpenRIFF(BPTR file0, GroupContext *new0,ClientFrame *clientFrame)
{
	register BPTR file = file0;
	register GroupContext *newtmp = new0;
	IFFP iffp = IFF_OKAY;

	newtmp->parent       = NL;      /* "whole file" has no parent.*/
	newtmp->clientFrame  = clientFrame;
	newtmp->file         = file;
	newtmp->position     = 0;
	newtmp->ckHdr.ckID   = newtmp->subtype    = NULL_CHUNK;
	newtmp->ckHdr.ckSize = newtmp->bytesSoFar = 0;

	/* Set newtmp->bound. AmigaDOS specific code.*/
	if (file <= 0)   return(NO_FILE);
	Seek(file, 0L, OFFSET_END);         /* Seek to end of file.*/
	newtmp->bound = ftell(file);//Seek(file, 0L, OFFSET_CURRENT);   /* Pos'n == #bytes in file.*/
	if (newtmp->bound < 0)   return(DOS_ERROR);   /* DOS being absurd.*/
	Seek(file, 0L, OFFSET_BEGINNING);      /* Go to file start.*/
	/* Would just do this if Amiga DOS maintained fh_End: */
	/* newtmp->bound = (FileHandle *)BADDR(file)->fh_End; */

	if ( newtmp->bound < (long)sizeof(ChunkHeader) )
		iffp = NOT_IFF;
	return(iffp);
}

/* ---------- OpenRGroup -----------------------------------------------*/
IFFP OpenRGroup(GroupContext* parent0,GroupContext* new0)
{
	register GroupContext *parent = parent0;
	register GroupContext *newtmp    = new0;
	IFFP iffp = IFF_OKAY;

	newtmp->parent       = parent;
	newtmp->clientFrame  = parent->clientFrame;
	newtmp->file         = parent->file;
	newtmp->position     = parent->position;
	newtmp->bound        = parent->position + ChunkMoreBytes(parent);
	newtmp->ckHdr.ckID   = newtmp->subtype    = NULL_CHUNK;
	newtmp->ckHdr.ckSize = newtmp->bytesSoFar = 0;

	if ( newtmp->bound > parent->bound  ||  IS_ODD(newtmp->bound) )
		iffp = BAD_IFF;
	return(iffp);
}

/* ---------- CloseRGroup -----------------------------------------------*/
IFFP CloseRGroup(GroupContext *context)
{
	register int position;

	if (context->parent == NL) {
	}  /* Context for whole file.*/
	else {
		position = context->position;
		context->parent->bytesSoFar += position - context->parent->position;
		context->parent->position = position;
	}
	return(IFF_OKAY);
}

/* ---------- SkipFwd --------------------------------------------------*/
/* Skip over bytes in a context. Won't go backwards.*/
/* Updates context->position but not context->bytesSoFar.*/
/* This implementation is AmigaDOS specific.*/
IFFP SkipFwd(GroupContext *context,int bytes)
{
	IFFP iffp = IFF_OKAY;

	if (bytes > 0) {
		if (-1 == Seek(context->file, bytes, OFFSET_CURRENT))
			iffp = BAD_IFF;   /* Ran out of bytes before chunk complete.*/
		else
			context->position += bytes;
	}
	return(iffp);
}

short int endianSwap16(short int val)
{
   long int i = 1;
   const char *p = (const char *) &i;
   if (p[0] == 1)  // Lowest address contains the least significant byte
   {
	   return (((val & 0xff00) >> 8) | ((val & 0x00ff) << 8));
   }
   return val;
}



int endianSwap32(int val)
{
   long int i = 1;
   const char *p = (const char *) &i;
   if (p[0] == 1)  // Lowest address contains the least significant byte
   {
	   return (((val & 0xff000000) >> 24) | ((val & 0x00ff0000) >> 8) | ((val & 0x0000ff00) << 8)  | ((val & 0x000000ff) << 24));
   }
   return val;
}





/* ---------- GetChunkHdr ----------------------------------------------*/
int GetChunkHdr(GroupContext *context0)
{
	register GroupContext *context = context0;
	register IFFP iffp;
	int remaining;

	/* Skip remainder of previous chunk & padding. */
	iffp = SkipFwd(context,
		ChunkMoreBytes(context) + IS_ODD(context->ckHdr.ckSize));
	CheckIFFP();

	/* Set up to read the newtmp header. */
	context->ckHdr.ckID = BAD_IFF;   /* Until we know it's okay, mark it BAD.*/
	context->subtype    = NULL_CHUNK;
	context->bytesSoFar = 0;

	/* Generate a psuedo-chunk if at end-of-context. */
	remaining = context->bound - context->position;
	if (remaining == 0 ) {
		context->ckHdr.ckSize = 0;
		context->ckHdr.ckID   = END_MARK;
	}
	/* BAD_IFF if not enough bytes in the context for a ChunkHeader.*/
	else if ((long)sizeof(ChunkHeader) > remaining) {
		context->ckHdr.ckSize = remaining;
	}

	/* Read the chunk header (finally). */
	else {
		switch (Read(context->file,
			&context->ckHdr, (long)sizeof(ChunkHeader)))
		{
		case -1: return(context->ckHdr.ckID = DOS_ERROR);
		case 0:  return(context->ckHdr.ckID = BAD_IFF);
		}
		//swap endian-ness of ckSize on little endian machines
		context->ckHdr.ckSize = endianSwap32(context->ckHdr.ckSize);




		/*** $$$ ***
		PutID(context->ckHdr.ckID);
		printf("\n");
		printf("id = %lx\n", context->ckHdr.ckID);
		**/

		/* Check: Top level chunk must be LIST or FORM or CAT. */
		if (context->parent == NL) {
			if (context->ckHdr.ckID != FORM &&
				context->ckHdr.ckID != LIST &&
				context->ckHdr.ckID != CAT )
				return(context->ckHdr.ckID = NOT_IFF);
		}

		/* Update the context. */
		context->position += (long)sizeof(ChunkHeader);
		remaining         -= (long)sizeof(ChunkHeader);

		/* Non-positive int values are illegal and used for error codes.*/
		/* We could check for other illegal IDs...*/
		if (context->ckHdr.ckID <= 0 )
			context->ckHdr.ckID = BAD_IFF;

		/* Check: ckSize negative or larger than # bytes left in context? */
		else if (context->ckHdr.ckSize < 0  ||
			context->ckHdr.ckSize > remaining) {
				context->ckHdr.ckSize = remaining;
				context->ckHdr.ckID   = BAD_IFF;
		}

		/* Automatically read the LIST, FORM, PROP, or CAT subtype int */
		else {
			if (context->ckHdr.ckID == LIST ||
				context->ckHdr.ckID == FORM ||
				context->ckHdr.ckID == PROP ||
				context->ckHdr.ckID == CAT) {
					iffp = IFFReadBytes(context, (BYTE *)&context->subtype,
						(long)sizeof(int));
					if (iffp != IFF_OKAY )
						context->ckHdr.ckID = iffp;
			}
		}
	}
	return(context->ckHdr.ckID);
}

/* ---------- IFFReadBytes ---------------------------------------------*/
IFFP IFFReadBytes(GroupContext *context,BYTE *buffer, int nBytes)
{
	register IFFP iffp = IFF_OKAY;

	if (nBytes < 0)
		iffp = CLIENT_ERROR;

	else if (nBytes > ChunkMoreBytes(context))
		iffp = SHORT_CHUNK;

	else if (nBytes > 0 )
		switch ( Read(context->file, buffer, nBytes) ) {
		 case -1: {iffp = DOS_ERROR; break; }
		 case 0:  {iffp = BAD_IFF;   break; }
		 default: {
			 context->position   += nBytes;
			 context->bytesSoFar += nBytes;
				  }
	}
	return(iffp);
}

/* ---------- SkipGroup ------------------------------------------------*/
IFFP SkipGroup( GroupContext* context) 
{
	return 0;
}   /* Nothing to do, thanks to GetChunkHdr */

/* ---------- ReadIFF --------------------------------------------------*/
IFFP ReadIFF(BPTR file,ClientFrame *clientFrame)
{
	/*CompilerBug register*/ IFFP iffp;
	GroupContext context;

	iffp = OpenRIFF(file, &context,clientFrame);
	context.clientFrame = clientFrame;

	if (iffp == IFF_OKAY) {
		iffp = GetChunkHdr(&context);

		if (iffp == FORM)
			iffp = (*clientFrame->getForm)(&context);

		else if (iffp == LIST)
			iffp = (*clientFrame->getList)(&context);

		else if (iffp == CAT)
			iffp = (*clientFrame->getCat)(&context);
	}
	CloseRGroup(&context);

	if (iffp > 0 )           /* Make sure we don't return an int.*/
		iffp = NOT_IFF;      /* GetChunkHdr should've caught this.*/
	return(iffp);
}

/* ---------- ReadIList ------------------------------------------------*/
IFFP ReadIList(GroupContext *parent,ClientFrame *clientFrame)
{
	GroupContext listContext;
	IFFP iffp;
	BOOL propOk = TRUE;

	iffp = OpenRGroup(parent, &listContext);
	CheckIFFP();

	/* One special case test lets us handle CATs as well as LISTs.*/
	if (parent->ckHdr.ckID == CAT)
		propOk = FALSE;
	else
		listContext.clientFrame = clientFrame;

	do {
		iffp = GetChunkHdr(&listContext);
		if (iffp == PROP) {
			if (propOk)
				iffp = (*clientFrame->getProp)(&listContext);
			else
				iffp = BAD_IFF;
		}
		else if (iffp == FORM)
			iffp = (*clientFrame->getForm)(&listContext);

		else if (iffp == LIST)
			iffp = (*clientFrame->getList)(&listContext);

		else if (iffp == CAT)
			iffp = (*clientFrame->getList)(&listContext);

		if (listContext.ckHdr.ckID != PROP)
			propOk = FALSE;   /* No PROPs allowed after this point.*/
	} while (iffp == IFF_OKAY);

	CloseRGroup(&listContext);

	if (iffp > 0 )   /* Only chunk types above are allowed in a LIST/CAT.*/
		iffp = BAD_IFF;
	return(iffp == END_MARK ? IFF_OKAY : iffp);
}

/* ---------- ReadICat -------------------------------------------------*/
/* By special arrangement with the ReadIList implement'n, this is trivial.*/
IFFP ReadICat(GroupContext *parent)
{
	return( ReadIList(parent, parent->clientFrame));//NL) );
}

/* ---------- GetFChunkHdr ---------------------------------------------*/
int GetFChunkHdr(GroupContext *context)
{
	register int id;

	id = GetChunkHdr(context);
	if (id == PROP)
		context->ckHdr.ckID = id = BAD_IFF;
	return(id);
}

/* ---------- GetF1ChunkHdr ---------------------------------------------*/
int GetF1ChunkHdr(GroupContext *context)
{
	register int id;
	register ClientFrame *clientFrame = context->clientFrame;

	id = GetChunkHdr(context);
	if (id == PROP)
		id = BAD_IFF;

	else if (id == FORM)
		id = (*clientFrame->getForm)(context);

	else if (id == LIST)
		id = (*clientFrame->getForm)(context);

	else if (id == CAT)
		id = (*clientFrame->getCat)(context);

	return(context->ckHdr.ckID = id);
}

/* ---------- GetPChunkHdr ---------------------------------------------*/
int GetPChunkHdr(GroupContext *context)
{
	register int id;

	id = GetChunkHdr(context);
	if (id == LIST || id == FORM || id == PROP || id == CAT )
		id = context->ckHdr.ckID = BAD_IFF;
	return(id);
}
