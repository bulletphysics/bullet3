/*----------------------------------------------------------------------*
* IFFW.C  Support routines for writing IFF-85 files.          12/02/85
* (IFF is Interchange Format File.)
*
* By Jerry Morrison and Steve Shaw, Electronic Arts.
* This software is in the public domain.
*
* Original version was for the Commodore-Amiga computer.
* This version is compatible with PC, OSX, PS3, Wii, iPhone. 10/26/2008
*----------------------------------------------------------------------*/
#include "iff.h"




/* ---------- IFF Writer -----------------------------------------------*/

/* A macro to test if a chunk size is definite, i.e. not szNotYetKnown.*/
#define Known(size)   ( (size) != szNotYetKnown )

/* Yet another weird macro to make the source code simpler...*/
#define IfIffp(expr)  {if (iffp == IFF_OKAY)  iffp = (expr);}

/* ---------- OpenWIFF -------------------------------------------------*/

IFFP OpenWIFF(BPTR file, GroupContext *new0, int limit)
{
	register GroupContext *newtmp = new0;
	register IFFP iffp = IFF_OKAY;

	newtmp->parent       = NULL;
	newtmp->clientFrame  = NULL;
	newtmp->file         = file;
	newtmp->position     = 0;
	newtmp->bound        = limit;
	newtmp->ckHdr.ckID   = NULL_CHUNK;  /* indicates no current chunk */
	newtmp->ckHdr.ckSize = newtmp->bytesSoFar = 0;

	if (0 > Seek(file, 0, OFFSET_BEGINNING))   /* Go to start of the file.*/
		iffp = DOS_ERROR;
	else if ( Known(limit) && IS_ODD(limit) )
		iffp = CLIENT_ERROR;
	return(iffp);
}

/* ---------- StartWGroup ----------------------------------------------*/
IFFP StartWGroup( GroupContext* parent, int groupType,int groupSize,int subtype, GroupContext * newtmp)
{
	register IFFP iffp;

	iffp = PutCkHdr(parent, groupType, groupSize);
	IfIffp( IFFWriteBytes(parent, (char *)&subtype, sizeof(int)) );
	IfIffp( OpenWGroup(parent, newtmp) );
	return(iffp);
}

/* ---------- OpenWGroup -----------------------------------------------*/
IFFP OpenWGroup(GroupContext *parent0,GroupContext* new0)
{
	register GroupContext *parent = parent0;
	register GroupContext *newtmp    = new0;
	register int ckEnd;
	register IFFP iffp = IFF_OKAY;

	newtmp->parent       = parent;
	newtmp->clientFrame  = parent->clientFrame;
	newtmp->file         = parent->file;
	newtmp->position     = parent->position;
	newtmp->bound        = parent->bound;
	newtmp->ckHdr.ckID   = NULL_CHUNK;
	newtmp->ckHdr.ckSize = newtmp->bytesSoFar = 0;

	if ( Known(parent->ckHdr.ckSize) ) {
		ckEnd = newtmp->position + ChunkMoreBytes(parent);
		if ( newtmp->bound == szNotYetKnown || newtmp->bound > ckEnd )
			newtmp->bound = ckEnd;
	};

	if ( parent->ckHdr.ckID == NULL_CHUNK || /* not currently writing a chunk*/
		IS_ODD(newtmp->position) ||
		(Known(newtmp->bound) && IS_ODD(newtmp->bound)) )
		iffp = CLIENT_ERROR;
	return(iffp);
}

/* ---------- CloseWGroup ----------------------------------------------*/
IFFP CloseWGroup(GroupContext *old0)
{
	register GroupContext *old = old0;

	if ( old->ckHdr.ckID != NULL_CHUNK )  /* didn't close the last chunk */
		return(CLIENT_ERROR);

	if ( old->parent == NULL ) {     /* top level file context */
		GWriteFlush(old->file);
	}
	else {
		old->parent->bytesSoFar += old->position - old->parent->position;
		old->parent->position = old->position;
	};
	return(IFF_OKAY);
}

/* ---------- EndWGroup ------------------------------------------------*/
IFFP EndWGroup(GroupContext *old)
{
	register GroupContext *parent = old->parent;
	register IFFP iffp;


	iffp = CloseWGroup(old);
	IfIffp( PutCkEnd(parent) );
	return(iffp);
}

/* ---------- PutCk ----------------------------------------------------*/
IFFP PutCk(GroupContext *context, int ckID,int ckSize,char *data)
{
	register IFFP iffp = IFF_OKAY;

	if ( ckSize == szNotYetKnown )
		iffp = CLIENT_ERROR;
	IfIffp( PutCkHdr(context, ckID, ckSize) );
	IfIffp( IFFWriteBytes(context, data, ckSize) );
	IfIffp( PutCkEnd(context) );
	return(iffp);
}

/* ---------- PutCkHdr -------------------------------------------------*/
IFFP PutCkHdr(GroupContext *context0, int ckID, int ckSize)
{
	register GroupContext *context = context0;
	int minPSize = sizeof(ChunkHeader); /* physical chunk >= minPSize bytes*/

	/* CLIENT_ERROR if we're already inside a chunk or asked to write
	* other than one FORM, LIST, or CAT at the top level of a file */
	/* Also, non-positive int values are illegal and used for error codes.*/
	/* (We could check for other illegal IDs...)*/
	if ( context->ckHdr.ckID != NULL_CHUNK  ||  ckID <= 0 )
		return(CLIENT_ERROR);
	else if (context->parent == NULL)  {
		switch (ckID)  {
	   case FORM:  case LIST:  case CAT:  break;
	   default: return(CLIENT_ERROR);
		}
		if (context->position != 0)
			return(CLIENT_ERROR);
	}

	if ( Known(ckSize) ) {
		if ( ckSize < 0 )
			return(CLIENT_ERROR);
		minPSize += ckSize;
	};
	if ( Known(context->bound)  &&
		context->position + minPSize > context->bound )
		return(CLIENT_ERROR);

	context->ckHdr.ckID   = ckID;
	context->ckHdr.ckSize = ckSize;
	context->bytesSoFar   = 0;
	{
		context->ckHdr.ckSize = endianSwap32(context->ckHdr.ckSize);
		if (0 > fwrite(&context->ckHdr, 1,sizeof(ChunkHeader),context->file))
			return(DOS_ERROR);
		context->ckHdr.ckSize = endianSwap32(context->ckHdr.ckSize);
	}
	context->position += sizeof(ChunkHeader);
	return(IFF_OKAY);
}

/* ---------- IFFWriteBytes ---------------------------------------------*/
IFFP IFFWriteBytes(GroupContext *context0, char *data, int nBytes)
{
	register GroupContext *context = context0;

	if ( context->ckHdr.ckID == NULL_CHUNK  ||   /* not in a chunk */
		nBytes < 0  ||            /* negative nBytes */
		(Known(context->bound)  &&      /* overflow context */
		context->position + nBytes > context->bound)  ||
		(Known(context->ckHdr.ckSize)  &&      /* overflow chunk */
		context->bytesSoFar + nBytes > context->ckHdr.ckSize) )
	{
		return(CLIENT_ERROR);
	}

	if (0 > fwrite(data, 1,nBytes,context->file))
		return(DOS_ERROR);

	context->bytesSoFar += nBytes;
	context->position   += nBytes;
	return(IFF_OKAY);
}

/* ---------- PutCkEnd -------------------------------------------------*/
IFFP PutCkEnd(GroupContext *context0)
{
	register GroupContext *context = context0;
	WORD zero = 0;   /* padding source */

	if ( context->ckHdr.ckID == NULL_CHUNK )  /* not in a chunk */
		return(CLIENT_ERROR);

	if ( context->ckHdr.ckSize == szNotYetKnown ) {
		/* go back and set the chunk size to bytesSoFar */
		int offset = context->bytesSoFar+sizeof(int);
		if ( 0 > GSeek(context->file,
			-(offset),
			OFFSET_CURRENT))
		{
			return(DOS_ERROR);
		} else
		{
			context->bytesSoFar = endianSwap32(context->bytesSoFar);
			if (0 > fwrite(&context->bytesSoFar, 1,sizeof(int),context->file))
				return (DOS_ERROR);
			context->bytesSoFar = endianSwap32(context->bytesSoFar);
			if (0 > GSeek(context->file, context->bytesSoFar, OFFSET_CURRENT)  )
				return (DOS_ERROR);
		}


	}
	else {  /* make sure the client wrote as many bytes as planned */
		if ( context->ckHdr.ckSize != context->bytesSoFar )
			return(CLIENT_ERROR);
	};

	/* Write a pad byte if needed to bring us up to an even boundary.
	* Since the context end must be even, and since we haven't
	* overwritten the context, if we're on an odd position there must
	* be room for a pad byte. */
	if ( IS_ODD(context->bytesSoFar) ) {
		if ( 0 > fwrite(&zero, 1,1,context->file) )
			return(DOS_ERROR);
		context->position += 1;
	};

	context->ckHdr.ckID   = NULL_CHUNK;
	context->ckHdr.ckSize = context->bytesSoFar = 0;
	return(IFF_OKAY);
}

