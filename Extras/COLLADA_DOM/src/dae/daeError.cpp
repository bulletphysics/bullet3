/*
 * Copyright 2006 Sony Computer Entertainment Inc.
 *
 * Licensed under the SCEA Shared Source License, Version 1.0 (the "License"); you may not use this 
 * file except in compliance with the License. You may obtain a copy of the License at:
 * http://research.scea.com/scea_shared_source_license.html
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License 
 * is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or 
 * implied. See the License for the specific language governing permissions and limitations under the 
 * License. 
 */

#include <dae/daeError.h>

typedef struct 
{
	int errCode;
	const char *errString;
} DAEERROR;

static DAEERROR errorsArray[] =
{
	{ DAE_OK, "Success" },
	{ DAE_ERR_INVALID_CALL, "Invalid function call" },
	{ DAE_ERR_FATAL, "Fatal" },
	{ DAE_ERR_BACKEND_IO, "Backend IO" },
	{ DAE_ERR_BACKEND_VALIDATION, "Backend validation" },
	{ DAE_ERR_QUERY_SYNTAX, "Query syntax" },
	{ DAE_ERR_QUERY_NO_MATCH, "Query no match" },
	{ DAE_ERR_COLLECTION_ALREADY_EXISTS, "A document with the same name exists already" },
	{ DAE_ERR_COLLECTION_DOES_NOT_EXIST, "No document is loaded with that name or index" }, 
	{ DAE_ERR_NOT_IMPLEMENTED, "This function is not implemented in this reference implementation" },
};

const char *daeErrorString(int errorCode)
{
	int iErrorCount = (int)(sizeof(errorsArray)/sizeof(DAEERROR));
	for (int i=0;i<iErrorCount;i++)
	{
		if (errorsArray[i].errCode == errorCode)
			return errorsArray[i].errString;
	}
	return "Unknown Error code";
}
