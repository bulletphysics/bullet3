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

#ifndef __DAE__ERROR__
#define __DAE__ERROR__

/** Success */
#define DAE_OK 0 
/** Fatal Error, should never be returned unless there is a bug in the library. */
#define DAE_ERR_FATAL -1
/** Call invalid, the combination of parameters given is invalid. */
#define DAE_ERR_INVALID_CALL -2
/** IO error, the file hasn't been found or there is a problem with the IO plugin. */
#define DAE_ERR_BACKEND_IO -100
/** The IOPlugin backend wasn't able to successfully validate the data. */
#define DAE_ERR_BACKEND_VALIDATION -101
/** The IOPlugin tried to write to a file that already exists and the "replace" parameter was set to false */
#define DAE_ERR_BACKEND_FILE_EXISTS -102
/** Error in the syntax of the query. */
#define DAE_ERR_QUERY_SYNTAX -200
/** No match to the search criteria. */
#define DAE_ERR_QUERY_NO_MATCH -201
/** A document with that name already exists. */
#define DAE_ERR_COLLECTION_ALREADY_EXISTS -202
/** A document with that name does not exist. */
#define DAE_ERR_COLLECTION_DOES_NOT_EXIST -203
/** Function is not implemented. */
#define DAE_ERR_NOT_IMPLEMENTED -1000

/** Gets the ASCII error string.  
* @param errorCode Error code returned by a function of the API.
* @return Returns an English string describing the error.
*/
const char *daeErrorString(int errorCode);

#endif //__DAE__ERROR__
