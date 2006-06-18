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

#ifndef __DAE_IOPLUGIN__
#define __DAE_IOPLUGIN__

#include <dae/daeTypes.h>
class daeDatabase;
class daeMetaElement;
class daeURI;
class daeDocument;

/**
* The @c daeIOPlugin class provides the input/output plugin interface, which is
* the interface between the COLLADA runtime and the backend storage. A native
* COLLADA XML plugin implementation is provided along with this interface.
*/
class daeIOPlugin
{
public:	
	/**
	* Destructor
	*/
	virtual ~daeIOPlugin() {}	
	/** 
	* Sets the top meta object.
	* Called by @c dae::setIOPlugin() when the IO plugin changes. It passes to this function the
	* top meta object, which is the root of a 
    * hierarchy of @c daeMetaElement objects. This top meta object is capable of creating
	* any of the root objects in the DOM tree.
	* @param topMeta Top meta object to use to create objects to fill the database.
	* @return Returns DAE_OK if successful, otherwise returns a negative value defined in daeError.h.
	*/
	virtual daeInt setMeta(daeMetaElement *topMeta) = 0;

	/** @name Database setup	 */
	//@{
	/** 
	* Sets the database to use.
	* All @c daeIOPlugins use the same interface to the @c daeDatabase, 
	* @c setDatabase() tells the @c daeIOPlugin which @c daeDatabase object it should use
	* for storage and queries.
	* @param database Database to set.
	*/
	virtual void setDatabase(daeDatabase* database) = 0;
	//@}


	/** @name Operations	 */
	//@{
	/** 
	* Imports content into the database from an input.
	* The input can be a file, a database or another runtime.
	* @param uri the URI of the COLLADA document to load, not all plugins accept all types of URIs,
	* check the documentation for the IO plugin you are using.
	* @param docBuffer A string containing the text of the document to load. This is an optional attribute
	* and should only be used if the document has already been loaded into memory.
	* @return Returns DAE_OK if successfully loaded, otherwise returns a negative value defined in daeError.h.
	* @see @c daeInterface::load().
	*/
	virtual daeInt read(daeURI& uri, daeString docBuffer) = 0;

	/** @name Operations	 */
	//@{
	/**
	* Writes a specific document to an output.
	* @param name URI to write the document to, not all IO plugins support all types of URIs
	* check the documentation for the IO plugin you are using.
	* @param document Pointer to the document that we're going to write out.
	* @param replace True if write should overwrite an existing file. False otherwise.
	* @return Returns DAE_OK if success, a negative value defined in daeError.h otherwise.
	* @see @c daeInterface::saveAS()
	*/
	virtual daeInt write(daeURI *name, daeDocument *document, daeBool replace) = 0;
	//@}
	
	/** @name Load/Save Progress */
	//@{
	/**
	* Gets the progress of @c load() operation.
	* This function can be used from another thread to check the progress of a @c load() 
	* operation. The user can update a progress bar <tt> bytesParsed/totalBytes </tt> gives the
	* percentage of progress of the operation.
	* @param bytesParsed Pointer to an integer that receives the number of bytes already 
	* consumed from the file, can be NULL if the user don't want to retrieve this information.
	* @param lineNumber Pointer to an integer that receives the number of lines read so far,
	* can be NULL.
	* @param totalBytes Pointer to an integer that receives the total number of bytes in the 
	* file currently beeing loaded, can be NULL.
	* @param reset Indicates whether to reset the counters. A value of false is the default behaviour
	* that fits most usage. Set it to true to reset 
	* the <tt><i> bytesParsed </i></tt> and <tt><i> lineNumber </i></tt> counters. The system resets the counter at the beginning of 
	* each file.
	*/
	virtual void getProgress(daeInt* bytesParsed,
							 daeInt* lineNumber,
							 daeInt* totalBytes,
							 daeBool reset = false ) = 0;
	//@}
};

#endif // __DAE_IOPLUGIN__
