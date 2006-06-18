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

#ifndef __DAE_STLDATABASE__
#define __DAE_STLDATABASE__

#include <stdio.h>

#include <vector>
#include <algorithm>
#include <functional>

#include <dae/daeElement.h>
#include <dae/daeDatabase.h>

/**
 * The @c daeSTLDatabase class derives from @c daeDatabase and implements
 * the default database.
 */
class daeSTLDatabase : public daeDatabase
{
public:
	/**
	  * Constructor
	  */
	daeSTLDatabase();
	/**
	  * Destructor
	  */
	virtual ~daeSTLDatabase();

public:
	// Element Types of all Elements
	virtual daeUInt			getTypeCount();
	virtual daeString		getTypeName(daeUInt index);
	virtual daeInt			setMeta(daeMetaElement *_topMeta);

	// Documents
	virtual daeInt			insertDocument(const char *name, daeElement* dom, daeDocument** document = NULL);
	virtual daeInt			insertDocument(daeString name, daeDocument** document = NULL);
	virtual daeInt			createDocument(daeString name, daeElement* dom, daeDocument** document = NULL);
	virtual daeInt			createDocument(daeString name, daeDocument** document = NULL);
	virtual daeInt			insertDocument( daeDocument *c );

	virtual daeInt			removeDocument(daeDocument* document);
	virtual daeUInt			getDocumentCount();
	virtual daeDocument*	getDocument(daeUInt index);
	virtual daeDocument*	getDocument(daeString name);
	virtual daeString		getDocumentName(daeUInt index);
	virtual daeBool			isDocumentLoaded(daeString name);

	// Elements 
	virtual daeInt			insertElement(daeDocument* document, daeElement* element);  
	virtual daeInt			removeElement(daeDocument* document, daeElement* element); 
	virtual daeInt			clear();
	virtual void			validate();
	virtual daeUInt getElementCount(daeString name = NULL,
	                              daeString type = NULL,
	                              daeString file = NULL);
	virtual daeInt getElement(daeElement** pElement, 
	                        daeInt index,
	                        daeString name = NULL,
	                        daeString type = NULL,
	                        daeString file = NULL);

	// Generic Query
	virtual daeInt queryElement(daeElement** pElement, daeString genericQuery);

private:

	/**
	 * A struct to describe a cell in the STL run-time database.
	 */
	typedef struct
	{
		daeElement* element;
		daeString name;
		daeString type;
		daeDocument *document;
	} DAE_STL_DATABASE_CELL;
	
	/**
	 * Sorting structure.
	 */
	struct daeSTLDatabaseLess: public std::binary_function<DAE_STL_DATABASE_CELL,DAE_STL_DATABASE_CELL,bool>
	{
		bool operator() (const DAE_STL_DATABASE_CELL& x, const DAE_STL_DATABASE_CELL& y) const
		{
			int res = strcmp(x.type,y.type);
			if (res != 0)
				return res<0;
			else
				return strcmp(x.name,y.name)<0;
		}
	};
	
	/**
	 * Sorting structure.
	 */
	struct daeSTLDatabaseTypeLess: public std::binary_function<DAE_STL_DATABASE_CELL,DAE_STL_DATABASE_CELL,bool>
	{
		bool operator() (const DAE_STL_DATABASE_CELL& x, const DAE_STL_DATABASE_CELL& y) const
		{
			int res = strcmp(x.type,y.type);
			return res<0;
		}
	};

	std::vector<DAE_STL_DATABASE_CELL> elements;
	std::vector<daeDocument*> documents;
	bool validated;
	daeMetaElement* topMeta;

	//!!!ACL removing this function.
	/*daeInt getElementByType(daeElement** pElement, 
									 int *currentIndex,
									 int index,
									 daeString name = NULL,
									 daeString type = NULL,
									 daeString file = NULL);*/

	daeInt insertChildren( daeDocument *c, daeElement *element );
	daeInt removeChildren( daeDocument *c, daeElement *element );

};

#endif // __DAE_STLDATABASE__
