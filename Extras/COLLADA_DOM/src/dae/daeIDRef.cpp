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

#include <dae/daeIDRef.h>
#include <dae/daeDatabase.h>
daeIDRefResolverPtrArray daeIDRefResolver::_KnownResolvers;

void
daeIDRef::initialize()
{
	id = NULL;
	element = NULL;
	container = NULL;
}

daeIDRef::~daeIDRef()
{
	reset();
}

daeIDRef::daeIDRef()
{
	initialize();
	reset();
}

daeIDRef::daeIDRef(daeString IDRefString)
{
	initialize();
	setID(IDRefString);
	validate();
}

daeIDRef::daeIDRef(daeIDRef& copyFrom)
{
	initialize();
	element = copyFrom.element;
	setID(copyFrom.getID());
	state = copyFrom.state;
}

void
daeIDRef::copyFrom(daeIDRef& copyFrom)
{
	element = copyFrom.element;
	setID(copyFrom.getID());
	state = copyFrom.state;
}

daeString emptyID = "";

void
daeIDRef::reset()
{
	if ((id != NULL) && (strcmp(id, emptyID) != 0))
	  daeMemorySystem::free("idref",(void*)id);

	id = emptyID;
}

daeString safeCreateID(daeString src)
{
	if (src == NULL)
		return emptyID;
	daeChar* ret = (daeChar*)daeMemorySystem::malloc("idref",strlen(src)+1);
	if (ret == NULL)
		return emptyID;
	strcpy(ret,src);
	
	return ret;
}

void
daeIDRef::setID(daeString _IDString)
{
	reset();

	id = safeCreateID(_IDString);
	
	state = id_loaded;
}

void
daeIDRef::print()
{
	fprintf(stderr,"id = %s\n",id);
	fflush(stderr);
}

daeString
daeIDRef::getID() const
{
	return id;
}

void
daeIDRef::validate()
{
	state = id_pending;
}

void
daeIDRef::resolveElement( daeString typeNameHint )
{
	if (state == id_empty)
		return;
	
	if (state == id_loaded)
		validate();
	
	daeIDRefResolver::attemptResolveElement(*((daeIDRef*)this), typeNameHint );
}

void
daeIDRef::resolveID()
{
	if (state == id_empty) {
		if (element != NULL)
			setID(element->getID());
		else
			state = id_failed_invalid_reference;
	}
}

void
daeIDRefResolver::attemptResolveElement(daeIDRef& id, daeString typeNameHint)
{
	int i;
	int cnt = (int)_KnownResolvers.getCount();

	for(i=0;i<cnt;i++)
		if (_KnownResolvers[i]->resolveElement(id, typeNameHint))
			return;
}

void
daeIDRefResolver::attemptResolveID(daeIDRef& id)
{
	int i,cnt = (int)_KnownResolvers.getCount();

//	daeBool foundProtocol = false;
	for(i=0;i<cnt;i++)
		if (_KnownResolvers[i]->resolveID(id))
			return;

#if defined(_DEBUG) && defined(WIN32)
	fprintf(stderr,
			"daeIDRefResolver::attemptResolveID(%s) - failed\n",
			id.getID());
#endif
			
}

daeIDRefResolver::daeIDRefResolver()
{
	_KnownResolvers.append((daeIDRefResolver*)this);
}

daeIDRefResolver::~daeIDRefResolver()
{
	_KnownResolvers.remove((daeIDRefResolver*)this);
}



daeDefaultIDRefResolver::daeDefaultIDRefResolver(daeDatabase* database)
{
	_database = database;
}

daeDefaultIDRefResolver::~daeDefaultIDRefResolver()
{
}

daeBool
daeDefaultIDRefResolver::resolveID(daeIDRef& id)
{
	(void)id; 
	return false;
}

daeString
daeDefaultIDRefResolver::getName()
{
	return "DefaultIDRefResolver";
}

daeBool
daeDefaultIDRefResolver::resolveElement(daeIDRef& idref, daeString typeNameHint)
{
	if (idref.getState() == daeIDRef::id_loaded)
		idref.validate();
	
	daeElement* resolved = NULL;
	int status;

	daeString id = idref.getID();

	status = _database->getElement(&resolved,0,id,typeNameHint,NULL);

	idref.setElement( resolved );

	if (status||(resolved==NULL)) {
		idref.setState( daeIDRef::id_failed_id_not_found );
		fprintf(stderr,
				"daeDefaultIDRefResolver::resolveElement() - failed to resolve %s\n",
				idref.getID());
		fflush(stderr);
		return false;
	}

	idref.setState( daeIDRef::id_success );
	return true;
}




