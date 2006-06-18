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

#include <dae/daeDocument.h>

void daeDocument::addExternalReference( daeURI &uri ) {
	if ( uri.getContainer() == NULL || uri.getContainer()->getDocument() != this ) {
		return;	
	}	
	size_t idx;
	daeURI tempURI( uri.getURI(), true );
	daeStringRef docURI( tempURI.getURI() );
	if ( referencedDocuments.find( docURI, idx ) == DAE_OK ) {
		externalURIs[idx]->appendUnique( &uri );
	}
	else {
		referencedDocuments.append( docURI );
		idx = externalURIs.append( new daeTArray<daeURI*> );
		externalURIs[idx]->append( &uri );
	}
}

void daeDocument::removeExternalReference( daeURI &uri ) {
	for( unsigned int i = 0; i < externalURIs.getCount(); i++ ) {
		for ( unsigned int j = 0; j < externalURIs[i]->getCount(); j++ ) {
			daeURI *tempURI = externalURIs[i]->get(j);
			if ( tempURI == &uri ) {
				//found the uri. now remove it
				externalURIs[i]->removeIndex(j);
				if ( externalURIs[i]->getCount() == 0 ) {
					externalURIs.removeIndex(i);
					referencedDocuments.removeIndex(i);
				}
				return;
			}
		}
	}
}

void daeDocument::resolveExternals( daeString docURI ) {
	size_t idx;
	if ( referencedDocuments.find( docURI, idx ) == DAE_OK ) {
		for ( unsigned int j = 0; j < externalURIs[idx]->getCount(); j++ ) {
			daeURI *tempURI = externalURIs[idx]->get(j);
			tempURI->resolveElement();
		}
		return;
	}
}
