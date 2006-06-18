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
#ifndef __DOM__
#define __DOM__

class daeMetaElement;

extern daeString COLLADA_VERSION;
extern daeString COLLADA_NAMESPACE;

// Register all types
void registerDomTypes();

// Register all elements
daeMetaElement* registerDomElements();


#endif // __DOM_INTERFACE__
