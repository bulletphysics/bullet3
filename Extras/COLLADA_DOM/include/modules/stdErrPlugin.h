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

#ifndef _STDERR_PLUGIN_
#define _STDERR_PLUGIN_

#include <dae/daeTypes.h>
#include <dae/daeErrorHandler.h>
/**
 * The @c stdErrPlugin class is the default implementation of daeErrorHandler. It routes the Error
 * and Warning messaged to stdout.
 */
class DLLSPEC stdErrPlugin : public daeErrorHandler {
public:
	stdErrPlugin();
	virtual ~stdErrPlugin();

public:
	void handleError( daeString msg );
	void handleWarning( daeString msg );
};

#endif
