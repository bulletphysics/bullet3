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

#ifndef _DAE_ERROR_HANDLER_
#define _DAE_ERROR_HANDLER_

#include <dae/daeTypes.h>

/**
 * The @c daeErrorHandler class is a plugin that allows the use to overwrite how error and warning
 * messages get handled in the client application. An example of this would be a class that reports
 * the message to a gui front end instead of just printing on stdout.
 */
class DLLSPEC daeErrorHandler {
public:
	/**
	 * Constructor.
	 */
	daeErrorHandler();
	/**
	 * Destructor.
	 */
	virtual ~daeErrorHandler();

	/**
	 * This function is called when there is an error and a string needs to be sent to the user.
	 * You must overwrite this function in your plugin.
	 * @param msg Error message.
	 */
	virtual void handleError( daeString msg ) = 0;
	/**
	 * This function is called when there is a warning and a string needs to be sent to the user.
	 * You must overwrite this function in your plugin.
	 * @param msg Warning message.
	 */
	virtual void handleWarning( daeString msg ) = 0;

	/**
	 * Sets the daeErrorHandler to the one specified.
	 * @param eh The new daeErrorHandler to use. Passing in NULL results in the default plugin being used.
	 */
	static void setErrorHandler( daeErrorHandler *eh );
	/**
	 * Returns the current daeErrorHandlerPlugin. DaeErrorHandler implements a singleton design pattern
	 * so you can get the current daeErrorHandler statically.
	 * @return The current daeErrorHandler.
	 */
	static daeErrorHandler *get();

private:
	static daeErrorHandler *_instance;
	static daeBool _default;
};

#endif
