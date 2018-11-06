#pragma once
#ifndef GWEN_CONTROLS_DIALOGS_FILESAVE_H
#define GWEN_CONTROLS_DIALOGS_FILESAVE_H

#include "Gwen/Gwen.h"

namespace Gwen
{
namespace Dialogs
{
// Usage:
//
// Gwen::Dialogs::FileOpen( true, "Open Map", "C:\my\folder\", "My Map Format|*.bmf", this, &MyClass::OpenFilename );
//

//
// Callback function, for success
//
typedef void (Event::Handler::*FileSaveSuccessCallback)(const String& filename);

//
// The REAL function.
// If bUseSystem is used, it may use the system's modal dialog - which
// will steal focus and pause the rest of GWEN until it's continued.
//
void GWEN_EXPORT FileSaveEx(bool bUseSystem, const String& Name, const String& StartPath, const String& Extension, Gwen::Event::Handler* pHandler = NULL, Gwen::Event::Handler::FunctionStr fnCallback = NULL);

//
// Templated function simply to avoid having to manually cast the callback function.
//
template <typename A>
void FileSave(bool bUseSystem, const String& Name, const String& StartPath, const String& Extension, Gwen::Event::Handler* pHandler = NULL, A fnCallback = NULL)
{
	FileSaveEx(bUseSystem, Name, StartPath, Extension, pHandler, (Gwen::Event::Handler::FunctionStr)fnCallback);
}

}  // namespace Dialogs
}  // namespace Gwen
#endif
