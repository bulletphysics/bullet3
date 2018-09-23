#pragma once
#ifndef GWEN_CONTROLS_DIALOGS_FILEOPEN_H
#define GWEN_CONTROLS_DIALOGS_FILEOPEN_H

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
// The REAL function.
// If bUseSystem is used, it may use the system's modal dialog - which
// will steal focus and pause the rest of GWEN until it's continued.
//
void GWEN_EXPORT FileOpenEx(bool bUseSystem, const String& Name, const String& StartPath, const String& Extension, Gwen::Event::Handler* pHandler = NULL, Gwen::Event::Handler::FunctionStr fnCallback = NULL);

//
// Templated function simply to avoid having to manually cast the callback function.
//
template <typename A>
void FileOpen(bool bUseSystem, const String& Name, const String& StartPath, const String& Extension, Gwen::Event::Handler* pHandler = NULL, A fnCallback = NULL)
{
	FileOpenEx(bUseSystem, Name, StartPath, Extension, pHandler, (Gwen::Event::Handler::FunctionStr)fnCallback);
}

}  // namespace Dialogs
}  // namespace Gwen
#endif
