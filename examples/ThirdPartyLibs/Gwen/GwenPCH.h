/*
GWEN Library - Precompiled Header
*/

#ifndef GWEN_PCH_H
#define GWEN_PCH_H

// Standard Library Headers - Most expensive
#include <string>
#include <vector>
#include <map>
#include <list>
#include <set>
#include <algorithm>
#include <cmath>
#include <cassert>
#include <sstream>
#include <iostream>

// Windows headers (if on Windows) - without LEAN_AND_MEAN for Gwen
#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <commdlg.h>
#endif

#endif // GWEN_PCH_H
