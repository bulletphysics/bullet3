#ifndef BT_OVERRIDE_H
#define BT_OVERRIDE_H

#if (defined(__cplusplus) && __cplusplus >= 201103L) || (defined(_MSC_VER) && _MSC_VER >= 1600) // FIXME: msvc may need extra treatment if not /Z:__cplusplus is given as compile flag
// give us a compile error if any signatures of overriden methods is changed
#define BT_OVERRIDE override
#endif
#ifndef BT_OVERRIDE
#define BT_OVERRIDE
#endif

#endif // BT_OVERRIDE_H
