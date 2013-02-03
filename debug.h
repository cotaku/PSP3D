#ifndef DEBUG_H
#define DEBUG_H

#include <pspdebug.h>

#define DEBUG 1

#define DebugPrint(fmt, ...) do { if (DEBUG) pspDebugScreenPrintf("!!%s(%d):"fmt, __func__, __LINE__, ##__VA_ARGS__); } while (0)
#define DebugQuickPrint(fmt, ...) do { if (DEBUG) pspDebugScreenPrintf(fmt, ##__VA_ARGS__); } while (0)

#endif
