#ifndef _DEBUG_H_
#define _DEBUG_H_

#include "util/logger.h"

#define _DEBUG

#ifdef _DEBUG
#define DEBUG(format, ...)  printf( _COLOR_BLUE "[%s]: " format _COLOR_NONE, __func__, ##__VA_ARGS__)
#else
#define DEBUG(format, ...)
#endif

#define TO_STRING(var) #var

#endif // _DEBUG_H_
