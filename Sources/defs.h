#ifndef _DEFS_H_
#define _DEFS_H_

#include <stdio.h>


#ifdef _OWNER_
#define _GLDECL_  __align(4)
#else
#define _GLDECL_  extern
#endif


#define __CREV16(x)   (((x>>8)&0xFF) | (x<<8))
#define ELM_OFFSET(type, elm)  ((uint32_t)(&((type *)0)->elm))


#define LOG_LEVEL     0

#ifdef _DEBUG_
#define _MSG_LOG(lvl, ...) if(lvl>=LOG_LEVEL) printf(__VA_ARGS__);
#else
#define _MSG_LOG(lvl, ...)
#endif


#endif

