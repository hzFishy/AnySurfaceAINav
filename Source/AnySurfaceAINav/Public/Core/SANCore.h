// By hzFishy - 2026 - Do whatever you want with it.

#pragma once

#include "Console/FUConsole.h"

#if FU_WITH_CONSOLE
	#define SAN_WITH_DEBUG 1
#else
	#define SAN_WITH_DEBUG 0
#endif

DECLARE_LOG_CATEGORY_EXTERN(LogAnySurfaceAINav, Display, All);

#define SAN_LOG_D(FORMAT, ...)							  _FU_LOG_OBJECT_D(LogAnySurfaceAINav, FORMAT, ##__VA_ARGS__)
#define SAN_LOG_W(FORMAT, ...)							  _FU_LOG_OBJECT_W(LogAnySurfaceAINav, FORMAT, ##__VA_ARGS__)
#define SAN_VLOG_D( FORMAT, ...)						  UE_VLOG_UELOG(this, LogAnySurfaceAINav, Display, TEXT(FORMAT), ##__VA_ARGS__)
#define SAN_VLOG_W(FORMAT, ...)						   	  UE_VLOG_UELOG(this, LogAnySurfaceAINav, Warning, TEXT(FORMAT), ##__VA_ARGS__)

#define SAN_LOG_Static_D(FORMAT, ...)					  _FU_LOG_STATIC_D(LogAnySurfaceAINav, FORMAT, ##__VA_ARGS__)
#define SAN_LOG_Static_W(FORMAT, ...)					  _FU_LOG_STATIC_W(LogAnySurfaceAINav, FORMAT, ##__VA_ARGS__)
#define SAN_VLOG_Static_D(LogOwner, FORMAT, ...)		  UE_VLOG_UELOG(LogOwner, LogAnySurfaceAINav, Display, TEXT(FORMAT), ##__VA_ARGS__)
#define SAN_VLOG_Static_W(LogOwner, FORMAT, ...)		  UE_VLOG_UELOG(LogOwner, LogAnySurfaceAINav, Warning, TEXT(FORMAT), ##__VA_ARGS__)
