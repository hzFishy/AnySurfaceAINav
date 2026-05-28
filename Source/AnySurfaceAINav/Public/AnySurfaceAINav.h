// By hzFishy - 2026 - Do whatever you want with it.

#pragma once

#include "Modules/ModuleManager.h"

class FAnySurfaceAINavModule : public IModuleInterface
{
	
public:
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};
