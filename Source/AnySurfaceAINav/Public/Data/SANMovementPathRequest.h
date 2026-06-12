// By hzFishy - 2026 - Do whatever you want with it.

#pragma once

#include "SANSurfaceTypes.h"
#include "SANMovementPathRequest.generated.h"


USTRUCT(BlueprintType, DisplayName="SAN Movement Path Request")
struct FSANMovementPathRequest
{
	GENERATED_BODY()
	
	FSANMovementPathRequest();
	
	UPROPERTY(BlueprintReadOnly, EditAnywhere)
	FSANFindPathRequest CachedPathRequest;
	
	UPROPERTY(BlueprintReadOnly, EditAnywhere)
	FSANFindPathResult CachedPathResult;
	
	bool IsValid() const;
};
