// By hzFishy - 2026 - Do whatever you want with it.

#pragma once

#include "SANSurfaceTypes.generated.h"

USTRUCT(BlueprintType, DisplayName="SAN Surface Hit Result")
struct ANYSURFACEAINAV_API FSANSurfaceHitResult
{
	FSANSurfaceHitResult();
	
	GENERATED_BODY()
	
	UPROPERTY(BlueprintReadOnly)
	FVector HitLocation;
	
	UPROPERTY(BlueprintReadOnly)
	FVector HitNormal;
	
	bool IsValid() const;
};

USTRUCT(BlueprintType, DisplayName="SAN Find Path Request")
struct ANYSURFACEAINAV_API FSANFindPathRequest
{
	FSANFindPathRequest();
	
	GENERATED_BODY()
	
	UPROPERTY(BlueprintReadWrite)
	const UObject* WorldContextObject;
	
	UPROPERTY(BlueprintReadWrite)
	FVector StartLocation;
	
	UPROPERTY(BlueprintReadWrite)
	FVector EndLocation;
	
	UPROPERTY(BlueprintReadWrite)
	float AgentRadius;
	
	bool IsValid() const;
};

USTRUCT(BlueprintType, DisplayName="SAN Find Path Result")
struct ANYSURFACEAINAV_API FSANFindPathResult
{
	FSANFindPathResult();
	
	GENERATED_BODY()
	
	TArray<FNavPathPoint> NavPathPoints;
	
	UPROPERTY(BlueprintReadOnly)
	TArray<FSANSurfaceHitResult> SurfaceHitResults;
	
	void SurfacesToPositions(TArray<FVector>& PositionsArray, float SurfaceDistance) const;
	
	bool IsEmpty() const;
};
