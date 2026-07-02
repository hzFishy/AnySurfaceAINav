// By hzFishy - 2026 - Do whatever you want with it.

#pragma once

#include "Data/SANSurfaceTypes.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "Core/SANCore.h"
#include "SANAnySurfaceNavLibrary.generated.h"
class USANAnySurfaceNavSettings;


UCLASS(DisplayName="Any Surface Nav Library", Category="SAN")
class ANYSURFACEAINAV_API USANAnySurfaceNavLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	
public:
	UFUNCTION(BlueprintCallable, Category="SAN")
	static bool FindAnySurfacePathSync(const FSANFindPathRequest& Request, FSANFindPathResult& Result);
	
	UFUNCTION(BlueprintPure, Category="SAN")
	static bool IsPathResultEmpty(const FSANFindPathResult& PathResult);
	
protected:
	static int32 FillGapsLoopCount;
#if SAN_WITH_DEBUG
	static int32 FillGapsLoopCountDebug;
#endif
	
	static bool GetBestSurface(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams, const FVector PointLocation, float& Radius, const FSANSurfaceHitResult& PreviousSurface, TArray<FSANSurfaceHitResult>& OutBestSurfaces);
	
	static bool GetBestSurfaceInternal(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams, const FVector PointLocation, float& Radius, TArray<FHitResult>& HitResults);
	
	static void RemoveSimilarPoints(UWorld* World, const USANAnySurfaceNavSettings* Settings, TArray<FSANSurfaceHitResult>& SurfacePoints, const FName VLogName);
	
	static void KeepShortestDistancePoints(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams, float AgentRadius, const TArray<FSANSurfaceHitResult>& InRawSurfaceHits, TArray<FSANSurfaceHitResult>& OutFilteredRawSurfaceHits);
	
	static bool IsLineBlocking(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams, float AgentRadius, const FSANSurfaceHitResult& Start, const FSANSurfaceHitResult& End, FHitResult& OutHitResult);
	static bool IsPointBlocked(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams, float AgentRadius, const FVector& PointLocation, FHitResult& OutHitResult);
	
	/** 
	 *  Add extra surfaces between given points, recursive.
	 *  @param RawSurfaceHits The actual surface hits
	 *  @param SurfaceExtraPoints Extra points if scene queries returned multiple surfaces at a single point
	 */
	static bool FillGaps(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams, float AgentRadius, TArray<FSANSurfaceHitResult>& RawSurfaceHits, TArray<FSANSurfaceHitResult>& SurfaceExtraPoints);
	
	static int32 FindBestInBetweenIndex(const FSANSurfaceHitResult& NewSurface, const TArray<FSANSurfaceHitResult>& Surfaces);
	
	static void MakeCollisionQueryParamsFromRequest(const FSANFindPathRequest& Request, FCollisionQueryParams& Params);
	
	static void SmoothSegment(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams, float AgentRadius, const FSANSurfaceHitResult& StartSurface, const FSANSurfaceHitResult& EndSurface, TArray<FSANSurfaceHitResult>& OutNewSurfaces);
};
