// By hzFishy - 2026 - Do whatever you want with it.

#pragma once

#include "Data/SANSurfaceTypes.h"
#include "Kismet/BlueprintFunctionLibrary.h"
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
	
	static bool GetBestSurface(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FVector PointLocation, const TArray<FSANSurfaceHitResult>& PreviousSurfaces, TArray<FSANSurfaceHitResult>& OutBestSurfaces);
	
	static bool GetBestSurfaceInternal(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FVector PointLocation, float Radius, TArray<FHitResult>& HitResults);

	/** FilteredRawSurfaceHits will be edited */
	static void KeepShortestDistancePoints(UWorld* World, const USANAnySurfaceNavSettings* Settings, const TArray<FSANSurfaceHitResult>& InRawSurfaceHits, TArray<FSANSurfaceHitResult>& OutFilteredRawSurfaceHits);
	
	static void FindAndInsertSubdivisions(UWorld* World, const USANAnySurfaceNavSettings* Settings, const TArray<FSANSurfaceHitResult>& InSurfaceHitResults, TArray<FSANSurfaceHitResult>& OutSurfaceHitResults);
	
	/** 
	 *  
	 *  @param RawSurfaceHits The actual surface hits path
	 */
	static bool FillGaps(UWorld* World, const USANAnySurfaceNavSettings* Settings, TArray<FSANSurfaceHitResult>& PreviousSurfaces, TArray<FSANSurfaceHitResult>& RawSurfaceHits);
};
