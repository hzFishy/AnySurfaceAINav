// By hzFishy - 2026 - Do whatever you want with it.

#pragma once

#include "Engine/DeveloperSettings.h"
#include "SANAnySurfaceNavSettings.generated.h"


UCLASS(DisplayName="Any Surface Nav Settings", DefaultConfig, Config=Game)
class ANYSURFACEAINAV_API USANAnySurfaceNavSettings : public UDeveloperSettings
{
	GENERATED_BODY()
	
public:
	USANAnySurfaceNavSettings();
	
	static const USANAnySurfaceNavSettings* Get();
	
	
	/** 
	 * Collision Profile to use when tracing for surfaces.
	 * This is used with a multi trace so the surfaces that should be taken into account needs to overlap.
	 * Other surfaces shoulds be ignored.
	 */
	UPROPERTY(EditAnywhere, Config, Category="Collision")
	FCollisionProfileName OverlapSurfaceCollisionProfile;
	
	/** 
	 * Collision Profile to use when tracing for surfaces.
	 * This is used after the multi trace to filter out surfaces that should block.
	 * Surfaces that should be taken into account needs to block.
	 * Other surfaces shoulds be ignored.
	 * 
	 * This is generally the same as OverlapSurfaceCollisionProfile but instead of overlapping it blocks.
	 */
	UPROPERTY(EditAnywhere, Config, Category="Collision")
	FCollisionProfileName BlockSurfaceCollisionProfile;

	/** 
	 *  When tracing to find surfaces, what is the minimum size sphere radius we should use.
	 */
	UPROPERTY(EditAnywhere, Config, Category="Collision", meta=(UIMin=0, ClampMin=0))
	float SurfaceCollisionSphereMinRadius;
	
	/** 
	 *  When tracing to find surfaces, what is the maximum size sphere radius we should use.
	 */
	UPROPERTY(EditAnywhere, Config, Category="Collision", meta=(UIMin=0, ClampMin=0))
	float SurfaceCollisionSphereMaxRadius;
	
	/** 
	 *  When doing subdivisions, what distance should be used.
	 */
	UPROPERTY(EditAnywhere, Config, Category="Path", meta=(UIMin=0, ClampMin=0))
	float MinDistanceBetweenSubdivisions;
	
	/**
	 * Used more for security reasons in case of edge cases where infinite loops can happen.
	 */
	UPROPERTY(EditAnywhere, Config, Category="Path", meta=(UIMin=0, ClampMin=0))
	int32 MaxFillGapsLoopCount;

	/**  */
	UPROPERTY(EditAnywhere, Config, Category="Path", meta=(UIMin=0, ClampMin=0))
	float ShortFilteringMaxHeightDiff;
	
	/** 
	 *  If surface points are close enough they are removed.
	 *  See also CleanUpPathPointNormalThreshold.
	 */
	UPROPERTY(EditAnywhere, Config, Category="Path", meta=(UIMin=0, ClampMin=0))
	float CleanUpPathPointDistanceThreshold;
	
	/**
	 * If surface points normal are to similar they are removed.
	 * Between 0 and 1. 1 Means that the normals perfectly matches.
	 * 
	 * See also CleanUpPathPointDistanceThreshold.
	 */
	UPROPERTY(EditAnywhere, Config, Category="Path", meta=(UIMin=0, ClampMin=0, UIMax=1, ClampMax=1))
	float CleanUpPathPointNormalThreshold;
	
	
	virtual FName GetCategoryName() const override;
};
