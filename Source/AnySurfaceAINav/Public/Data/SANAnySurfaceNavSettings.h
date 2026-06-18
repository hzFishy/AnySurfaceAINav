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
	UPROPERTY(EditAnywhere, Config, Category="Path|Collision")
	FCollisionProfileName BlockSurfaceCollisionProfile;

	/** 
	 *  When tracing to find surfaces, what is the minimum size sphere radius we should use.
	 */
	UPROPERTY(EditAnywhere, Config, Category="Path|Collision", meta=(UIMin=0, ClampMin=0))
	float SurfaceCollisionSphereMinRadius;
	
	/** 
	 *  When tracing to find surfaces, what is the maximum size sphere radius we should use.
	 */
	UPROPERTY(EditAnywhere, Config, Category="Path|Collision", meta=(UIMin=0, ClampMin=0))
	float SurfaceCollisionSphereMaxRadius;
	
	/** 
	 *  When doing subdivisions, what distance should be used.
	 */
	UPROPERTY(EditAnywhere, Config, Category="Path|Subdivisions", meta=(UIMin=0, ClampMin=0))
	float MinDistanceBetweenSubdivisions;
	
	/**
	 * If two points are far away we will fill the gab (recursif).
	 */
	UPROPERTY(EditAnywhere, Config, Category="Path|Gaps", meta=(UIMin=0, ClampMin=0))
	float GabsMaxDistanceBetweenPoints;
	
	/**
	 * When doing subdivisions from filed gabs, what distance should be used.
	 */
	UPROPERTY(EditAnywhere, Config, Category="Path|Gaps", meta=(UIMin=0, ClampMin=0))
	float GabsMinDistanceBetweenSubdivisions;
	
	/**
	 * Used more for security reasons in case of edge cases where infinite loops can happen.
	 */
	UPROPERTY(EditAnywhere, Config, Category="Path|Gaps", meta=(UIMin=0, ClampMin=0))
	int32 MaxFillGapsLoopCount;
	
	/** 
	 *  When doing distance based filtering to remove points, this is used as the maximum height difference there can be between two points.
	 */
	UPROPERTY(EditAnywhere, Config, Category="Path|Short", meta=(UIMin=0, ClampMin=0))
	float ShortFilteringMaxHeightDiff;
	
	/** 
	 *  If surface points are close enough only one will be kept.
	 *  
	 *  See also CleanUpPathPointNormalThreshold.
	 */
	UPROPERTY(EditAnywhere, Config, Category="Path|CleanUp", meta=(UIMin=0, ClampMin=0))
	float CleanUpPathPointDistanceThreshold;
	
	/**
	 * If surface points normal are to similar and close only one will be kept.
	 * Range is [0, 1]. 1 Means that the normals perfectly matches.
	 * 
	 * See also CleanUpPathPointDistanceThreshold.
	 */
	UPROPERTY(EditAnywhere, Config, Category="Path|CleanUp", meta=(UIMin=0, ClampMin=0, UIMax=1, ClampMax=1))
	float CleanUpPathPointNormalThreshold;
	
	
	virtual FName GetCategoryName() const override;
};
