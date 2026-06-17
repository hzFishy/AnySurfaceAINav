// By hzFishy - 2026 - Do whatever you want with it.


#include "Data/SANAnySurfaceNavSettings.h"


USANAnySurfaceNavSettings::USANAnySurfaceNavSettings():
	OverlapSurfaceCollisionProfile("OverlapAllDynamic"),
	BlockSurfaceCollisionProfile("BlockAllDynamic"),
	SurfaceCollisionSphereMinRadius(200),
	SurfaceCollisionSphereMaxRadius(2000),
	MinDistanceBetweenSubdivisions(150),
	MaxFillGapsLoopCount(5),
	ShortFilteringMaxHeightDiff(10),
	CleanUpPathPointDistanceThreshold(50),
	CleanUpPathPointNormalThreshold(0.8)
{}

const USANAnySurfaceNavSettings* USANAnySurfaceNavSettings::Get()
{
	return GetDefault<USANAnySurfaceNavSettings>();
}

FName USANAnySurfaceNavSettings::GetCategoryName() const
{
	return "Plugins";
}
