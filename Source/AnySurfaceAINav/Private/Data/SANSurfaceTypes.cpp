// By hzFishy - 2026 - Do whatever you want with it.


#include "Data/SANSurfaceTypes.h"


FSANSurfaceHitResult::FSANSurfaceHitResult() :
	HitLocation(FVector::ZeroVector),
	HitNormal(FVector::ZeroVector)
{}


bool FSANSurfaceHitResult::IsValid() const
{
	return !HitLocation.IsZero() && !HitNormal.IsZero();
}

FSANFindPathRequest::FSANFindPathRequest(): 
	WorldContextObject(nullptr), 
	StartLocation(FVector::ZeroVector), 
	EndLocation(FVector::ZeroVector), 
	AgentRadius(-1)
{}

bool FSANFindPathRequest::IsValid() const
{
	return ::IsValid(WorldContextObject) && AgentRadius > 0;
}


FSANFindPathResult::FSANFindPathResult()
{}

void FSANFindPathResult::SurfacesToPositions(TArray<FVector>& PositionsArray, float SurfaceDistance) const
{
	PositionsArray.Reserve(SurfaceHitResults.Num());

	for (auto& SurfaceHitResult : SurfaceHitResults)
	{
		PositionsArray.Emplace(SurfaceHitResult.HitLocation + SurfaceHitResult.HitNormal * SurfaceDistance);
	}
}

bool FSANFindPathResult::IsEmpty() const
{
	return NavPathPoints.IsEmpty() && SurfaceHitResults.IsEmpty();
}
