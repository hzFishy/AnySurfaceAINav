// By hzFishy - 2026 - Do whatever you want with it.


#include "Data/SANSurfaceTypes.h"


FSANSurfaceHitResult::FSANSurfaceHitResult():
	HitLocation(FVector::ZeroVector),
	HitNormal(FVector::ZeroVector)
{}

FSANSurfaceHitResult::FSANSurfaceHitResult(const FHitResult& HitResult):
	HitLocation(HitResult.ImpactPoint),
	HitNormal(HitResult.ImpactNormal)
{}

FSANSurfaceHitResult::FSANSurfaceHitResult(const FVector& HitLocation, const FVector& HitNormal):
	HitLocation(HitLocation),
	HitNormal(HitNormal)
{}

bool FSANSurfaceHitResult::operator==(const FSANSurfaceHitResult& Other) const
{
	return HitLocation.Equals(Other.HitLocation, 1) && HitNormal.Equals(Other.HitNormal, 1);
}

bool FSANSurfaceHitResult::IsValid() const
{
	return !HitLocation.IsZero() && !HitNormal.IsZero();
}

void FSANSurfaceHitResult::Reset()
{
	HitLocation = FVector::ZeroVector;
	HitNormal = FVector::ZeroVector;
}

FSANFindPathRequest::FSANFindPathRequest(): 
	WorldContextObject(nullptr), 
	StartLocation(FVector::ZeroVector), 
	EndLocation(FVector::ZeroVector), 
	AgentRadius(-1),
	bTraceComplex(true)
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
	return SurfaceHitResults.IsEmpty();
}
