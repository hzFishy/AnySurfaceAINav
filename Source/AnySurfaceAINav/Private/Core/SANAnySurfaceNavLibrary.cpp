// By hzFishy - 2026 - Do whatever you want with it.

#include "Core/SANAnySurfaceNavLibrary.h"
#include "CPathVolume.h"
#include "MathUtil.h"
#include "Data/SANAnySurfaceNavSettings.h"
#include "Data/SANCore.h"
#include "Kismet/GameplayStatics.h"
#if SAN_WITH_DEBUG
#include "Draw/FUDraw.h"
#endif


namespace SAN::Library
{
#if SAN_WITH_DEBUG
	namespace Debug
	{
		FU_CMD_AUTOVAR(DebugDisplayFindAnySurfacePathCmd, 
			"SAN.Debug.DisplayFindAnySurfacePath", "Show debug data, increase number for more details",
			int32, DebugDisplayFindAnySurfacePath, 0
		);
		
		FU_CMD_AUTOVAR(DebugDisplayFindAnySurfacePathTimeCmd, 
			"SAN.Debug.DisplayFindAnySurfacePathTime", "", 
			float, DebugDisplayFindAnySurfacePathTime, 10
		);
	}
#endif
}

int32 USANAnySurfaceNavLibrary::FillGapsLoopCount = 0;

bool USANAnySurfaceNavLibrary::FindAnySurfacePathSync(const FSANFindPathRequest& Request, FSANFindPathResult& Result)
{
	if (!Request.IsValid())
	{
		return false;
	}
	
	UWorld* World = GEngine->GetWorldFromContextObject(Request.WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
	if (!World)
	{
		return false;
	}
	
	auto* CPathVolume = Cast<ACPathVolume>(UGameplayStatics::GetActorOfClass(World, ACPathVolume::StaticClass()));
	FCPathResult CPathResult = CPathVolume->FindPathSynchronous(Request.StartLocation, Request.EndLocation,  0, 0, 2);
	
	for (auto& CPathNode : CPathResult.UserPath)
	{
		Result.NavPathPoints.Emplace(CPathNode.WorldLocation);
	}
	
	if (Result.IsEmpty())
	{
		return false;
	}
	
#if SAN_WITH_DEBUG
	if (SAN::Library::Debug::DebugDisplayFindAnySurfacePath > 1)
	{
		for (auto& PathPoint : Result.NavPathPoints)
		{
			FU::Draw::DrawDebugSphere(
				World,
				PathPoint.Location,
				20,
				FColor::Green,
				SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
			);
		}
	}
#endif
	
	const auto* AnySurfaceNavSettings = USANAnySurfaceNavSettings::Get();
	
	// from each point, get closest surface
	TArray<FSANSurfaceHitResult> RawSurfaceHitResults;
	RawSurfaceHitResults.Reserve(Result.NavPathPoints.Num() * 5);
	
	FSANSurfaceHitResult PreviousSurface;
	// iterate all points
	for (int32 NavPointIndx = 0; NavPointIndx < Result.NavPathPoints.Num(); ++NavPointIndx)
	{
		const auto& CurrentPoint = Result.NavPathPoints[NavPointIndx];
		
		// current point
		{
			FSANSurfaceHitResult BestSurfaceHit;
			if (GetBestSurface(World, AnySurfaceNavSettings, CurrentPoint.Location, PreviousSurface, BestSurfaceHit))
			{
				RawSurfaceHitResults.Emplace(BestSurfaceHit);
			}
			PreviousSurface = BestSurfaceHit;
		}
		// skip the last
		if (NavPointIndx == Result.NavPathPoints.Num() - 1)
		{
			break;
		}
		
		// subdivions
		const auto& NextPoint = Result.NavPathPoints[NavPointIndx + 1];
		
		const float DistanceBetweenPoints = FVector::Dist(CurrentPoint.Location, NextPoint.Location);
		const int32 TotalNbOfSubdivisions = FMath::Floor(DistanceBetweenPoints / AnySurfaceNavSettings->MinDistanceBetweenSubdivisions);
		// we skip the "last" point in the subdivision since it will be the next nav point
		const int32 NbOfSubdivisions = TotalNbOfSubdivisions - 1;
		
		if (NbOfSubdivisions > 0)
		{
			for (int32 SbdvIndx = 1; SbdvIndx < NbOfSubdivisions + 1; ++SbdvIndx)
			{
				const FVector SbdvLocation = FMath::Lerp(CurrentPoint.Location, NextPoint.Location, (float)SbdvIndx/TotalNbOfSubdivisions);
			
				FSANSurfaceHitResult BestSurfaceHit;
				if (GetBestSurface(World, AnySurfaceNavSettings, SbdvLocation, PreviousSurface, BestSurfaceHit))
				{
					RawSurfaceHitResults.Emplace(BestSurfaceHit);
				}
				PreviousSurface = BestSurfaceHit;
			}
		}
	}
	
	// from generated points find short path
	TArray<FSANSurfaceHitResult> RawShortSurfaceHitResults;
	KeepShortestDistancePoints(World, AnySurfaceNavSettings, RawSurfaceHitResults, RawShortSurfaceHitResults);
	
	// try to smooth the path if there is a big distance between them
	FillGapsLoopCount = 0;
	const bool bFilledGaps = FillGaps(World, AnySurfaceNavSettings, PreviousSurface, RawShortSurfaceHitResults);
	
	// remove points that are to close with similar normal
	int32 ComparedSourceSurfaceIndex = 0;
	int32 ComparedNextSourceSurfaceIndex = 1;
	
	while (ComparedSourceSurfaceIndex < RawShortSurfaceHitResults.Num() - 1)
	{
		const auto& RawSurfaceHit = RawShortSurfaceHitResults[ComparedSourceSurfaceIndex];
		const auto& NextRawSurfaceHit = RawShortSurfaceHitResults[ComparedNextSourceSurfaceIndex];
		
		// check distance
		const float DistanceToNextPoint = FVector::Dist(RawSurfaceHit.HitLocation, NextRawSurfaceHit.HitLocation);
		if (DistanceToNextPoint < AnySurfaceNavSettings->CleanUpPathPointDistanceThreshold)
		{
			const float NormalDiff = FVector::DotProduct(RawSurfaceHit.HitNormal, NextRawSurfaceHit.HitNormal);
			if (NormalDiff >= AnySurfaceNavSettings->CleanUpPathPointNormalThreshold)
			{
#if SAN_WITH_DEBUG
				if (SAN::Library::Debug::DebugDisplayFindAnySurfacePath > 1)
				{
					FU::Draw::DrawDebugSphere(
						World,
						NextRawSurfaceHit.HitLocation,
						20,
						FColor::Red,
						SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
					);
				}
#endif
				RawShortSurfaceHitResults.RemoveAt(ComparedNextSourceSurfaceIndex);
				
				// keep same source point, go next
				ComparedNextSourceSurfaceIndex++;
				continue;
			}
		}
		
		// we keep the next point, make it source
		ComparedSourceSurfaceIndex++;
		ComparedNextSourceSurfaceIndex = ComparedSourceSurfaceIndex + 1;
	}
	
	// save results
	Result.SurfaceHitResults = RawShortSurfaceHitResults;
	
#if SAN_WITH_DEBUG
	if (SAN::Library::Debug::DebugDisplayFindAnySurfacePath)
	{
		for (int32 i = 0; i < Result.SurfaceHitResults.Num(); ++i)
		{
			const auto& HitResult = Result.SurfaceHitResults[i];
			
			FU::Draw::DrawDebugString(
				World,
				HitResult.HitLocation + FVector(0, 0, 20),
				FString::Printf(TEXT("%i"), i),
				FColor::Orange,
				SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
			);
			
			FU::Draw::DrawDebugSphere(
				World,
				HitResult.HitLocation,
				10,
				FColor::Magenta,
				SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
			);
		
			FU::Draw::DrawDebugDirectionalArrow(
				World,
				HitResult.HitLocation,
				HitResult.HitNormal * 100,
				FColor::Magenta,
				SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
			);
		}
	}
#endif
	
	return !Result.NavPathPoints.IsEmpty() && !Result.SurfaceHitResults.IsEmpty();
}

bool USANAnySurfaceNavLibrary::IsPathResultEmpty(const FSANFindPathResult& PathResult)
{
	return PathResult.IsEmpty();
}

bool USANAnySurfaceNavLibrary::GetBestSurface(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FVector PointLocation, FSANSurfaceHitResult PreviousSurface, FSANSurfaceHitResult& OutBestSurface)
{
	TArray<FHitResult> HitResults;
	// trace until we find a surface
	// TODO: from previous used radius use that as base to avoid repititive traces to find similar radius
	const bool bFoundSurface = GetBestSurfaceInternal(World, Settings, PointLocation, Settings->SurfaceCollisionSphereMinRadius, HitResults);
	
	if (bFoundSurface && !HitResults.IsEmpty())
	{
		if (PreviousSurface.IsValid())
		{
			// remove hits blocked by geometry
			for (auto HitIt = HitResults.CreateIterator(); HitIt; ++HitIt)
			{
				FHitResult BlockHitResult;
				const bool bHit = World->LineTraceSingleByProfile(
					BlockHitResult,
					PointLocation,
					(*HitIt).ImpactPoint,
					Settings->BlockSurfaceCollisionProfile.Name
				);
				
				// remove if we hit something else than the destination comp
				if (bHit && (*HitIt).GetComponent() != BlockHitResult.GetComponent())
				{
#if SAN_WITH_DEBUG
					if (SAN::Library::Debug::DebugDisplayFindAnySurfacePath > 1)
					{
						FU::Draw::DrawDebugSphere(
							World,
							(*HitIt).ImpactPoint,
							5,
							FColor::Red,
							SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
						);
					}
#endif
					HitIt.RemoveCurrent();
				}
			}
			
			// take closest
			if (HitResults.Num() > 1)
			{
				int32 ClosestIndex = -1;
				float ClosestDistance = UE_MAX_FLT;

				for (int32 HitIndex = 0; HitIndex < HitResults.Num(); ++HitIndex)
				{
					const auto& HitResult = HitResults[HitIndex];
					const float Distance = FVector::Dist(HitResult.ImpactPoint, PreviousSurface.HitLocation);
				
					if (Distance > 0 && Distance < ClosestDistance)
					{
						ClosestIndex = HitIndex;
						ClosestDistance = Distance;
					}
				}
			
				OutBestSurface.HitLocation = HitResults[ClosestIndex].ImpactPoint;
				OutBestSurface.HitNormal = HitResults[ClosestIndex].ImpactNormal;
			}
			else
			{
				OutBestSurface.HitLocation = HitResults[0].ImpactPoint;
				OutBestSurface.HitNormal = HitResults[0].ImpactNormal;
			}
		}
		else
		{
			OutBestSurface.HitLocation = HitResults[0].ImpactPoint;
			OutBestSurface.HitNormal = HitResults[0].ImpactNormal;
		}
	}
	
#if SAN_WITH_DEBUG
	if (SAN::Library::Debug::DebugDisplayFindAnySurfacePath > 1)
	{
		for (auto& HitResult : HitResults)
		{
			FU::Draw::DrawDebugDirectionalArrow(
				World,
				HitResult.ImpactPoint,
				HitResult.ImpactNormal * 120,
				FColor::Cyan,
				SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
			);
		}
	}
#endif
	
	return !HitResults.IsEmpty();
}

bool USANAnySurfaceNavLibrary::GetBestSurfaceInternal(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FVector PointLocation, float Radius, TArray<FHitResult>& HitResults)
{
	World->SweepMultiByProfile(
		HitResults,
		PointLocation,
		PointLocation,
		FQuat::Identity,
		Settings->OverlapSurfaceCollisionProfile.Name,
		FCollisionShape::MakeSphere(Radius)
	);
	
	if (HitResults.IsEmpty())
	{
		// calc a bigger radius since we found nothing
		const float NewRadius = Radius * 1.2;
		
		if (NewRadius > Settings->SurfaceCollisionSphereMaxRadius)
		{
			// exceeded max radius
			return false;
		}
		
		return GetBestSurfaceInternal(World, Settings, PointLocation, NewRadius, HitResults);
	}
	else
	{
#if SAN_WITH_DEBUG
		if (SAN::Library::Debug::DebugDisplayFindAnySurfacePath > 2)
		{
			FU::Draw::DrawDebugSphere(
				World,
				PointLocation,
				5,
				FColor::Orange,
				SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
			);
			
			FU::Draw::DrawDebugSphere(
				World,
				PointLocation,
				Radius,
				FColor::Orange,
				SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
			);
		}
#endif
	
		return true;
	}
}

void USANAnySurfaceNavLibrary::KeepShortestDistancePoints(UWorld* World, const USANAnySurfaceNavSettings* Settings, const TArray<FSANSurfaceHitResult>& InRawSurfaceHits, TArray<FSANSurfaceHitResult>& OutFilteredRawSurfaceHits)
{
	OutFilteredRawSurfaceHits.Reserve(InRawSurfaceHits.Num());
	
	TArray<int32> RemovedIndices;
	RemovedIndices.Reserve(InRawSurfaceHits.Num());
	
	// get source point (n)
	// get n+1 and n+2
	// see if dist(n, n+2) is shorter then dist(n, n+1) + dist(n+1, n+2)
	// if yes, remove n+1
	// also check collisions if distance shorter (since distance math is faster than checking scene collisions)
	
	int32 SourceIndex = 0;
	int32 NIndex = 1;
	
	// iterate all points except the last one
	while (SourceIndex < (InRawSurfaceHits.Num() - 1))
	{
		const FSANSurfaceHitResult& SourceRawSurfaceHit = InRawSurfaceHits[SourceIndex];
		const FSANSurfaceHitResult& N1RawSurfaceHit = InRawSurfaceHits[NIndex];
		
		// if we are close to the end there will be no n+2 point
		if (NIndex + 1 < InRawSurfaceHits.Num())
		{
			const FSANSurfaceHitResult& N2RawSurfaceHit = InRawSurfaceHits[NIndex + 1];
			
			if (FMathf::Abs(SourceRawSurfaceHit.HitLocation.Z - N1RawSurfaceHit.HitLocation.Z) > Settings->ShortFilteringMaxHeightDiff)
			{
				SourceIndex += 1;
				NIndex = SourceIndex + 1;
				continue;
			}
			
			// dist(n, n+1) + dist(n+1, n+2)
			const float DefaultDistance = FVector::Dist(SourceRawSurfaceHit.HitLocation, N1RawSurfaceHit.HitLocation) + FVector::Dist(N1RawSurfaceHit.HitLocation, N2RawSurfaceHit.HitLocation);
			
			// dist(n, n+2)
			const float N2Distance = FVector::Dist(SourceRawSurfaceHit.HitLocation, N2RawSurfaceHit.HitLocation);
			
			if (N2Distance < DefaultDistance)
			{
				// TODO: test for collisions in between
				
				// remove n+1
				RemovedIndices.Emplace(NIndex);
				
#if SAN_WITH_DEBUG
				if (SAN::Library::Debug::DebugDisplayFindAnySurfacePath > 1)
				{
					FU::Draw::DrawDebugSphere(
						World,
						InRawSurfaceHits[NIndex].HitLocation,
						20,
						FColor::Red,
						SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
					);
				}
#endif
				
				// go forward
				NIndex += 1;
			}
			else
			{
				// go forward
				SourceIndex += 1;
				NIndex = SourceIndex + 1;
			}
		}
		else
		{
			break;
		}
	}
	
	// TODO: directly add points in while loop instead to avoir a second loop
	for (int32 InSurfaceHitsIndex = 0; InSurfaceHitsIndex < InRawSurfaceHits.Num(); ++InSurfaceHitsIndex)
	{
		if (!RemovedIndices.Contains(InSurfaceHitsIndex))
		{
			OutFilteredRawSurfaceHits.Emplace(InRawSurfaceHits[InSurfaceHitsIndex]);
		}
	}
}

bool USANAnySurfaceNavLibrary::FillGaps(UWorld* World, const USANAnySurfaceNavSettings* Settings, FSANSurfaceHitResult PreviousSurface, TArray<FSANSurfaceHitResult>& RawSurfaceHits)
{
	FillGapsLoopCount++;
	
	if (FillGapsLoopCount > Settings->MaxFillGapsLoopCount)
	{
		return true;
	}
	
	constexpr float MaxDistanceBetweenPoints = 300;
	
	bool bDidChanged = false;
	
	for (int32 RawSurfaceHitIndex = 0; RawSurfaceHitIndex < RawSurfaceHits.Num() - 1; ++RawSurfaceHitIndex)
	{
		const auto& RawSurfaceHit = RawSurfaceHits[RawSurfaceHitIndex];
		// here we do a copy since below we might edit the index which will change where the ref would point
		FSANSurfaceHitResult NextRawSurfaceHit = RawSurfaceHits[RawSurfaceHitIndex + 1];
		
		const float DistanceToNextPoint = FVector::Dist(RawSurfaceHit.HitLocation, NextRawSurfaceHit.HitLocation);
		
		if (DistanceToNextPoint > MaxDistanceBetweenPoints)
		{
#if SAN_WITH_DEBUG
			if (SAN::Library::Debug::DebugDisplayFindAnySurfacePath > 1)
			{
				FU::Draw::DrawDebugLine(World,
				   RawSurfaceHit.HitLocation,
				   NextRawSurfaceHit.HitLocation,
				   FColor::White,
				   SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
			   );
			}
#endif
			
			bDidChanged = true;
			
			// make subdivisions
			const int32 TotalNbOfSubdivisions = FMath::Floor(DistanceToNextPoint / Settings->MinDistanceBetweenSubdivisions);
			// we skip the "last" point in the subdivision since it will be the next nav point
			const int32 NbOfSubdivisions = TotalNbOfSubdivisions - 1;
			
			for (int32 SbdvIndx = 1; SbdvIndx < NbOfSubdivisions + 1; ++SbdvIndx)
			{
				const FVector SbdvLocation = FMath::Lerp(RawSurfaceHit.HitLocation, NextRawSurfaceHit.HitLocation, (float)SbdvIndx/TotalNbOfSubdivisions);
				
				FSANSurfaceHitResult BestSurfaceHit;
				if (GetBestSurface(World, Settings, SbdvLocation, PreviousSurface, BestSurfaceHit))
				{
					// inject at the correct index
					RawSurfaceHits.EmplaceAt(RawSurfaceHitIndex + 1, BestSurfaceHit);
					RawSurfaceHitIndex++;
				}
				PreviousSurface = BestSurfaceHit;
			}
		}
	}
	
	// if we didnt change anything we can stop here
	if (bDidChanged)
	{
		return FillGaps(World, Settings, PreviousSurface, RawSurfaceHits);
	}
	else
	{
		return true;
	}
}
