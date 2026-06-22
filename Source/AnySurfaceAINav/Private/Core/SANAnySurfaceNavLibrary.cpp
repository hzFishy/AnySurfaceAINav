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
		
		FU_CMD_AUTOVAR(DebugDisableFillGapsCmd, 
			"SAN.Debug.DisableFillGaps", "Set to 1 to debug the pathfinding without filling the gaps. Set to 0 to go back to the default behavior.",
			int32, DebugDisableFillGaps, 0
		);
		
		FU_CMD_AUTOVAR(DebugDisableShortPathFilteringCmd, 
			"SAN.Debug.DisableShortPathFiltering", "Set to 1 to debug the pathfinding without filtering point to keep the shortest distances. Set to 0 to go back to the default behavior.",
			int32, DebugDisableShortPathFiltering, 0
		);
		
		FU_CMD_AUTOVAR(DebugDisableDistanceNormalFilteringCmd, 
			"SAN.Debug.DisableDistanceNormalFiltering", "Set to 1 to debug the pathfinding without filtering points based on distances and normals. Set to 0 to go back to the default behavior.",
			int32, DebugDisableDistanceNormalFiltering, 0
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
	
	// TODO: rework, cache it ? or add it to the request params (as optional?)
	auto* CPathVolume = Cast<ACPathVolume>(UGameplayStatics::GetActorOfClass(World, ACPathVolume::StaticClass()));
	
	FCPathResult CPathResult = CPathVolume->FindPathSynchronous(Request.StartLocation, Request.EndLocation,  0, 0, 2);
	
	for (auto& CPathNode : CPathResult.UserPath)
	{
		Result.NavPathPoints.Emplace(CPathNode.WorldLocation);
	}
	
	// if the 3D nav pathfinding gave no points we abort
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
	
	// from each point, get closest surface(s) to build a surface "path"
	TArray<FSANSurfaceHitResult> RawSurfaceHitResults;
	RawSurfaceHitResults.Reserve(Result.NavPathPoints.Num() * 5);
	
	
	// iterate all points
	TArray<FSANSurfaceHitResult> PreviousSurfaces;
	for (int32 NavPointIndx = 0; NavPointIndx < Result.NavPathPoints.Num(); ++NavPointIndx)
	{
		const auto& CurrentPoint = Result.NavPathPoints[NavPointIndx];
		
		// current point
		{
			TArray<FSANSurfaceHitResult> BestSurfaceHits;
			if (GetBestSurface(World, AnySurfaceNavSettings, CurrentPoint.Location, PreviousSurfaces, BestSurfaceHits))
			{
				for (const auto& BestSurfaceHit : BestSurfaceHits)
				{
					RawSurfaceHitResults.Emplace(BestSurfaceHit);
				}
			}
			
			// override any previous surfaces
			PreviousSurfaces = BestSurfaceHits;
		}
		
		// skip the last
		if (NavPointIndx == Result.NavPathPoints.Num() - 1)
		{
			break;
		}
		
		// see if we have to do subdivions if nav points are far enough from each other
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
			
				TArray<FSANSurfaceHitResult> BestSurfaceHits;
				if (GetBestSurface(World, AnySurfaceNavSettings, SbdvLocation, PreviousSurfaces, BestSurfaceHits))
				{
					for (const auto& BestSurfaceHit : BestSurfaceHits)
					{
						RawSurfaceHitResults.Emplace(BestSurfaceHit);
					}
				}
				
				// override any previous surfaces
				PreviousSurfaces = BestSurfaceHits;
			}
		}
	}
	
	// clear before further use
	PreviousSurfaces.Empty();
	
	
	// from generated points find shortest path
	TArray<FSANSurfaceHitResult> RawShortSurfaceHitResults;
#if SAN_WITH_DEBUG
	if (SAN::Library::Debug::DebugDisableShortPathFiltering == 0)
	{
		KeepShortestDistancePoints(World, AnySurfaceNavSettings, RawSurfaceHitResults, RawShortSurfaceHitResults);
	}
	else
	{
		// since we are not calling KeepShortestDistancePoints we have to copy over the results like nothing was removed
		RawShortSurfaceHitResults = RawSurfaceHitResults;
	}
#else 
	KeepShortestDistancePoints(World, AnySurfaceNavSettings, RawSurfaceHitResults, RawShortSurfaceHitResults);
#endif
	
	
	// try to clean the path if there is a big distance between found surfaces points
	// reset vars
	FillGapsLoopCount = 0;
	
#if SAN_WITH_DEBUG
	if (SAN::Library::Debug::DebugDisableFillGaps == 0)
	{
		const bool bFilledGaps = FillGaps(World, AnySurfaceNavSettings, PreviousSurfaces, RawShortSurfaceHitResults);
	}
#else 
	const bool bFilledGaps = FillGaps(World, AnySurfaceNavSettings, PreviousSurfaces, RawShortSurfaceHitResults);
#endif
	
	
	// remove points that are to close with similar normal
	int32 ComparedSourceSurfaceIndex = 0;
	int32 ComparedNextSourceSurfaceIndex = 1;
	while (ComparedSourceSurfaceIndex < RawShortSurfaceHitResults.Num() - 1)
	{
#if SAN_WITH_DEBUG
		if (SAN::Library::Debug::DebugDisableDistanceNormalFiltering > 0)
		{
			break;
		}
#endif
		
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

bool USANAnySurfaceNavLibrary::GetBestSurface(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FVector PointLocation, const TArray<FSANSurfaceHitResult>& PreviousSurfaces, TArray<FSANSurfaceHitResult>& OutBestSurfaces)
{
	// there is always a result
	OutBestSurfaces.AddDefaulted(1);
	
	TArray<FHitResult> HitResults;
	// trace until we find a surface
	// TODO: from previous used radius use that as base to avoid repititive traces to find similar radius
	const bool bFoundSurface = GetBestSurfaceInternal(World, Settings, PointLocation, Settings->SurfaceCollisionSphereMinRadius, HitResults);
	
	if (bFoundSurface && !HitResults.IsEmpty())
	{
		if (!PreviousSurfaces.IsEmpty())
		{
			if (HitResults.Num() > 1)
			{
				// remove hits blocked by geometry (if there is a blocking primitive between the nav point and the found surfaces)
				for (auto HitIt = HitResults.CreateIterator(); HitIt; ++HitIt)
				{
					FHitResult BlockHitResult;
					const bool bHit = World->LineTraceSingleByProfile(
						BlockHitResult,
						PointLocation,
						(*HitIt).ImpactPoint,
						Settings->BlockSurfaceCollisionProfile.Name
					);
					
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
				
				// get the ideal previous surface, for now its the closest surface
				float BestPreviousSurfaceDistance = TNumericLimits<float>::Max();
				int32 BestPreviousSurfaceIndex = -1;
				
				for (int i = 0; i < PreviousSurfaces.Num(); ++i)
				{
					const float Distance = FVector::Dist(PreviousSurfaces[i].HitLocation, PointLocation);
					if (Distance < BestPreviousSurfaceDistance)
					{
						BestPreviousSurfaceDistance = Distance;
						BestPreviousSurfaceIndex = i;
					}
				}
				
				const auto& BestPreviousSurface = PreviousSurfaces[BestPreviousSurfaceIndex];
				
				// take the closest hit location to PointLocation, while having the closest normal
				
				// TODO: move to settings
				constexpr float CloseDistanceScorePerCm = 1000;
				constexpr float SimilarNormalScorePerDot = 10;
				constexpr float DotNormalDiffMaxThreshold = 0.2;
				
				TArray<float> HitResultsScores;
				HitResultsScores.Reserve(HitResults.Num());
				
				// calc score with distances
				for (int32 HitIndex = 0; HitIndex < HitResults.Num(); ++HitIndex)
				{
					const auto& HitResult = HitResults[HitIndex];
					const float Distance = FVector::Dist(HitResult.ImpactPoint, BestPreviousSurface.HitLocation);
				
					// store score, the closest distance will have the highest score
					HitResultsScores.Emplace(CloseDistanceScorePerCm / Distance);
				}
				
				TArray<int32> ExtraDiffSurfaces;
				ExtraDiffSurfaces.Reserve(HitResults.Num() / 2);
				
				// calc score with normals
				for (int32 HitIndex = 0; HitIndex < HitResults.Num(); ++HitIndex)
				{
					// store score, the more the normals match the highest the score
					const auto& HitResult = HitResults[HitIndex];
					const float Dot = FVector::DotProduct(HitResult.ImpactNormal, BestPreviousSurface.HitNormal);
					HitResultsScores[HitIndex] += Dot * SimilarNormalScorePerDot;
					
					// if normals are to different we will append the surface
					if (Dot <= DotNormalDiffMaxThreshold)
					{
						ExtraDiffSurfaces.Emplace(HitIndex);
					}
				}
				
				// get best score
				int32 BestScoreIndex = 0;
				float BestScore = 0;
				for (int32 i = 1; i < HitResults.Num(); ++i)
				{
					if (HitResultsScores[i] > BestScore && !ExtraDiffSurfaces.Contains(i))
					{
						BestScore = HitResultsScores[i];
						BestScoreIndex = i;
					}
				}
				
				OutBestSurfaces[0] = FSANSurfaceHitResult(HitResults[BestScoreIndex]);
				
				// TODO: if normals are to different, two points can be taken
				for (int32 i = 0; i < ExtraDiffSurfaces.Num(); ++i)
				{
					OutBestSurfaces.Emplace(HitResults[ExtraDiffSurfaces[i]]);
				}
				
				// sort the out surfaces so we order from closest to furthest from the best previous location
				
				// TODO: debug sort
				OutBestSurfaces.Sort([BestPreviousSurface] (const FSANSurfaceHitResult& A, const FSANSurfaceHitResult& B)
				{
					return FVector::Dist(A.HitLocation, BestPreviousSurface.HitLocation) < FVector::Dist(B.HitLocation, BestPreviousSurface.HitLocation);
				});
			}
			else
			{
				OutBestSurfaces[0] = FSANSurfaceHitResult(HitResults[0]);
			}
		}
		else
		{
			// TODO: might have to still run the "multiple out surfaces" code even if we are the first point
			OutBestSurfaces[0] = FSANSurfaceHitResult(HitResults[0]);
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
			
			if (FMathf::Abs(SourceRawSurfaceHit.HitLocation.Z - N1RawSurfaceHit.HitLocation.Z) > Settings->ShortFilteringMaxHeightDiff
				|| FMathf::Abs(SourceRawSurfaceHit.HitLocation.Z - N2RawSurfaceHit.HitLocation.Z) > Settings->ShortFilteringMaxHeightDiff)
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

bool USANAnySurfaceNavLibrary::FillGaps(UWorld* World, const USANAnySurfaceNavSettings* Settings, TArray<FSANSurfaceHitResult>& PreviousSurfaces, TArray<FSANSurfaceHitResult>& RawSurfaceHits)
{
	FillGapsLoopCount++;
	
	if (FillGapsLoopCount > Settings->MaxFillGapsLoopCount)
	{
		return true;
	}
	
	
	bool bDidChanged = false;
	
	// iterate all points and see if we can find gaps (starting from the end of the path)
	for (int32 RawSurfaceHitIndex = 0; RawSurfaceHitIndex < RawSurfaceHits.Num() - 1; ++RawSurfaceHitIndex)
	{
		const auto& RawSurfaceHit = RawSurfaceHits[RawSurfaceHitIndex];
		// here we do a copy since below we might edit the index which will change where the ref would point
		FSANSurfaceHitResult NextRawSurfaceHit = RawSurfaceHits[RawSurfaceHitIndex + 1];
		
		const float DistanceToNextPoint = FVector::Dist(RawSurfaceHit.HitLocation, NextRawSurfaceHit.HitLocation);
		
		if (DistanceToNextPoint > Settings->GabsMaxDistanceBetweenPoints)
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
			const int32 TotalNbOfSubdivisions = FMath::Floor(DistanceToNextPoint / Settings->GabsMinDistanceBetweenSubdivisions);
			// we skip the "last" point in the subdivision since it will be the next nav point
			const int32 NbOfSubdivisions = TotalNbOfSubdivisions - 1;
			
			for (int32 SbdvIndx = 1; SbdvIndx < NbOfSubdivisions + 1; ++SbdvIndx)
			{
				const FVector SbdvLocation = FMath::Lerp(RawSurfaceHit.HitLocation, NextRawSurfaceHit.HitLocation, (float)SbdvIndx/TotalNbOfSubdivisions);
				
				TArray<FSANSurfaceHitResult> BestSurfaceHits;
				if (GetBestSurface(World, Settings, SbdvLocation, PreviousSurfaces, BestSurfaceHits))
				{
					// inject filling surfaces at the correct indices
					for (int i = 0; i < BestSurfaceHits.Num(); ++i)
					{
						RawSurfaceHits.EmplaceAt(RawSurfaceHitIndex + 1, BestSurfaceHits[i]);
						RawSurfaceHitIndex++;
					}
				}
				
				// override any past surfaces
				PreviousSurfaces = BestSurfaceHits;
			}
		}
	}
	
	// if we didnt change anything we can stop here
	if (bDidChanged)
	{
		return FillGaps(World, Settings, PreviousSurfaces, RawSurfaceHits);
	}
	else
	{
		return true;
	}
}
