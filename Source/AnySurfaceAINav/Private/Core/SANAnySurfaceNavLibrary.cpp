// By hzFishy - 2026 - Do whatever you want with it.

#include "Core/SANAnySurfaceNavLibrary.h"
#include "CPathVolume.h"
#include "MathUtil.h"
#include "Data/SANAnySurfaceNavSettings.h"
#include "Core/SANCore.h"
#include "Kismet/GameplayStatics.h"
#include "Logging/FULogging.h"
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
	TRACE_CPUPROFILER_EVENT_SCOPE(SAN::GlobalFindPathSync)
	
	const FName VLogCategoryName_Raw = "SANFindAnySurfacePathSync_Raw";
	const FName VLogCategoryName_BasePass = "SANFindAnySurfacePathSync_BasePass";
	
	if (!Request.IsValid())
	{
		SAN_VLOG_Static_W(GWorld, "Find Path failed: request is invalid");
		return false;
	}
	
	UWorld* World = GEngine->GetWorldFromContextObject(Request.WorldContextObject, EGetWorldErrorMode::LogAndReturnNull);
	if (!World)
	{
		return false;
	}
	
	// TODO: rework, cache it ? or add it to the request params (as optional?)
	auto* CPathVolume = Cast<ACPathVolume>(UGameplayStatics::GetActorOfClass(World, ACPathVolume::StaticClass()));
	
	if (!IsValid(CPathVolume))
	{
		SAN_VLOG_Static_W(World, "Find Path failed: no CPathVolume actor");
		return false;
	}
	
	FCPathResult CPathResult;
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(SAN::CFindPathSync)
		// TODO: make settings in request for smoothing passes and time limit
		
		SAN_VLOG_Static_D(World, "Requesting a path synchronously to CPath plugin");
		CPathResult = CPathVolume->FindPathSynchronous(Request.StartLocation, Request.EndLocation,  0, 0, 2);
	}
	
	// if the 3D nav pathfinding gave no points we abort
	if (CPathResult.UserPath.IsEmpty())
	{
		SAN_VLOG_Static_W(World, "Find Path found no points")
		return false;
	}
	else
	{
		SAN_VLOG_Static_D(World, "CPath FindPath found %i points", CPathResult.UserPath.Num());
		for (int32 i = 0; i < CPathResult.UserPath.Num(); ++i)
		{
			auto& CPathNode = CPathResult.UserPath[i];
			Result.NavPathPoints.Emplace(CPathNode.WorldLocation);
			
			UE_VLOG_LOCATION(World, VLogCategoryName_Raw, Display, 
				CPathNode.WorldLocation, 15, 
				FColor::Magenta, TEXT("Raw Point [%i]"), i
			);
		
			if (i < CPathResult.UserPath.Num() - 1)
			{
				UE_VLOG_SEGMENT_THICK(World, VLogCategoryName_Raw, Display, 
					CPathNode.WorldLocation, CPathResult.UserPath[i + 1].WorldLocation, 
					FColor::Purple, 7, TEXT_EMPTY
				);
			}
		}
	}
	
#if SAN_WITH_DEBUG
	if (SAN::Library::Debug::DebugDisplayFindAnySurfacePath > 1)
	{
		for (auto& PathPoint : Result.NavPathPoints)
		{
			FU::Draw::Advanced::DrawDebugSphere(
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
	
	// from each point, get closest surface(s) to build a surface "path" (Base Pass)
	TArray<FSANSurfaceHitResult> PreviousSurfaces;
	TArray<FSANSurfaceHitResult> BasePassSurfacePoints;
	BasePassSurfacePoints.Reserve(Result.NavPathPoints.Num() * 5);
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(SAN::BasePass)
		for (int32 NavPointIndx = 0; NavPointIndx < Result.NavPathPoints.Num(); ++NavPointIndx)
		{
			const auto& CurrentPoint = Result.NavPathPoints[NavPointIndx];
			
			// current point
			{
				TArray<FSANSurfaceHitResult> BestSurfaceHits;
				float Radius = AnySurfaceNavSettings->SurfaceCollisionSphereMinRadius;;
				if (GetBestSurface(World, AnySurfaceNavSettings, CurrentPoint.Location, Radius, PreviousSurfaces, BestSurfaceHits))
				{
					UE_VLOG_WIRESPHERE(World, VLogCategoryName_BasePass, Display, 
						CurrentPoint.Location, Radius, FColor::Orange, TEXT("[%i]"), NavPointIndx
					);
					
					// TODO: fix incorrect order between points
					// the issue is that at a given index if many surfaces are found from a nav point some surfaces might not follow up at the given index
					// fix 1: get only first surface and store extra in a special array, iterate at the end the special array and fill in the points in-between
					
					for (int32 InnerSurfaceHitIndx = 0; InnerSurfaceHitIndx < BestSurfaceHits.Num(); ++InnerSurfaceHitIndx)
					{
						const auto& BestSurfaceHit = BestSurfaceHits[InnerSurfaceHitIndx];
						
						BasePassSurfacePoints.Emplace(BestSurfaceHit);
						UE_VLOG_LOCATION(World, VLogCategoryName_BasePass, Display, 
							BestSurfaceHit.HitLocation, 10, 
							FColor::Green, TEXT("Base Point [%i::%i], \n N: %s"), NavPointIndx, InnerSurfaceHitIndx, *FU::Utils::PrintCompactVector(BestSurfaceHit.HitNormal)
						);
						
						UE_VLOG_ARROW(World, VLogCategoryName_BasePass, Display, 
							BestSurfaceHit.HitLocation, BestSurfaceHit.HitLocation + BestSurfaceHit.HitNormal * 50, 
							FColor::Emerald, TEXT_EMPTY
						);
					}
				}
				
				// override any previous surfaces
				PreviousSurfaces = BestSurfaceHits;
			}
		}
	}
	// clear before further use
	PreviousSurfaces.Empty();
	
	
	// remove points that are to close with similar normal
	RemoveSimilarPoints(World, AnySurfaceNavSettings, BasePassSurfacePoints);
	
	
	// from generated points find shortest paths
	TArray<FSANSurfaceHitResult> BasePassShortenSurfacePoints;
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(SAN::ShortDistanceFiltering)
#if SAN_WITH_DEBUG
		if (SAN::Library::Debug::DebugDisableShortPathFiltering == 0)
		{
			KeepShortestDistancePoints(World, AnySurfaceNavSettings, BasePassSurfacePoints, BasePassShortenSurfacePoints);
		}
		else
		{
			// since we are not calling KeepShortestDistancePoints we have to copy over the results like nothing was removed
			BasePassShortenSurfacePoints = BasePassSurfacePoints;
		}
#else 
		KeepShortestDistancePoints(World, AnySurfaceNavSettings, BasePassSurfacePoints, BasePassShortenSurfacePoints);
#endif
	}
	
	// clear before further use
	PreviousSurfaces.Empty();
	
	
	// add subdivisions
	TArray<FSANSurfaceHitResult> BasePassSubdividedSurfacePoints;
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(SAN::BasePassSubdivision)
		
		FindAndInsertSubdivisions(World, AnySurfaceNavSettings, BasePassShortenSurfacePoints, BasePassSubdividedSurfacePoints);
	}
	
	// try to clean the path if there is a big distance between found surfaces points
	// reset vars
	FillGapsLoopCount = 0;
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(SAN::FillGapsFiltering)
#if SAN_WITH_DEBUG
		if (SAN::Library::Debug::DebugDisableFillGaps == 0)
		{
			const bool bFilledGaps = FillGaps(World, AnySurfaceNavSettings, PreviousSurfaces, BasePassSubdividedSurfacePoints);
		}
#else 
		const bool bFilledGaps = FillGaps(World, AnySurfaceNavSettings, PreviousSurfaces, BasePassSubdividedSurfacePoints);
#endif
	}
	
	
	// save results
	Result.SurfaceHitResults = BasePassSubdividedSurfacePoints;
	
#if SAN_WITH_DEBUG
	if (SAN::Library::Debug::DebugDisplayFindAnySurfacePath)
	{
		for (int32 i = 0; i < Result.SurfaceHitResults.Num(); ++i)
		{
			const auto& HitResult = Result.SurfaceHitResults[i];
			
			FU::Draw::Advanced::DrawDebugText(
				World,
				HitResult.HitLocation + FVector(0, 0, 20),
				FString::Printf(TEXT("%i"), i),
				FColor::Orange,
				SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime,
				2
			);
			
			FU::Draw::Advanced::DrawDebugSphere(
				World,
				HitResult.HitLocation,
				10,
				FColor::Magenta,
				SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
			);
		
			FU::Draw::Advanced::DrawDebugDirectionalArrow(
				World,
				HitResult.HitLocation,
				 HitResult.HitLocation + HitResult.HitNormal * 100,
				FColor::Magenta,
				SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
			);
		}
	}
#endif
	
	SAN_VLOG_Static_D(World, "Final resumts: kept %i surfaces", Result.SurfaceHitResults.Num());
	
	return !Result.IsEmpty();
}

bool USANAnySurfaceNavLibrary::IsPathResultEmpty(const FSANFindPathResult& PathResult)
{
	return PathResult.IsEmpty();
}

bool USANAnySurfaceNavLibrary::GetBestSurface(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FVector PointLocation, float& Radius, const TArray<FSANSurfaceHitResult>& PreviousSurfaces, TArray<FSANSurfaceHitResult>& OutBestSurfaces)
{
	// there is always a result
	OutBestSurfaces.AddDefaulted(1);
	
	TArray<FHitResult> HitResults;
	// trace until we find a surface
	// TODO: from previous used radius use that as base to avoid repititive traces to find similar radius
	const bool bFoundSurface = GetBestSurfaceInternal(World, Settings, PointLocation, Radius, HitResults);
	
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
							FU::Draw::Advanced::DrawDebugSphere(
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
				
				for (int32 PrevSurfaceIndx = 0; PrevSurfaceIndx < PreviousSurfaces.Num(); ++PrevSurfaceIndx)
				{
					const float Distance = FVector::Dist(PreviousSurfaces[PrevSurfaceIndx].HitLocation, PointLocation);
					if (Distance < BestPreviousSurfaceDistance)
					{
						BestPreviousSurfaceDistance = Distance;
						BestPreviousSurfaceIndex = PrevSurfaceIndx;
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
				// TODO: fix sorting, its incorrect
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
			FU::Draw::Advanced::DrawDebugDirectionalArrow(
				World,
				HitResult.ImpactPoint,
				HitResult.ImpactPoint + HitResult.ImpactNormal * 120,
				FColor::Cyan,
				SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
			);
		}
	}
#endif
	
	return !HitResults.IsEmpty();
}

bool USANAnySurfaceNavLibrary::GetBestSurfaceInternal(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FVector PointLocation, float& Radius, TArray<FHitResult>& HitResults)
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
		Radius *= (1 + Settings->SurfaceCollisionSphereRadiusGrowMultiplier);
		
		if (Radius > Settings->SurfaceCollisionSphereMaxRadius)
		{
			// exceeded max radius
			return false;
		}
		
		return GetBestSurfaceInternal(World, Settings, PointLocation, Radius, HitResults);
	}
	else
	{
#if SAN_WITH_DEBUG
		if (SAN::Library::Debug::DebugDisplayFindAnySurfacePath > 2)
		{
			FU::Draw::Advanced::DrawDebugSphere(
				World,
				PointLocation,
				5,
				HitResults.Num() > 1 ? FColor::Red : FColor::Orange,
				SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
			);
			
			FU::Draw::Advanced::DrawDebugSphere(
				World,
				PointLocation,
				Radius,
				HitResults.Num() > 1 ? FColor::Red : FColor::Orange,
				SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
			);
		}
#endif
		
		return true;
	}
}

void USANAnySurfaceNavLibrary::RemoveSimilarPoints(UWorld* World, const USANAnySurfaceNavSettings* Settings, TArray<FSANSurfaceHitResult>& SurfacePoints)
{
	TRACE_CPUPROFILER_EVENT_SCOPE(SAN::ShortDistanceFiltering)
	
	const FName VLogCategoryName_SimilarPointsFiltering = "SANFindAnySurfacePathSync_SimilarPointsFiltering";
	
	int32 ComparedSourceSurfaceIndex = 0;
	int32 ComparedNextSourceSurfaceIndex = 1;
	while (ComparedSourceSurfaceIndex < SurfacePoints.Num() - 1)
	{
#if SAN_WITH_DEBUG
		if (SAN::Library::Debug::DebugDisableDistanceNormalFiltering > 0)
		{
			break;
		}
#endif
		
		const auto& RawSurfaceHit = SurfacePoints[ComparedSourceSurfaceIndex];
		const auto& NextRawSurfaceHit = SurfacePoints[ComparedNextSourceSurfaceIndex];
		
		// check distance
		const float DistanceToNextPoint = FVector::Dist(RawSurfaceHit.HitLocation, NextRawSurfaceHit.HitLocation);
		if (DistanceToNextPoint < Settings->CleanUpPathPointDistanceThreshold)
		{
			const float NormalDiff = FVector::DotProduct(RawSurfaceHit.HitNormal, NextRawSurfaceHit.HitNormal);
			if (NormalDiff >= Settings->CleanUpPathPointNormalThreshold)
			{
#if SAN_WITH_DEBUG
				if (SAN::Library::Debug::DebugDisplayFindAnySurfacePath > 1)
				{
					FU::Draw::Advanced::DrawDebugSphere(
						World,
						NextRawSurfaceHit.HitLocation,
						20,
						FColor::Red,
						SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
					);
				}
#endif
				
				UE_VLOG_LOCATION(World, VLogCategoryName_SimilarPointsFiltering, Display, 
					NextRawSurfaceHit.HitLocation, 25, 
					FColor::Red, TEXT("Removed [previously %i]"), ComparedNextSourceSurfaceIndex
				);
				
				SurfacePoints.RemoveAt(ComparedNextSourceSurfaceIndex);
				
				// keep same source point, go next
				ComparedNextSourceSurfaceIndex++;
				continue;
			}
		}
		
		UE_VLOG_LOCATION(World, VLogCategoryName_SimilarPointsFiltering, Display, 
			RawSurfaceHit.HitLocation, 15, 
			FColor::Green, TEXT("[%i]"), ComparedNextSourceSurfaceIndex
		);
		
		// we keep the next point, make it source
		ComparedSourceSurfaceIndex++;
		ComparedNextSourceSurfaceIndex = ComparedSourceSurfaceIndex + 1;
	}
}

void USANAnySurfaceNavLibrary::KeepShortestDistancePoints(UWorld* World, const USANAnySurfaceNavSettings* Settings, const TArray<FSANSurfaceHitResult>& InRawSurfaceHits, TArray<FSANSurfaceHitResult>& OutFilteredRawSurfaceHits)
{
	const FName VLogCategoryName_ShortFiltering = "SANFindAnySurfacePathSync_ShortFiltering";
	
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
					FU::Draw::Advanced::DrawDebugSphere(
						World,
						N1RawSurfaceHit.HitLocation,
						20,
						FColor::Red,
						SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
					);
				}
#endif
				
				UE_VLOG_LOCATION(World, VLogCategoryName_ShortFiltering, Display, 
					N1RawSurfaceHit.HitLocation, 15, 
					FColor::Red, TEXT("Def total dist: %f.0"), DefaultDistance
				);
				
				UE_VLOG_SEGMENT_THICK(World, VLogCategoryName_ShortFiltering, Display, 
					SourceRawSurfaceHit.HitLocation, N1RawSurfaceHit.HitLocation, 
					FColor::Red, 5, TEXT("Dist: %f.0"), FVector::Dist(SourceRawSurfaceHit.HitLocation, N1RawSurfaceHit.HitLocation)
				);
				
				UE_VLOG_SEGMENT_THICK(World, VLogCategoryName_ShortFiltering, Display, 
					N1RawSurfaceHit.HitLocation, N2RawSurfaceHit.HitLocation, 
					FColor::Red, 5, TEXT("Dist: %f.0"), FVector::Dist(N1RawSurfaceHit.HitLocation, N2RawSurfaceHit.HitLocation)
				);
				
				UE_VLOG_SEGMENT_THICK(World, VLogCategoryName_ShortFiltering, Display, 
					SourceRawSurfaceHit.HitLocation, N2RawSurfaceHit.HitLocation, 
					FColor::Green, 7, TEXT("N2 dist: %f.0"), N2Distance
				);
				
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
	
	// TODO: directly add points in while loop instead to avoid a second loop
	for (int32 InSurfaceHitsIndex = 0; InSurfaceHitsIndex < InRawSurfaceHits.Num(); ++InSurfaceHitsIndex)
	{
		if (!RemovedIndices.Contains(InSurfaceHitsIndex))
		{
			OutFilteredRawSurfaceHits.Emplace(InRawSurfaceHits[InSurfaceHitsIndex]);
			UE_VLOG_LOCATION(World, VLogCategoryName_ShortFiltering, Display, 
				InRawSurfaceHits[InSurfaceHitsIndex].HitLocation, 10, 
				FColor::Cyan, TEXT("Base Point [%i], \n N: %s"), OutFilteredRawSurfaceHits.Num() - 1, *FU::Utils::PrintCompactVector(InRawSurfaceHits[InSurfaceHitsIndex].HitNormal)
			);
		}
	}
}

void USANAnySurfaceNavLibrary::FindAndInsertSubdivisions(UWorld* World, const USANAnySurfaceNavSettings* Settings, const TArray<FSANSurfaceHitResult>& InSurfaceHitResults, TArray<FSANSurfaceHitResult>& OutSurfaceHitResults)
{
	const FName VLogCategoryName_Subdivisions = "SANFindAnySurfacePathSync_Subdivisions";
	
	TArray<FSANSurfaceHitResult> PreviousSurfaces;
	PreviousSurfaces.Reserve(10);
	
	OutSurfaceHitResults.Reserve(InSurfaceHitResults.Num());
	
	for (int32 PointIndex = 0; PointIndex < InSurfaceHitResults.Num() - 1; ++PointIndex)
	{
		const auto& CurrentPoint = InSurfaceHitResults[PointIndex];
		const auto& NextPoint = InSurfaceHitResults[PointIndex + 1];
		
		OutSurfaceHitResults.Emplace(CurrentPoint);
		
		UE_VLOG_LOCATION(World, VLogCategoryName_Subdivisions, Display, 
			CurrentPoint.HitLocation, 15, 
			FColor::Cyan, TEXT_EMPTY
		);
		
		UE_VLOG_LOCATION(World, VLogCategoryName_Subdivisions, Display, 
			NextPoint.HitLocation, 15, 
			FColor::Cyan, TEXT_EMPTY
		);
		
		// we dont handle subdivisions if the height is to different (it will when filling gaps)
		const float HeighDiff = FMath::Abs(CurrentPoint.HitLocation.Z - NextPoint.HitLocation.Z);
		
		if (HeighDiff >= Settings->SubdivisionsMaxHeightDifference)
		{
			UE_VLOG_SEGMENT_THICK(World, VLogCategoryName_Subdivisions, Display, 
				CurrentPoint.HitLocation, NextPoint.HitLocation, 
				FColor::Red, 5, TEXT("Subdivs [%i] To much height diff"), PointIndex
			);
			
			UE_VLOG_LOCATION(World, VLogCategoryName_Subdivisions, Display, 
				CurrentPoint.HitLocation, 15, 
				FColor::Red, TEXT_EMPTY
			);
			
			UE_VLOG_LOCATION(World, VLogCategoryName_Subdivisions, Display, 
				NextPoint.HitLocation, 15, 
				FColor::Red, TEXT_EMPTY
			);
			
			continue;
		}
		
		// see if we have to do subdivions if nav points are far enough from each other
		const float DistanceToNextPoint = FVector::Dist(CurrentPoint.HitLocation, NextPoint.HitLocation);
		
		if (DistanceToNextPoint < Settings->MinDistanceBetweenSubdivisions)
		{
			UE_VLOG_SEGMENT_THICK(World, VLogCategoryName_Subdivisions, Display, 
				CurrentPoint.HitLocation, NextPoint.HitLocation, 
				FColor::Red, 5, TEXT("Subdivs [%i] short distance"), PointIndex
			);
			
			UE_VLOG_LOCATION(World, VLogCategoryName_Subdivisions, Display, 
				CurrentPoint.HitLocation, 15, 
				FColor::Red, TEXT_EMPTY
			);
			
			UE_VLOG_LOCATION(World, VLogCategoryName_Subdivisions, Display, 
				NextPoint.HitLocation, 15, 
				FColor::Red, TEXT_EMPTY
			);
			continue;
		}
		
		// make subdivisions
		const int32 TotalNbOfSubdivisions = FMath::Floor(DistanceToNextPoint / Settings->MinDistanceBetweenSubdivisions);
		// we skip the "last" point in the subdivision since it will be the next nav point
		const int32 NbOfSubdivisions = TotalNbOfSubdivisions - 1;
		
		if (NbOfSubdivisions > 0)
		{
			OutSurfaceHitResults.Reserve(OutSurfaceHitResults.Num() + NbOfSubdivisions);
			
			UE_VLOG_SEGMENT_THICK(World, VLogCategoryName_Subdivisions, Display, 
				CurrentPoint.HitLocation, NextPoint.HitLocation, 
				FColor::Cyan, 5, TEXT("Subdivs [%i] Count: %i"), PointIndex, NbOfSubdivisions
			);
			
			for (int32 SbdvIndx = 1; SbdvIndx < NbOfSubdivisions + 1; ++SbdvIndx)
			{
				const FVector SbdvLocation = FMath::Lerp(CurrentPoint.HitLocation, NextPoint.HitLocation, (float)SbdvIndx/TotalNbOfSubdivisions);
				
				TArray<FSANSurfaceHitResult> BestSurfaceHits;
				float Radius = Settings->SurfaceCollisionSphereMinRadius;
				if (GetBestSurface(World, Settings, SbdvLocation, Radius, PreviousSurfaces, BestSurfaceHits))
				{
					for (const auto& BestSurfaceHit : BestSurfaceHits)
					{
						OutSurfaceHitResults.Emplace(BestSurfaceHit);
						
						UE_VLOG_LOCATION(World, VLogCategoryName_Subdivisions, Display, 
							BestSurfaceHit.HitLocation, 20, 
							FColor::Blue, TEXT("Subdiv [%i:%i] \nN: %s"), PointIndex, SbdvIndx, *FU::Utils::PrintCompactVector(BestSurfaceHit.HitNormal)
						);
						
						UE_VLOG_ARROW(World, VLogCategoryName_Subdivisions, Display, 
							BestSurfaceHit.HitLocation, BestSurfaceHit.HitLocation + BestSurfaceHit.HitNormal * 50, 
							FColor::Cyan, TEXT_EMPTY
						);
					}
				}
				
				// override any previous surfaces
				PreviousSurfaces = BestSurfaceHits;
			}
		}
	}
}

bool USANAnySurfaceNavLibrary::FillGaps(UWorld* World, const USANAnySurfaceNavSettings* Settings, TArray<FSANSurfaceHitResult>& PreviousSurfaces, TArray<FSANSurfaceHitResult>& RawSurfaceHits)
{
	const FName VLogCategoryName_FillGaps = "SANFindAnySurfacePathSync_FillGaps";
	
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
		const FSANSurfaceHitResult NextRawSurfaceHit = RawSurfaceHits[RawSurfaceHitIndex + 1];
		
		const float DistanceToNextPoint = FVector::Dist(RawSurfaceHit.HitLocation, NextRawSurfaceHit.HitLocation);
		
		if (DistanceToNextPoint > Settings->GapsMaxDistanceBetweenPoints)
		{
#if SAN_WITH_DEBUG
			if (SAN::Library::Debug::DebugDisplayFindAnySurfacePath > 1)
			{
				FU::Draw::Advanced::DrawDebugSolidLine(World,
				   RawSurfaceHit.HitLocation,
				   NextRawSurfaceHit.HitLocation,
				   FColor::Yellow,
				   SAN::Library::Debug::DebugDisplayFindAnySurfacePathTime
			   );
			}
#endif
			
			const FColor RandomColor = FColor::MakeRandomColor();
			const float DebugSphereSize = 20 / FillGapsLoopCount;
			
			// make subdivisions
			const int32 TotalNbOfSubdivisions = FMath::Floor(DistanceToNextPoint / Settings->GapsMinDistanceBetweenSubdivisions);
			// we skip the "last" point in the subdivision since it will be the next nav point
			const int32 NbOfSubdivisions = TotalNbOfSubdivisions - 1;
			
			if (NbOfSubdivisions > 0)
			{
				bDidChanged = true;
				
				UE_VLOG_SEGMENT_THICK(World, VLogCategoryName_FillGaps, Display, 
					RawSurfaceHit.HitLocation, NextRawSurfaceHit.HitLocation, 
					RandomColor, 5, TEXT("Subdivs [%i] Count: %i"), FillGapsLoopCount - 1, NbOfSubdivisions
				);
				
				for (int32 SbdvIndx = 1; SbdvIndx < NbOfSubdivisions + 1; ++SbdvIndx)
				{
					const FVector SbdvLocation = FMath::Lerp(RawSurfaceHit.HitLocation, NextRawSurfaceHit.HitLocation, (float)SbdvIndx/TotalNbOfSubdivisions);
					
					TArray<FSANSurfaceHitResult> BestSurfaceHits;
					float Radius = Settings->SurfaceCollisionSphereMinRadius;
					if (GetBestSurface(World, Settings, SbdvLocation, Radius, PreviousSurfaces, BestSurfaceHits))
					{
						// inject filling surfaces at the correct indices
						for (int32 i = 0; i < BestSurfaceHits.Num(); ++i)
						{
							RawSurfaceHits.EmplaceAt(RawSurfaceHitIndex + 1, BestSurfaceHits[i]);
							RawSurfaceHitIndex++;
							
							UE_VLOG_LOCATION(World, VLogCategoryName_FillGaps, Display, 
								BestSurfaceHits[i].HitLocation, DebugSphereSize, 
								RandomColor, TEXT("Subdiv [%i:%i(%i)] \nN: %s"), 
								FillGapsLoopCount - 1, SbdvIndx - 1, i, 
								*FU::Utils::PrintCompactVector(BestSurfaceHits[i].HitNormal)
							);
							
							UE_VLOG_ARROW(World, VLogCategoryName_FillGaps, Display, 
								BestSurfaceHits[i].HitLocation, BestSurfaceHits[i].HitLocation + BestSurfaceHits[i].HitNormal * 50, 
								RandomColor, TEXT_EMPTY
							);
						}
					}
					
					// override any past surfaces
					PreviousSurfaces = BestSurfaceHits;
				}
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
