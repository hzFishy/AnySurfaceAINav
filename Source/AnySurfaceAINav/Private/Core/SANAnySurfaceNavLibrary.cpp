// By hzFishy - 2026 - Do whatever you want with it.

#include "Core/SANAnySurfaceNavLibrary.h"
#include "CPathVolume.h"
#include "Data/SANAnySurfaceNavSettings.h"
#include "Draw/FUColors.h"
#include "Kismet/GameplayStatics.h"
#include "Logging/FULogging.h"


namespace SAN::Library
{
#if SAN_WITH_DEBUG
	namespace Debug
	{
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
	
#if ENABLE_VISUAL_LOG
	namespace VLog
	{
		inline FName RawCPathResult = "SANFindAnySurfacePathSync_1_Raw";
		inline FName BasePass = "SANFindAnySurfacePathSync_2_BasePass";
		inline FName BasePassReordered = "SANFindAnySurfacePathSync_2.1_BasePassReordered";
		inline FName BasePassSimilarPointsFiltering = "SANFindAnySurfacePathSync_2.2_SimilarPointsFiltering";
		inline FName BasePassShortPathFiltering = "SANFindAnySurfacePathSync_2.3_ShortPathFiltering";
		inline FName FillGaps = "SANFindAnySurfacePathSync_3_FillGaps";
		inline FName FillGapsSimilarPointsFiltering = "SANFindAnySurfacePathSync_3.1_FillGapsSimilarPointsFiltering";
		inline FName FillGapsShortPathFiltering = "SANFindAnySurfacePathSync_3.2_FillGapsShortPathFiltering";
		inline FName SmoothPass = "SANFindAnySurfacePathSync_4_SmoothPass";
		inline FName SmoothPass_BlockingTest = "SANFindAnySurfacePathSync_4_SmoothPass_BlockingTest";
		inline FName SmoothPassSimilarPointsFiltering = "SANFindAnySurfacePathSync_4.1_SmoothPassSimilarPointsFiltering";
		inline FName SmoothPassBlockingFiltering = "SANFindAnySurfacePathSync_4.2_SmoothPassBlockingFiltering";
		inline FName FinalPath = "SANFindAnySurfacePathSync_5_FinalPath";
		
		inline FName IsLineBlocking = "SANFindAnySurfacePathSync_IsLineBlocking";
		inline FName IsPointBlocked = "SANFindAnySurfacePathSync_IsPointBlocked";
		
		static int32 DebugSmoothLoopCount = 0;
	}
#endif
}

int32 USANAnySurfaceNavLibrary::FillGapsLoopCount = 0;
#if SAN_WITH_DEBUG
int32 USANAnySurfaceNavLibrary::FillGapsLoopCountDebug = 0;
#endif


bool USANAnySurfaceNavLibrary::FindAnySurfacePathSync(const FSANFindPathRequest& Request, FSANFindPathResult& Result)
{
	using namespace SAN::Library;
	
	TRACE_CPUPROFILER_EVENT_SCOPE(SAN::GlobalFindPathSync)
	
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
	
	const auto* AnySurfaceNavSettings = USANAnySurfaceNavSettings::Get();
	
	// if the 3D nav pathfinding gave no points we abort
	if (CPathResult.UserPath.IsEmpty())
	{
		SAN_VLOG_Static_W(World, "Find Path found no points")
		return false;
	}
	else
	{
		// we will insert points in the raw 3D path if there is to much distance between 2 points
		
		// TODO: move to settings
		const float SubdivRawDistance = 70;
		
		SAN_VLOG_Static_D(World, "CPath FindPath found %i points", CPathResult.UserPath.Num());
#if SAN_WITH_DEBUG
		int32 DebugInsertedRawPointsCount = 0;
#endif
		
		for (int32 RawIndx = 0; RawIndx < CPathResult.UserPath.Num(); ++RawIndx)
		{
			auto& CPathNode = CPathResult.UserPath[RawIndx];
			
			Result.NavPathPoints.Emplace(CPathNode.WorldLocation);
			
			UE_VLOG_LOCATION(World, VLog::RawCPathResult, Display,
				CPathNode.WorldLocation, 15,
				FColor::Magenta, TEXT("Raw Point [%i]"), RawIndx
			);
			
			if (RawIndx == CPathResult.UserPath.Num() - 1)
			{
				continue;
			}
			
			auto& NextCPathNode = CPathResult.UserPath[RawIndx + 1];
			
			UE_VLOG_SEGMENT_THICK(World, VLog::RawCPathResult, Display, 
				CPathNode.WorldLocation, NextCPathNode.WorldLocation, 
				FColor::Purple, 7, TEXT_EMPTY
			);
			
			const float DistanceToNextPoint = FVector::Distance(CPathNode.WorldLocation, NextCPathNode.WorldLocation);
			
			// make subdivisions
			const int32 TotalNbOfSubdivisions = FMath::Floor(DistanceToNextPoint / SubdivRawDistance);
			// we skip the "last" point in the subdivision since it will be the next nav point
			const int32 NbOfSubdivisions = TotalNbOfSubdivisions - 1;
			
			if (NbOfSubdivisions > 0)
			{
				for (int32 SbdvIndx = 1; SbdvIndx < NbOfSubdivisions + 1; ++SbdvIndx)
				{
					const FVector SbdvLocation = FMath::Lerp(CPathNode.WorldLocation, NextCPathNode.WorldLocation, (float)SbdvIndx/TotalNbOfSubdivisions);
					
					Result.NavPathPoints.Emplace(SbdvLocation);
					
					UE_VLOG_LOCATION(World, VLog::RawCPathResult, Display,
						SbdvLocation, 8,
						FColor::Purple, TEXT("Raw Point [%i] (Subdiv)"), RawIndx
					);
					
					DebugInsertedRawPointsCount++;
				}
			}
		}
		
		SAN_VLOG_Static_D(World, "Inserted %i points to raw path with subdivisions", DebugInsertedRawPointsCount);
	}
	
	FCollisionQueryParams CollisionQueryParams;
	MakeCollisionQueryParamsFromRequest(Request, CollisionQueryParams);
	
	// from each point, get closest surface(s) to build a surface "path" (Base Pass)
	FSANSurfaceHitResult PreviousSurface;
	TArray<FSANSurfaceHitResult> BasePassSurfacePoints;
	BasePassSurfacePoints.Reserve(Result.NavPathPoints.Num() * 5);
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(SAN::BasePass)
		
		TArray<FSANSurfaceHitResult> BasePassSurfaceExtraPoints;
		BasePassSurfaceExtraPoints.Reserve(20);
		
		for (int32 NavPointIndx = 0; NavPointIndx < Result.NavPathPoints.Num(); ++NavPointIndx)
		{
			const auto& CurrentPoint = Result.NavPathPoints[NavPointIndx];
			
			TArray<FSANSurfaceHitResult> BestSurfaceHits;
			float Radius = AnySurfaceNavSettings->SurfaceCollisionSphereMinRadius;
			
			if (GetBestSurface(World, AnySurfaceNavSettings, CollisionQueryParams, CurrentPoint.Location, Radius, PreviousSurface, BestSurfaceHits))
			{
				// add first surface since its the closest to previous best point index
				BasePassSurfacePoints.Emplace(BestSurfaceHits[0]);
				
				UE_VLOG_WIRESPHERE(World, VLog::BasePass, Display, 
					CurrentPoint.Location, Radius, FColor::Orange, TEXT("[%i]"), NavPointIndx
				);
				
				BasePassSurfaceExtraPoints.Reserve(BasePassSurfaceExtraPoints.Num() + BestSurfaceHits.Num() - 1);
				
				for (int32 InnerSurfaceHitIndx = 0; InnerSurfaceHitIndx < BestSurfaceHits.Num(); ++InnerSurfaceHitIndx)
				{
					const auto& BestSurfaceHit = BestSurfaceHits[InnerSurfaceHitIndx];
					
					// skip first index
					if (InnerSurfaceHitIndx >= 1)
					{
						BasePassSurfaceExtraPoints.Emplace(BestSurfaceHit);
					}
					
					UE_VLOG_LOCATION(World, VLog::BasePass, Display, 
						BestSurfaceHit.HitLocation, 10, 
						InnerSurfaceHitIndx == 0 ? FColor::Green : FColor::Orange, TEXT("Base Point [%i::%i], \n N: %s"), NavPointIndx, InnerSurfaceHitIndx, *FU::Utils::PrintCompactVector(BestSurfaceHit.HitNormal)
					);
					
					UE_VLOG_ARROW(World, VLog::BasePass, Display, 
						BestSurfaceHit.HitLocation, BestSurfaceHit.HitLocation + BestSurfaceHit.HitNormal * 50, 
						FColor::Emerald, TEXT_EMPTY
					);
				}
				
				// override any previous surface
				PreviousSurface = BestSurfaceHits[0];
			}
		}
		
		// base pass done, now we iterate the extra points and insert them
		{
			RemoveSimilarPointsInArrays(BasePassSurfacePoints, BasePassSurfaceExtraPoints);
			
			// fill in the points in-between
			for (auto& BasePassSurfaceExtraPoint : BasePassSurfaceExtraPoints)
			{
				const int32 Index = FindBestInBetweenIndex(BasePassSurfaceExtraPoint, BasePassSurfacePoints);
				BasePassSurfacePoints.EmplaceAt(Index + 1, BasePassSurfaceExtraPoint);
			}
			
#if SAN_WITH_DEBUG
			// debug results
			for (int32 Index = 0; Index < BasePassSurfacePoints.Num(); ++Index)
			{
				const auto& BasePassSurfacePoint = BasePassSurfacePoints[Index];
				
				UE_VLOG_LOCATION(World, VLog::BasePassReordered, Display, 
					BasePassSurfacePoint.HitLocation, 10, 
					FColor::Green, TEXT("Base Point [%i], \n N: %s"), Index, *FU::Utils::PrintCompactVector(BasePassSurfacePoint.HitNormal)
				);
				
				UE_VLOG_ARROW(World, VLog::BasePassReordered, Display, 
					BasePassSurfacePoint.HitLocation, BasePassSurfacePoint.HitLocation + BasePassSurfacePoint.HitNormal * 50, 
					FColor::Emerald, TEXT_EMPTY
				);
			}
#endif
		}
		
		// TODO: find closest point to the pawn and remove any points before since we dont want the agent to "go backwards" when moving to goal location
	}
	// clear before further use
	PreviousSurface.Reset();
	
	
	// remove points that are to close with similar normal
	RemoveSimilarPoints(
		World, 
		AnySurfaceNavSettings, 
		CollisionQueryParams, 
		Request.AgentRadius, 
		BasePassSurfacePoints,
		VLog::BasePassSimilarPointsFiltering
	);
	
	// from generated points find shortest paths
	TArray<FSANSurfaceHitResult> BasePassShortenSurfacePoints;
	{
#if SAN_WITH_DEBUG
		if (SAN::Library::Debug::DebugDisableShortPathFiltering == 0)
		{
			KeepShortestDistancePoints(
				World, 
				AnySurfaceNavSettings, 
				CollisionQueryParams, 
				Request.AgentRadius, 
				BasePassSurfacePoints, 
				BasePassShortenSurfacePoints, 
				VLog::BasePassShortPathFiltering
			);
		}
		else
		{
			// since we are not calling KeepShortestDistancePoints we have to copy over the results like nothing was removed
			BasePassShortenSurfacePoints = BasePassSurfacePoints;
		}
#else 
		KeepShortestDistancePoints(
			World, 
			AnySurfaceNavSettings, 
			CollisionQueryParams, 
			Request.AgentRadius, 
			BasePassSurfacePoints, 
			BasePassShortenSurfacePoints, 
			VLog::BasePassShortPathFiltering
		);
#endif
	}
	
	// clear before further use
	PreviousSurface.Reset();
	
	// filling gaps: try to clean the path if there is a big distance between found surfaces points
	TArray<FSANSurfaceHitResult> FillGapsFinalSurfaces;
	{
		TArray<FSANSurfaceHitResult> BasePassSurfaceExtraPoints;
		BasePassSurfaceExtraPoints.Reserve(20);
		
		FillGapsLoopCount = 0;
#if SAN_WITH_DEBUG
		FillGapsLoopCountDebug = -1;
#endif
		
		TRACE_CPUPROFILER_EVENT_SCOPE(SAN::FillGapsFiltering)
#if SAN_WITH_DEBUG
		if (SAN::Library::Debug::DebugDisableFillGaps == 0)
		{
			FillGaps(
				World, 
				AnySurfaceNavSettings, 
				CollisionQueryParams,
				Request.AgentRadius, 
				BasePassShortenSurfacePoints, 
				BasePassSurfaceExtraPoints
			);
		}
#else 
		FillGaps(
			World, 
			AnySurfaceNavSettings, 
			CollisionQueryParams,
			Request.AgentRadius, 
			BasePassShortenSurfacePoints, 
			BasePassSurfaceExtraPoints
		);
#endif
		
		// remove points that are to close with similar normal
		RemoveSimilarPoints(
			World, 
			AnySurfaceNavSettings, 
			CollisionQueryParams, 
			Request.AgentRadius, 
			BasePassShortenSurfacePoints, 
			VLog::FillGapsSimilarPointsFiltering
		);
		
		RemoveSimilarPointsInArrays(BasePassShortenSurfacePoints, BasePassSurfaceExtraPoints);
		
		// fill in the points in-between
		for (auto& BasePassSurfaceExtraPoint : BasePassSurfaceExtraPoints)
		{
			const int32 Index = FindBestInBetweenIndex(BasePassSurfaceExtraPoint, BasePassShortenSurfacePoints);
			BasePassShortenSurfacePoints.EmplaceAt(Index + 1, BasePassSurfaceExtraPoint);
		}
		
		{
			// TODO: maybe remove shortening process after filling gaps since this usually just undo the fill gaps work
	#if SAN_WITH_DEBUG
			if (SAN::Library::Debug::DebugDisableShortPathFiltering == 0)
			{
				KeepShortestDistancePoints(
					World, 
					AnySurfaceNavSettings, 
					CollisionQueryParams, 
					Request.AgentRadius, 
					BasePassShortenSurfacePoints, 
					FillGapsFinalSurfaces, 
					VLog::FillGapsShortPathFiltering
				);
			}
			else
			{
				// since we are not calling KeepShortestDistancePoints we have to copy over the results like nothing was removed
				FillGapsFinalSurfaces = BasePassShortenSurfacePoints;
			}
#else 
			KeepShortestDistancePoints(
				World, 
				AnySurfaceNavSettings, 
				CollisionQueryParams, 
				Request.AgentRadius, 
				BasePassShortenSurfacePoints, 
				FillGapsFinalSurfaces, 
				VLog::FillGapsShortPathFiltering
			);
#endif
		}
	}
	
	// extra: smooth the path by blending with the environment
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(SAN::Smoothing)
		
		// TODO: maybe replace this big ugly while loop with something cleaner with a recursive function
		
		// did we added points this iteration ?
		bool bPointsAdded = true;
		
#if ENABLE_VISUAL_LOG
		VLog::DebugSmoothLoopCount = 0;
#endif
		
		while (bPointsAdded)
		{
			// iterate all points from start to end-1 until we could smooth everything
			
			TArray<FSANSurfaceHitResult> FoundNewSurfaces;
			for (int32 SurfaceIndex = 0; SurfaceIndex < FillGapsFinalSurfaces.Num() - 1; ++SurfaceIndex)
			{
				const auto& CurrentSurface = FillGapsFinalSurfaces[SurfaceIndex];
				const auto& NextSurface = FillGapsFinalSurfaces[SurfaceIndex + 1];
				SmoothSegment(
					World, 
					AnySurfaceNavSettings, 
					CollisionQueryParams, 
					Request.AgentRadius, 
					CurrentSurface,
					NextSurface, 
					FoundNewSurfaces
				);
			}
			
			// now we filter and insert FoundNewSurfaces
			RemoveSimilarPointsInArrays(FillGapsFinalSurfaces, FoundNewSurfaces);
			
			for (auto& NewSurface : FoundNewSurfaces)
			{
				const int32 Index = FindBestInBetweenIndex(NewSurface, FillGapsFinalSurfaces);
				FillGapsFinalSurfaces.EmplaceAt(Index + 1, NewSurface);
			}
			
			// TODO: maybe instead only filter FoundNewSurfaces to save loop time
			// remove points that are to close with similar normal
			/*RemoveSimilarPoints(
				World, 
				AnySurfaceNavSettings, 
				CollisionQueryParams, 
				Request.AgentRadius, 
				FillGapsFinalSurfaces, 
				VLog::SmoothPassSimilarPointsFiltering
			);*/
			
			// last filter: check if the agent can move between all the points
			CheckAndClearUnusablePoints(
				World, 
				AnySurfaceNavSettings, 
				CollisionQueryParams, 
				Request.AgentRadius, 
				FillGapsFinalSurfaces
			);
			
			bPointsAdded = !FoundNewSurfaces.IsEmpty();
			
#if ENABLE_VISUAL_LOG
			VLog::DebugSmoothLoopCount++;
			
			// FIXME:
			if (VLog::DebugSmoothLoopCount > 0)
			{
				break;
			}
#endif
		}
	}
	
	// save results
	Result.SurfaceHitResults = FillGapsFinalSurfaces;
	
#if SAN_WITH_DEBUG
	for (int32 i = 0; i < Result.SurfaceHitResults.Num(); ++i)
	{
		const auto& HitResult = Result.SurfaceHitResults[i];
		
		UE_VLOG_LOCATION(World, VLog::FinalPath, Display, 
			HitResult.HitLocation, 15, 
			FColor::Magenta, TEXT("Point [%i]"), i
		);
		
		UE_VLOG_ARROW(World, VLog::FinalPath, Display, 
			HitResult.HitLocation, HitResult.HitLocation + HitResult.HitNormal * 50, 
			FColor::Magenta, TEXT_EMPTY
		);
		
		if (i < Result.SurfaceHitResults.Num() - 1)
		{
			UE_VLOG_SEGMENT_THICK(World, VLog::FinalPath, Display, 
				HitResult.HitLocation, Result.SurfaceHitResults[i + 1].HitLocation, 
				FColor::Purple, 7, TEXT_EMPTY
			);
		}
	}
#endif
	
	SAN_VLOG_Static_D(World, "Final results: kept %i surfaces", Result.SurfaceHitResults.Num());
	
	return !Result.IsEmpty();
}

bool USANAnySurfaceNavLibrary::IsPathResultEmpty(const FSANFindPathResult& PathResult)
{
	return PathResult.IsEmpty();
}

bool USANAnySurfaceNavLibrary::GetBestSurface(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams,
	const FVector PointLocation, float& Radius, const FSANSurfaceHitResult& PreviousSurface, TArray<FSANSurfaceHitResult>& OutBestSurfaces)
{
	// there is always a result
	OutBestSurfaces.AddDefaulted(1);
	
	TArray<FHitResult> HitResults;
	// trace until we find a surface
	// TODO: from previous used radius use that as base to avoid repititive traces to find similar radius
	const bool bFoundSurface = GetBestSurfaceInternal(World, Settings, CollisionQueryParams, PointLocation, Radius, HitResults);
	
	if (bFoundSurface && !HitResults.IsEmpty())
	{
		if (PreviousSurface.IsValid())
		{
			if (HitResults.Num() > 1)
			{
				// remove hits blocked by geometry (if there is a blocking primitive between the nav point and the found surfaces)
				// this is because we are using overlap sweep in GetBestSurfaceInternal to capture all possible surfaces
				for (auto HitIt = HitResults.CreateIterator(); HitIt; ++HitIt)
				{
					FHitResult BlockHitResult;
					const bool bHit = World->LineTraceSingleByProfile(
						BlockHitResult,
						PointLocation,
						(*HitIt).ImpactPoint,
						Settings->BlockSurfaceCollisionProfile.Name,
						CollisionQueryParams
					);
					
					if (bHit && (*HitIt).GetComponent() != BlockHitResult.GetComponent())
					{
						HitIt.RemoveCurrent();
					}
				}
				
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
					const float Distance = FVector::Dist(HitResult.ImpactPoint, PreviousSurface.HitLocation);
					
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
					const float Dot = FVector::DotProduct(HitResult.ImpactNormal, PreviousSurface.HitNormal);
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
					// TODO: to avoid repetitive loops just fix above code so we never have to check for uniqueness
					OutBestSurfaces.AddUnique(HitResults[ExtraDiffSurfaces[i]]);
				}
				
				// sort the out surfaces so we order from closest to furthest from the best previous location
				OutBestSurfaces.Sort([PreviousSurface] (const FSANSurfaceHitResult& A, const FSANSurfaceHitResult& B)
				{
					return FVector::Dist(A.HitLocation, PreviousSurface.HitLocation) < FVector::Dist(B.HitLocation, PreviousSurface.HitLocation);
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
	
	return !HitResults.IsEmpty();
}

bool USANAnySurfaceNavLibrary::GetBestSurfaceInternal(UWorld* World, const USANAnySurfaceNavSettings* Settings, 
	const FCollisionQueryParams& CollisionQueryParams, const FVector PointLocation, float& Radius, TArray<FHitResult>& HitResults)
{
	World->SweepMultiByProfile(
		HitResults,
		PointLocation,
		PointLocation,
		FQuat::Identity,
		Settings->OverlapSurfaceCollisionProfile.Name,
		FCollisionShape::MakeSphere(Radius),
		CollisionQueryParams
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
		
		return GetBestSurfaceInternal(World, Settings, CollisionQueryParams, PointLocation, Radius, HitResults);
	}
	else
	{
		return true;
	}
}

void USANAnySurfaceNavLibrary::RemoveSimilarPoints(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams, 
	float AgentRadius, TArray<FSANSurfaceHitResult>& SurfacePoints, const FName VLogName)
{
	// TODO: instead of just removing a similar point thats n+1, remove the ones that makes the path the less "straight"
	
	TRACE_CPUPROFILER_EVENT_SCOPE(SAN::ShortDistanceFiltering)
	
#if SAN_WITH_DEBUG
	if (SAN::Library::Debug::DebugDisableDistanceNormalFiltering > 0)
	{
		return;
	}
#endif
	
#if ENABLE_VISUAL_LOG
	int32 DebugSameSourcePointCounter = 0;
#endif
	
	int32 ComparedSourceSurfaceIndex = 0;
	int32 ComparedNextSourceSurfaceIndex = 1;
	while (ComparedSourceSurfaceIndex < SurfacePoints.Num())
	{
		const auto& RawSurfaceHit = SurfacePoints[ComparedSourceSurfaceIndex];
		
		if (ComparedSourceSurfaceIndex == SurfacePoints.Num() - 1)
		{
			UE_VLOG_SPHERE(World, VLogName, Display, 
				RawSurfaceHit.HitLocation, 15, 
				FColor::Green, TEXT_EMPTY
			);
			
			UE_VLOG_LOCATION(World, VLogName, Display, 
				RawSurfaceHit.HitLocation + FVector(0, 0, 20), 0, 
				FColor::Green, TEXT("[%i] (End)"), ComparedSourceSurfaceIndex
			);
			
			return;
		}
		
		const auto& NextRawSurfaceHit = SurfacePoints[ComparedNextSourceSurfaceIndex];
		
		// check distance
		const float DistanceToNextPoint = FVector::Dist(RawSurfaceHit.HitLocation, NextRawSurfaceHit.HitLocation);
		float NormalDiff = -2;

#if ENABLE_VISUAL_LOG
		bool bDebugDistanceTestPass = false;
		bool bDebugNormalTestPass = false;
		bool bDebugDidLineTestN1 = false;
		bool bDebugBlockedN1 = false;
		bool bDebugDidLineTestN2 = false;
		bool bDebugBlockedN2 = false;
#endif
		
		// some params are captured by ref so the value is updated as the code is executed before calling the lambda
#if ENABLE_VISUAL_LOG
		auto DebugResults = 
			[
				World, VLogName, RawSurfaceHit, NextRawSurfaceHit, ComparedSourceSurfaceIndex, &bDebugDistanceTestPass, 
				DistanceToNextPoint, &bDebugNormalTestPass, &NormalDiff, &bDebugBlockedN1, &bDebugBlockedN2, &bDebugDidLineTestN1, &bDebugDidLineTestN2, 
				ComparedNextSourceSurfaceIndex, Settings, &DebugSameSourcePointCounter
			]
			(bool bRemoved)
			{
				
				UE_VLOG_SPHERE(World, VLogName, Display, 
					RawSurfaceHit.HitLocation, 15, 
					FColor::Green, TEXT_EMPTY
				);
					
				UE_VLOG_ARROW(World, VLogName, Display, 
					RawSurfaceHit.HitLocation, 
					RawSurfaceHit.HitLocation + RawSurfaceHit.HitNormal * 50, 
					FColor::Green, TEXT_EMPTY
				);
				
				if (bRemoved)
				{
					UE_VLOG_SPHERE(World, VLogName, Display, 
						NextRawSurfaceHit.HitLocation, 15, 
						FColor::Red, TEXT_EMPTY
					);
					
					UE_VLOG_ARROW(World, VLogName, Display, 
						NextRawSurfaceHit.HitLocation, 
						NextRawSurfaceHit.HitLocation + NextRawSurfaceHit.HitNormal * 50, 
						FColor::Red, TEXT_EMPTY
					);
					
					UE_VLOG_LOCATION(World, VLogName, Display, 
						NextRawSurfaceHit.HitLocation + FVector(0, 0, 20), 0, 
						FColor::Red, TEXT("[Old %i]"), ComparedNextSourceSurfaceIndex 
					);
				}
				
				FU::Utils::FFUMessageBuilder BuiltDebugString = FString::Printf(TEXT("[%i] (vs %i)"), ComparedSourceSurfaceIndex, ComparedNextSourceSurfaceIndex);
				BuiltDebugString.NewLinef(TEXT("-Dist: %.1f %s %.1f && %.1f"), DistanceToNextPoint, bDebugDistanceTestPass ? TEXT("<") : TEXT(">"), Settings->CleanUpPathPointDistanceThreshold, Settings->CleanUpPathPointDistanceHardThreshold);
				if (bDebugDistanceTestPass)
				{
					BuiltDebugString.NewLinef(TEXT("-Nrm: %.1f %s %.1f"), NormalDiff, bDebugNormalTestPass ? TEXT(">=") : TEXT("<"), Settings->CleanUpPathPointNormalThreshold);
				}
				if (bDebugDidLineTestN1)
				{
					BuiltDebugString.NewLinef(TEXT("-Blocked (N0->N1): %s"), bDebugBlockedN1 ? TEXT("Y") : TEXT("N"));
					
					if (bDebugDidLineTestN2)
					{
						BuiltDebugString.NewLinef(TEXT("-Blocked (N0->N2): %s"), bDebugBlockedN2 ? TEXT("Y") : TEXT("N"));
					}
				}
				
				UE_VLOG_LOCATION(World, VLogName, Display, 
					RawSurfaceHit.HitLocation + FVector(0, 0, DebugSameSourcePointCounter * 20), 0, 
					bRemoved ? FColor::Red : FColor::Green, TEXT("%s"), *BuiltDebugString.GetMessage() 
				);
			};
#endif
		
		// check distance from N0 and N1
		if (DistanceToNextPoint <= Settings->CleanUpPathPointDistanceThreshold)
		{
			bDebugDistanceTestPass = true;
			
			// check normal diff between N0 and N1
			NormalDiff = FVector::DotProduct(RawSurfaceHit.HitNormal, NextRawSurfaceHit.HitNormal);
			if (DistanceToNextPoint <= Settings->CleanUpPathPointDistanceHardThreshold || NormalDiff >= Settings->CleanUpPathPointNormalThreshold)
			{
				
#if ENABLE_VISUAL_LOG
				if (NormalDiff >= Settings->CleanUpPathPointNormalThreshold)
				{
					bDebugNormalTestPass = true;
				}
#endif
				
				
				// check if there is any blocking collisions between NO and N1
				FHitResult HitResultN1;
				
				const FVector StartLocN0 = RawSurfaceHit.HitLocation + (RawSurfaceHit.HitNormal * (AgentRadius + 5) );
				const FVector EndLocN1 = NextRawSurfaceHit.HitLocation + (NextRawSurfaceHit.HitNormal * (AgentRadius + 5));
				
				const bool bHitN1 = World->LineTraceSingleByProfile(
					HitResultN1, 
					StartLocN0, EndLocN1, 
					Settings->BlockSurfaceCollisionProfile.Name, 
					CollisionQueryParams
				);
				
				bDebugDidLineTestN1 = true;
				const bool bPassedLineOfSightCheckN1 = !bHitN1;
				bDebugBlockedN1 = bHitN1;
				
#if ENABLE_VISUAL_LOG
				{
					const FColor HitColor = bHitN1 ? FColor::Red : FColor::Green;
					
					UE_VLOG_SEGMENT_THICK(World, VLogName, Display,
						StartLocN0, EndLocN1, 
						HitColor, 5, TEXT("[%i] vs [%i]"), ComparedSourceSurfaceIndex, ComparedNextSourceSurfaceIndex
					);
					
					UE_VLOG_WIRESPHERE(World, VLogName, Display, StartLocN0, AgentRadius, HitColor, TEXT_EMPTY);
					UE_VLOG_WIRESPHERE(World, VLogName, Display, EndLocN1, AgentRadius, HitColor, TEXT_EMPTY);
				}
#endif
				
				// we cannot remove N1 point if there is blocking geometry between N0 and N1
				if (bPassedLineOfSightCheckN1)
				{
					// see if there is enough points for a N2 check
					if (ComparedNextSourceSurfaceIndex + 1 < SurfacePoints.Num() - 1)
					{
						// check if there is any blocking collisions between N0 and N2 point
						const auto& RawSurfaceHitN2 = SurfacePoints[ComparedNextSourceSurfaceIndex + 1];
						
						FHitResult HitResultN2;
						
						const FVector StartLocN2 = RawSurfaceHit.HitLocation + (RawSurfaceHit.HitNormal * (AgentRadius + 5) );
						const FVector EndLocN2 = RawSurfaceHitN2.HitLocation + (RawSurfaceHitN2.HitNormal * (AgentRadius + 5));
				
						const bool bHitN2 = World->LineTraceSingleByProfile(
							HitResultN2, 
							StartLocN2, EndLocN2, 
							Settings->BlockSurfaceCollisionProfile.Name, 
							CollisionQueryParams
						);
						
						bDebugDidLineTestN2 = true;
						const bool bPassedLineOfSightCheckN2 = !bHitN2;
						bDebugBlockedN2 = bHitN2;
						
#if ENABLE_VISUAL_LOG
						{
							const FColor HitColor = bHitN2 ? FColor::Red : FColor::Green;
							
							UE_VLOG_SEGMENT_THICK(World, VLogName, Display,
								StartLocN2, EndLocN2, 
								HitColor, 5, TEXT("[%i] vs [%i]"), ComparedNextSourceSurfaceIndex, ComparedNextSourceSurfaceIndex + 1
							);
							
							UE_VLOG_WIRESPHERE(World, VLogName, Display, EndLocN2, AgentRadius, HitColor, TEXT_EMPTY);
						}
#endif
						
						// we cannot remove N1 point if there is a blocking collision between N0 and N2
						if (bPassedLineOfSightCheckN2)
						{
							
#if ENABLE_VISUAL_LOG
							DebugResults(true);
#endif
							
							SurfacePoints.RemoveAt(ComparedNextSourceSurfaceIndex);
							
							// keep same source point, go next
							DebugSameSourcePointCounter++;
							
							continue;
						}
					}
					else
					{
						
#if ENABLE_VISUAL_LOG
						DebugResults(true);
#endif
					
						SurfacePoints.RemoveAt(ComparedNextSourceSurfaceIndex);
					
						// keep same source point, go next
						DebugSameSourcePointCounter++;
					
						continue;
					}
				}
			}
		}
		
#if ENABLE_VISUAL_LOG
		DebugResults(false);
#endif
		
		// we keep the next point, make it source
		ComparedSourceSurfaceIndex++;
		ComparedNextSourceSurfaceIndex++;
#if ENABLE_VISUAL_LOG
		// reset
		DebugSameSourcePointCounter = 1;
#endif
	}
}

void USANAnySurfaceNavLibrary::KeepShortestDistancePoints(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams, 
	float AgentRadius, const TArray<FSANSurfaceHitResult>& InRawSurfaceHits, TArray<FSANSurfaceHitResult>& OutFilteredRawSurfaceHits, const FName VLogName)
{
	using namespace SAN::Library;
	
	TRACE_CPUPROFILER_EVENT_SCOPE(SAN::ShortDistanceFiltering)
	
	OutFilteredRawSurfaceHits.Reserve(InRawSurfaceHits.Num());
	
	TArray<int32> RemovedIndices;
	RemovedIndices.Reserve(InRawSurfaceHits.Num());
	
	// get source point (n)
	// get n+1 and n+2
	// see if dist(n, n+2) is shorter then dist(n, n+1) + dist(n+1, n+2)
	// if yes, remove n+1
	// also check collisions if n+2 distance is shorter because we dont want to skip points if it means going through geometry (since distance math is faster than checking scene collisions)
	
	// TODO: add "air"/"flying" rule: dont remove point even if longer if it causes the agent to "fly"
	
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
			
			// dist(n, n+1) + dist(n+1, n+2)
			const float DefaultDistance = FVector::Dist(SourceRawSurfaceHit.HitLocation, N1RawSurfaceHit.HitLocation) + FVector::Dist(N1RawSurfaceHit.HitLocation, N2RawSurfaceHit.HitLocation);
			
			// dist(n, n+2)
			const float N2Distance = FVector::Dist(SourceRawSurfaceHit.HitLocation, N2RawSurfaceHit.HitLocation);
			
			if (N2Distance < DefaultDistance)
			{
				FHitResult BlockingResult;
				
				const FVector BlockingStartLoc = SourceRawSurfaceHit.HitLocation + (SourceRawSurfaceHit.HitNormal * (AgentRadius + 5) );
				const FVector BlockingEndLoc = N2RawSurfaceHit.HitLocation + (N2RawSurfaceHit.HitNormal * (AgentRadius + 5));
				
				const bool bHit = World->SweepSingleByProfile(
					BlockingResult, 
					BlockingStartLoc, BlockingEndLoc, FQuat::Identity, 
					Settings->BlockSurfaceCollisionProfile.Name, 
					FCollisionShape::MakeSphere(AgentRadius),
					CollisionQueryParams
				);
				
#if ENABLE_VISUAL_LOG
				const FColor DebugHitColor = bHit ? FColor::Red : FColor::Green;
#endif
				
				UE_VLOG_SEGMENT_THICK(World, VLogName, Display,
					BlockingStartLoc, BlockingEndLoc, 
					DebugHitColor, 5, TEXT("[%i] -> [%i]"), SourceIndex, NIndex + 1
				);
				
				UE_VLOG_WIRESPHERE(World, VLogName, Display, BlockingStartLoc, AgentRadius, DebugHitColor, TEXT_EMPTY);
				UE_VLOG_WIRESPHERE(World, VLogName, Display, BlockingEndLoc, AgentRadius, DebugHitColor, TEXT_EMPTY);
				
				// we dont skip N1 point if we have blocking geometry between N0 and N2
				if (!bHit)
				{
					// remove n+1
					RemovedIndices.Emplace(NIndex);
					
					UE_VLOG_LOCATION(World, VLogName, Display, 
						N1RawSurfaceHit.HitLocation, 15, 
						FColor::Red, TEXT("Def dist: %.1f"), DefaultDistance
					);
					
					UE_VLOG_SEGMENT_THICK(World, VLogName, Display, 
						SourceRawSurfaceHit.HitLocation, N1RawSurfaceHit.HitLocation, 
						FColor::Red, 5, TEXT("Dist: %.1f"), FVector::Dist(SourceRawSurfaceHit.HitLocation, N1RawSurfaceHit.HitLocation)
					);
					
					UE_VLOG_SEGMENT_THICK(World, VLogName, Display, 
						N1RawSurfaceHit.HitLocation, N2RawSurfaceHit.HitLocation, 
						FColor::Red, 5, TEXT("Dist: %.1f"), FVector::Dist(N1RawSurfaceHit.HitLocation, N2RawSurfaceHit.HitLocation)
					);
					
					UE_VLOG_SEGMENT_THICK(World, VLogName, Display, 
						SourceRawSurfaceHit.HitLocation, N2RawSurfaceHit.HitLocation, 
						FColor::Green, 7, TEXT("N2 dist [%i->%i]: %.1f"), SourceIndex, NIndex + 1, N2Distance
					);
					
					// go forward (since we are not yet removing the element we move up the chain of points)
					NIndex += 1;
					continue;
				}
			}
			
			// N1 cannot be removed, move up the chain of points
			SourceIndex = NIndex;
			NIndex = SourceIndex + 1;
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
			UE_VLOG_LOCATION(World, VLogName, Display, 
				InRawSurfaceHits[InSurfaceHitsIndex].HitLocation, 10, 
				FColor::Cyan, TEXT("Base Point [%i], \n N: %s"), 
				OutFilteredRawSurfaceHits.Num() - 1, *FU::Utils::PrintCompactVector(InRawSurfaceHits[InSurfaceHitsIndex].HitNormal)
			);
		}
	}
}

bool USANAnySurfaceNavLibrary::IsLineBlocking(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams, 
	float AgentRadius, const FSANSurfaceHitResult& Start, const FSANSurfaceHitResult& End, FHitResult& OutHitResult)
{
	FHitResult Result;
	
	const FVector StartLoc = Start.HitLocation + (Start.HitNormal * (AgentRadius + 5) );
	const FVector EndLoc = End.HitLocation + (Start.HitNormal * (AgentRadius + 5));
	
	const bool bHit = World->SweepSingleByProfile(
		Result, 
		StartLoc, EndLoc, FQuat::Identity, 
		Settings->BlockSurfaceCollisionProfile.Name, 
		FCollisionShape::MakeSphere(AgentRadius),
		CollisionQueryParams
	);
	
	UE_VLOG_SEGMENT_THICK(World, SAN::Library::VLog::IsLineBlocking, VeryVerbose, 
		StartLoc, EndLoc, 
		bHit ? FColor::Red : FColor::Green, AgentRadius, TEXT_EMPTY
	);
	return bHit;
}

bool USANAnySurfaceNavLibrary::IsPointBlocked(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams, float AgentRadius, 
	const FVector& PointLocation, FHitResult& OutHitResult)
{
	FHitResult Result;
	
	const bool bHit = World->SweepSingleByProfile(
		Result, 
		PointLocation, PointLocation, FQuat::Identity, 
		Settings->BlockSurfaceCollisionProfile.Name, 
		FCollisionShape::MakeSphere(AgentRadius),
		CollisionQueryParams
	);
	
	UE_VLOG_SPHERE(World, SAN::Library::VLog::IsPointBlocked, VeryVerbose, 
		PointLocation, AgentRadius, 
		bHit ? FColor::Red : FColor::Green, TEXT_EMPTY
	);
	return bHit;
}

bool USANAnySurfaceNavLibrary::FillGaps(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams,
	float AgentRadius, TArray<FSANSurfaceHitResult>& RawSurfaceHits, TArray<FSANSurfaceHitResult>& SurfaceExtraPoints)
{
	using namespace SAN::Library;
	
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
#if ENABLE_VISUAL_LOG
			constexpr float DebugSphereSize = 20;
#endif
			
			// make subdivisions
			const int32 TotalNbOfSubdivisions = FMath::Floor(DistanceToNextPoint / Settings->GapsMinDistanceBetweenSubdivisions);
			// we skip the "last" point in the subdivision since it will be the next nav point
			const int32 NbOfSubdivisions = TotalNbOfSubdivisions - 1;
			
			if (NbOfSubdivisions > 0)
			{
				// we cannot subdive between 2 points if there is a blocking collision
				FHitResult BlockingResult;
				
				const FVector BlockingStartLoc = RawSurfaceHit.HitLocation + (RawSurfaceHit.HitNormal * (AgentRadius + 5) );
				const FVector BlockingEndLoc = NextRawSurfaceHit.HitLocation + (NextRawSurfaceHit.HitNormal * (AgentRadius + 5));
				
				const bool bHit = World->SweepSingleByProfile(
					BlockingResult, 
					BlockingStartLoc, BlockingEndLoc, FQuat::Identity, 
					Settings->BlockSurfaceCollisionProfile.Name, 
					FCollisionShape::MakeSphere(AgentRadius),
					CollisionQueryParams
				);
				
#if ENABLE_VISUAL_LOG
				const FColor DebugHitColor = bHit ? FColor::Red : FColor::Green;
#endif
				
				UE_VLOG_SEGMENT_THICK(World, VLog::FillGaps, Display,
					BlockingStartLoc, BlockingEndLoc, 
					DebugHitColor, 5, TEXT("[%i] -> [%i]"), RawSurfaceHitIndex, RawSurfaceHitIndex + 1
				);
				
				UE_VLOG_WIRESPHERE(World, VLog::FillGaps, Display, BlockingStartLoc, AgentRadius, DebugHitColor, TEXT_EMPTY);
				UE_VLOG_WIRESPHERE(World, VLog::FillGaps, Display, BlockingEndLoc, AgentRadius, DebugHitColor, TEXT_EMPTY);
				
				if (bHit)
				{
					continue;
				}
				
#if SAN_WITH_DEBUG
				FillGapsLoopCountDebug++;
#endif
				
				bDidChanged = true;
				
				const FColor RandomColor = FU::Colors::PickRandomColor();
				
				const FVector StartLoc = RawSurfaceHit.HitLocation + (RawSurfaceHit.HitNormal * AgentRadius);
				const FVector EndLoc = NextRawSurfaceHit.HitLocation + (NextRawSurfaceHit.HitNormal * AgentRadius);
				
				UE_VLOG_SEGMENT_THICK(World, VLog::FillGaps, Display, 
					StartLoc, EndLoc, 
					RandomColor, 5, TEXT_EMPTY
				);
				
#if SAN_WITH_DEBUG
				UE_VLOG_LOCATION(World, VLog::FillGaps, Display, 
					((StartLoc + EndLoc) / 2) + FVector(0, 0, 60), 0, 
					RandomColor, TEXT("Subdivs [%i, %i] Count: %i"), 
					FillGapsLoopCountDebug, FillGapsLoopCount - 1, NbOfSubdivisions
				);
#endif
				
				for (int32 SbdvIndx = 1; SbdvIndx < NbOfSubdivisions + 1; ++SbdvIndx)
				{
					const FVector SbdvLocation = FMath::Lerp(StartLoc, EndLoc, (float)SbdvIndx/TotalNbOfSubdivisions);
					
					TArray<FSANSurfaceHitResult> BestSurfaceHits;
					float Radius = Settings->SurfaceCollisionSphereMinRadius;
					
					const auto& PreviousSurface = RawSurfaceHit;
					
					if (GetBestSurface(World, Settings, CollisionQueryParams, SbdvLocation, Radius, RawSurfaceHit, BestSurfaceHits))
					{
						SurfaceExtraPoints.Reserve(SurfaceExtraPoints.Num() + BestSurfaceHits.Num() - 1);
						
						UE_VLOG_LOCATION(World, VLog::FillGaps, Display, 
							PreviousSurface.HitLocation, DebugSphereSize / 2, 
							RandomColor, TEXT_EMPTY
						);
						
#if SAN_WITH_DEBUG
						UE_VLOG_LOCATION(World, VLog::FillGaps, Display, 
							PreviousSurface.HitLocation + FVector(0, 0, 30), 0, 
							RandomColor, TEXT("Subdiv PreviousSurface [%i, %i:%i]"), 
							FillGapsLoopCountDebug, FillGapsLoopCount - 1, SbdvIndx - 1
						);
#endif
						
						// inject filling surfaces at the correct indices
						for (int32 InnerSurfaceHitIndx = 0; InnerSurfaceHitIndx < BestSurfaceHits.Num(); ++InnerSurfaceHitIndx)
						{
							// skip first index
							if (InnerSurfaceHitIndx >= 1)
							{
								SurfaceExtraPoints.Emplace(BestSurfaceHits[InnerSurfaceHitIndx]);
							}
							
							RawSurfaceHits.EmplaceAt(RawSurfaceHitIndex + 1, BestSurfaceHits[InnerSurfaceHitIndx]);
							RawSurfaceHitIndex++;
							
#if SAN_WITH_DEBUG
							UE_VLOG_LOCATION(World, VLog::FillGaps, Display, 
								BestSurfaceHits[InnerSurfaceHitIndx].HitLocation, DebugSphereSize, 
								RandomColor, TEXT("Subdiv [%i, %i:%i(%i)] \nN: %s"), 
								FillGapsLoopCountDebug, FillGapsLoopCount - 1, SbdvIndx - 1, InnerSurfaceHitIndx, 
								*FU::Utils::PrintCompactVector(BestSurfaceHits[InnerSurfaceHitIndx].HitNormal)
							);
#endif			
							UE_VLOG_ARROW(World, VLog::FillGaps, Display, 
								BestSurfaceHits[InnerSurfaceHitIndx].HitLocation, 
								BestSurfaceHits[InnerSurfaceHitIndx].HitLocation + BestSurfaceHits[InnerSurfaceHitIndx].HitNormal * 50, 
								RandomColor, TEXT_EMPTY
							);
						}
					}
				}
			}
		}
	}
	
	// if we didnt change anything we can stop here
	if (bDidChanged)
	{
		// TODO: instead of iterating again all the surfaces, just iterate between given range
		return FillGaps(World, Settings, CollisionQueryParams, AgentRadius, RawSurfaceHits, SurfaceExtraPoints);
	}
	else
	{
		return true;
	}
}

int32 USANAnySurfaceNavLibrary::FindBestInBetweenIndex(const FSANSurfaceHitResult& NewSurface, const TArray<FSANSurfaceHitResult>& Surfaces)
{
	float BestDistSq = TNumericLimits<float>::Max();
	int32 BestIndex = -1;
	
	for (int32 i = 0; i < Surfaces.Num() - 1; ++i)
	{
		FVector Closest = FMath::ClosestPointOnSegment(
			NewSurface.HitLocation,
			Surfaces[i].HitLocation,
			Surfaces[i + 1].HitLocation);
		
		float DistSq = FVector::DistSquared(NewSurface.HitLocation, Closest);
		
		if (DistSq < BestDistSq)
		{
			BestDistSq = DistSq;
			BestIndex = i;
		}
	}
	
	return BestIndex >= 0 ? BestIndex : Surfaces.Num();
}

void USANAnySurfaceNavLibrary::MakeCollisionQueryParamsFromRequest(const FSANFindPathRequest& Request, FCollisionQueryParams& Params)
{
	Params.bTraceComplex = Request.bTraceComplex;
	Params.AddIgnoredActors(Request.ActorsToIgnore);
	Params.AddIgnoredComponents(Request.ComponentsToIgnore);
}

int32 USANAnySurfaceNavLibrary::SmoothSegment(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams, float AgentRadius, 
	const FSANSurfaceHitResult& StartSurface, const FSANSurfaceHitResult& EndSurface, TArray<FSANSurfaceHitResult>& OutNewSurfaces)
{
	int32 OutNewSurfacesNum = 0;
	
	using namespace SAN::Library;
	
	// TODO: move to settings
	constexpr float SubdivisionDistance = 30;
	
	const float DistanceBetweenPoints = FVector::Distance(StartSurface.HitLocation, EndSurface.HitLocation);
	
	// make subdivisions
	const int32 TotalNbOfSubdivisions = FMath::Floor(DistanceBetweenPoints / SubdivisionDistance);
	// we skip the "last" point in the subdivision since it will be the next nav point
	const int32 NbOfSubdivisions = TotalNbOfSubdivisions - 1;
	
#if ENABLE_VISUAL_LOG
	const FColor RandDebugColor = FU::Colors::PickRandomColor();
#endif
	
	if (NbOfSubdivisions > 0)
	{
		// we cannot subdive between 2 points if there is a blocking collision so we first check for that
		FHitResult BlockingResult;
		
		const FVector BlockingStartLoc = StartSurface.HitLocation + (StartSurface.HitNormal * (AgentRadius + 5) );
		const FVector BlockingEndLoc = EndSurface.HitLocation + (EndSurface.HitNormal * (AgentRadius + 5));
		
		const bool bHit = World->SweepSingleByProfile(
			BlockingResult, 
			BlockingStartLoc, BlockingEndLoc, FQuat::Identity, 
			Settings->BlockSurfaceCollisionProfile.Name, 
			FCollisionShape::MakeSphere(AgentRadius),
			CollisionQueryParams
		);
		
#if ENABLE_VISUAL_LOG
		const FColor DebugHitColor = bHit ? FColor::Red : FColor::Green;
#endif
		
		UE_VLOG_SEGMENT_THICK(World, VLog::SmoothPass_BlockingTest, Display,
			BlockingStartLoc, BlockingEndLoc, 
			DebugHitColor, 5, TEXT_EMPTY
		);
		
		UE_VLOG_WIRESPHERE(World, VLog::SmoothPass_BlockingTest, Display, BlockingStartLoc, AgentRadius, DebugHitColor, TEXT_EMPTY);
		UE_VLOG_WIRESPHERE(World, VLog::SmoothPass_BlockingTest, Display, BlockingEndLoc, AgentRadius, DebugHitColor, TEXT_EMPTY);
		
		// there is a blocking collision, end here
		if (bHit)
		{
			return 0;
		}
		
		UE_VLOG_SEGMENT_THICK(World, VLog::SmoothPass, Display, 
			StartSurface.HitLocation, EndSurface.HitLocation, 
			RandDebugColor, 10, TEXT("[%i] Subdivs: %i"), VLog::DebugSmoothLoopCount, NbOfSubdivisions
		);
		
		// for each subdivision see if we are hitting a surface
		for (int32 SubdivIndx = 1; SubdivIndx < NbOfSubdivisions + 1; ++SubdivIndx)
		{
			FHitResult HitResult;
			const FVector SbdvLocation = FMath::Lerp(StartSurface.HitLocation, EndSurface.HitLocation, (float)SubdivIndx/TotalNbOfSubdivisions);
			// TODO: check if the point is INSIDE geomtry, if yes we need to place points outside
			if (IsPointBlocked(World, Settings, CollisionQueryParams, AgentRadius, SbdvLocation, HitResult))
			{
				// we are colliding with a surface so it's a correct point for agent nav
				
				// TODO: maybe snap nearby point to hit surface
			}
			else
			{
				UE_VLOG_LOCATION(World, VLog::SmoothPass, Display, 
					SbdvLocation + FVector(0, 0, 10), 0, 
					RandDebugColor, TEXT("[%i]"), SubdivIndx
				);
				
				// if nothing collides it means that the agent will be "floating" in the air which isnt wanted since we always want the agent to use surfaces
				// so we trace "down" to find a in-between surface
				
				// TODO: allow the agent to "jump" between gaps
				
				TArray<FSANSurfaceHitResult> BestSurfaceHits;
				// set initial radius
				// TODO: move to settings
				float Radius = 50;
				constexpr float MaxAxisDiff = 5;
				
				if (GetBestSurface(World, Settings, CollisionQueryParams, SbdvLocation, Radius, StartSurface, BestSurfaceHits))
				{
					UE_VLOG_WIRESPHERE(World, VLog::SmoothPass, Display, 
						SbdvLocation, Radius, 
						FColor::Orange, TEXT("[%i:%i]"), VLog::DebugSmoothLoopCount, SubdivIndx
					);
					
					// clear points that have a big difference in more than 1 axis
					
					for (int32 HitIndx = BestSurfaceHits.Num() - 1; HitIndx >= 0; --HitIndx)
					{
						const auto& BestSurfaceHit = BestSurfaceHits[HitIndx];
						
						int32 AxisCountDiff = 0;
						if (FMath::Abs(SbdvLocation.X - BestSurfaceHit.HitLocation.X) > MaxAxisDiff)
						{
							AxisCountDiff++;
						}
						if (FMath::Abs(SbdvLocation.Y - BestSurfaceHit.HitLocation.Y) > MaxAxisDiff)
						{
							AxisCountDiff++;
						}
						if (FMath::Abs(SbdvLocation.Z - BestSurfaceHit.HitLocation.Z) > MaxAxisDiff)
						{
							AxisCountDiff++;
						}
						
						if (AxisCountDiff > 1)
						{
							UE_VLOG_SPHERE(World, VLog::SmoothPass, Display, 
								BestSurfaceHit.HitLocation, 10, 
								FColor::Black, TEXT_EMPTY
							);
							BestSurfaceHits.RemoveAt(HitIndx);
						}
					}
					
					// store the found surfaces and process them all afterwards since some found surfaces may not be between current start and end
					OutNewSurfaces.Append(BestSurfaceHits);
					OutNewSurfacesNum += BestSurfaceHits.Num();
					
#if ENABLE_VISUAL_LOG
					for (int32 HitIndx = 0; HitIndx < BestSurfaceHits.Num(); ++HitIndx)
					{
						const auto& BestSurfaceHit = BestSurfaceHits[HitIndx];
						
						UE_VLOG_SPHERE(World, VLog::SmoothPass, Display, 
							BestSurfaceHit.HitLocation, 10, 
							RandDebugColor, TEXT("[%i:%i:%i]"), VLog::DebugSmoothLoopCount, SubdivIndx, HitIndx
						);
					}
#endif
				}
			}
		}
	}
	else
	{
		UE_VLOG_SEGMENT_THICK(World, VLog::SmoothPass, Display, 
			StartSurface.HitLocation, EndSurface.HitLocation, 
			RandDebugColor, 5, TEXT("Cannot subdiv")
		);
	}
	
	return OutNewSurfacesNum;
}

void USANAnySurfaceNavLibrary::RemoveSimilarPointsInArrays(const TArray<FSANSurfaceHitResult>& InContainer, TArray<FSANSurfaceHitResult>& FilteredContainer)
{
	for (auto It = FilteredContainer.CreateIterator(); It; ++It)
	{
		if (InContainer.Contains(*It))
		{
			It.RemoveCurrent();
		}
	}
}

void USANAnySurfaceNavLibrary::CheckAndClearUnusablePoints(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams, float AgentRadius, TArray<FSANSurfaceHitResult>& Points)
{
	using namespace SAN::Library;
	
	int32 CurrentIndex = 0;
	while (CurrentIndex < Points.Num() - 2)
	{
		const FSANSurfaceHitResult& Surface = Points[CurrentIndex];
		const FSANSurfaceHitResult& NextSurface = Points[CurrentIndex + 1];
		const FSANSurfaceHitResult& N2Surface = Points[CurrentIndex + 2];
		
		UE_VLOG_SPHERE(World, VLog::SmoothPassBlockingFiltering, Display,
			Surface.HitLocation, AgentRadius, FColor::Green, TEXT("[%i]"), CurrentIndex
		);
		
		FHitResult BlockingResult;
		
		const FVector BlockingStartLoc = Surface.HitLocation + (Surface.HitNormal * (AgentRadius + 5) );
		const FVector BlockingEndLoc = NextSurface.HitLocation + (NextSurface.HitNormal * (AgentRadius + 5));
		
		const bool bHit = World->SweepSingleByProfile(
			BlockingResult, 
			BlockingStartLoc, BlockingEndLoc, FQuat::Identity, 
			Settings->BlockSurfaceCollisionProfile.Name, 
			FCollisionShape::MakeSphere(AgentRadius),
			CollisionQueryParams
		);
		
#if ENABLE_VISUAL_LOG
		const FColor DebugHitColor = bHit ? FColor::Red : FColor::Green;
#endif
		
		UE_VLOG_SEGMENT_THICK(World, VLog::SmoothPassBlockingFiltering, Display,
			BlockingStartLoc, BlockingEndLoc, 
			DebugHitColor, 5, TEXT("[%i] -> [%i]"), CurrentIndex, CurrentIndex + 1
		);
		
		UE_VLOG_WIRESPHERE(World, VLog::SmoothPassBlockingFiltering, Display, BlockingStartLoc, AgentRadius, DebugHitColor, TEXT_EMPTY);
		UE_VLOG_WIRESPHERE(World, VLog::SmoothPassBlockingFiltering, Display, BlockingEndLoc, AgentRadius, DebugHitColor, TEXT_EMPTY);
		
		if (bHit)
		{
			// we can only remove N+1 point if there is no blocking connection with the future N+0 -> N+2
			FHitResult BlockingResultN2;
			
			const FVector BlockingStartLocN2 = Surface.HitLocation + (Surface.HitNormal * (AgentRadius + 5) );
			const FVector BlockingEndLocN2 = N2Surface.HitLocation + (N2Surface.HitNormal * (AgentRadius + 5));
			
			const bool bHitN2 = World->SweepSingleByProfile(
				BlockingResultN2, 
				BlockingStartLocN2, BlockingEndLocN2, FQuat::Identity, 
				Settings->BlockSurfaceCollisionProfile.Name, 
				FCollisionShape::MakeSphere(AgentRadius),
				CollisionQueryParams
			);
			
			if (!bHitN2)
			{
				UE_VLOG_SPHERE(World, VLog::SmoothPassBlockingFiltering, Display,
					NextSurface.HitLocation, AgentRadius, FColor::Red, TEXT("[%i]"), CurrentIndex + 1
				);
				
				// remove next point
				Points.RemoveAt(CurrentIndex + 1);
				
				continue;
			}
		}
		CurrentIndex++;
	}
}
