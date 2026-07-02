// By hzFishy - 2026 - Do whatever you want with it.

#include "Core/SANAnySurfaceNavLibrary.h"
#include "CPathVolume.h"
#include "Data/SANAnySurfaceNavSettings.h"
#include "Draw/FUColors.h"
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
	
	namespace VLog
	{
		inline FName RawCPathResult = "SANFindAnySurfacePathSync_1_Raw";
		inline FName BasePass = "SANFindAnySurfacePathSync_2_BasePass";
		inline FName BasePassReordered = "SANFindAnySurfacePathSync_2.1_BasePassReordered";
		inline FName BasePassSimilarPointsFiltering = "SANFindAnySurfacePathSync_2.2_SimilarPointsFiltering";
		inline FName ShortPathFiltering = "SANFindAnySurfacePathSync_3_ShortPathFiltering";
		inline FName FillGaps = "SANFindAnySurfacePathSync_4_FillGaps";
		inline FName FillGapsSimilarPointsFiltering = "SANFindAnySurfacePathSync_4.1_SimilarPointsFiltering";
		inline FName SmoothPass = "SANFindAnySurfacePathSync_5_SmoothPass";
		inline FName SmoothPassSimilarPointsFiltering = "SANFindAnySurfacePathSync_5.1_SimilarPointsFiltering";
		inline FName FinalPath = "SANFindAnySurfacePathSync_6_FinalPath";
		
		inline FName IsLineBlocking = "SANFindAnySurfacePathSync_IsLineBlocking";
		inline FName IsPointBlocked = "SANFindAnySurfacePathSync_IsPointBlocked";
	}
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
			
			UE_VLOG_LOCATION(World, VLog::RawCPathResult, Display,
				CPathNode.WorldLocation, 15,
				FColor::Magenta, TEXT("Raw Point [%i]"), i
			);
			
			if (i < CPathResult.UserPath.Num() - 1)
			{
				UE_VLOG_SEGMENT_THICK(World, VLog::RawCPathResult, Display, 
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
	}
	// clear before further use
	PreviousSurface.Reset();
	
	
	// remove points that are to close with similar normal
	RemoveSimilarPoints(World, AnySurfaceNavSettings, BasePassSurfacePoints, VLog::BasePassSimilarPointsFiltering);
	
	// from generated points find shortest paths
	TArray<FSANSurfaceHitResult> BasePassShortenSurfacePoints;
	{
		TRACE_CPUPROFILER_EVENT_SCOPE(SAN::ShortDistanceFiltering)
#if SAN_WITH_DEBUG
		if (SAN::Library::Debug::DebugDisableShortPathFiltering == 0)
		{
			KeepShortestDistancePoints(World, AnySurfaceNavSettings, CollisionQueryParams, Request.AgentRadius, BasePassSurfacePoints, BasePassShortenSurfacePoints);
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
	PreviousSurface.Reset();
	
	// filling gaps: try to clean the path if there is a big distance between found surfaces points
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
			const bool bFilledGaps = FillGaps(World, AnySurfaceNavSettings, CollisionQueryParams, Request.AgentRadius, BasePassShortenSurfacePoints, BasePassSurfaceExtraPoints);
		}
#else 
		const bool bFilledGaps = FillGaps(World, AnySurfaceNavSettings, CollisionQueryParams, PreviousSurface, BasePassShortenSurfacePoints, BasePassSurfaceExtraPoints);
#endif
		
		// remove points that are to close with similar normal
		RemoveSimilarPoints(World, AnySurfaceNavSettings, BasePassShortenSurfacePoints, VLog::FillGapsSimilarPointsFiltering);
		
		// fill in the points in-between
		for (auto& BasePassSurfaceExtraPoint : BasePassSurfaceExtraPoints)
		{
			const int32 Index = FindBestInBetweenIndex(BasePassSurfaceExtraPoint, BasePassShortenSurfacePoints);
			BasePassShortenSurfacePoints.EmplaceAt(Index + 1, BasePassSurfaceExtraPoint);
		}
	}
	
	// extra: smooth the path by blending with the environment
	{
		TArray<FSANSurfaceHitResult> FoundNewSurfaces;
		for (int32 SurfaceIndex = 0; SurfaceIndex < BasePassShortenSurfacePoints.Num() - 1; ++SurfaceIndex)
		{
			const auto& CurrentSurface = BasePassShortenSurfacePoints[SurfaceIndex];
			const auto& NextSurface = BasePassShortenSurfacePoints[SurfaceIndex + 1];
			SmoothSegment(World, AnySurfaceNavSettings, CollisionQueryParams, Request.AgentRadius, CurrentSurface, NextSurface, FoundNewSurfaces);
		}
		
		// fill in the points in-between
		for (auto& NewSurface : FoundNewSurfaces)
		{
			const int32 Index = FindBestInBetweenIndex(NewSurface, BasePassShortenSurfacePoints);
			BasePassShortenSurfacePoints.EmplaceAt(Index + 1, NewSurface);
		}
		
		// TODO: maybe instead only filter FoundNewSurfaces
		// TODO: need to check for line of sight before removing points
		// remove points that are to close with similar normal
		//RemoveSimilarPoints(World, AnySurfaceNavSettings, BasePassShortenSurfacePoints, VLog::SmoothPassSimilarPointsFiltering);
	}
	
	// save results
	Result.SurfaceHitResults = BasePassShortenSurfacePoints;
	
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
		
		if (SAN::Library::Debug::DebugDisplayFindAnySurfacePath)
		{
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

void USANAnySurfaceNavLibrary::RemoveSimilarPoints(UWorld* World, const USANAnySurfaceNavSettings* Settings, TArray<FSANSurfaceHitResult>& SurfacePoints, const FName VLogName)
{
	// TODO: instead of just removing a similar point thats n+1, remove the ones that makes the path the less "straight"
	
	TRACE_CPUPROFILER_EVENT_SCOPE(SAN::ShortDistanceFiltering)
	
#if SAN_WITH_DEBUG
	if (SAN::Library::Debug::DebugDisableDistanceNormalFiltering > 0)
	{
		return;
	}
#endif
	
	int32 ComparedSourceSurfaceIndex = 0;
	int32 ComparedNextSourceSurfaceIndex = 1;
	while (ComparedSourceSurfaceIndex < SurfacePoints.Num() - 1)
	{
		const auto& RawSurfaceHit = SurfacePoints[ComparedSourceSurfaceIndex];
		const auto& NextRawSurfaceHit = SurfacePoints[ComparedNextSourceSurfaceIndex];
		
		// check distance
		const float DistanceToNextPoint = FVector::Dist(RawSurfaceHit.HitLocation, NextRawSurfaceHit.HitLocation);
		if (DistanceToNextPoint <= Settings->CleanUpPathPointDistanceThreshold)
		{
			const float NormalDiff = FVector::DotProduct(RawSurfaceHit.HitNormal, NextRawSurfaceHit.HitNormal);
			if (DistanceToNextPoint <= Settings->CleanUpPathPointDistanceHardThreshold || NormalDiff >= Settings->CleanUpPathPointNormalThreshold)
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
				
				UE_VLOG_LOCATION(World, VLogName, Display, 
					NextRawSurfaceHit.HitLocation, 25, 
					FColor::Red, TEXT("Removed [previously %i]"), ComparedNextSourceSurfaceIndex
				);
				
				SurfacePoints.RemoveAt(ComparedNextSourceSurfaceIndex);
				
				// keep same source point, go next
				continue;
			}
		}
		
		UE_VLOG_LOCATION(World, VLogName, Display, 
			RawSurfaceHit.HitLocation, 15, 
			FColor::Green, TEXT("[%i]"), ComparedSourceSurfaceIndex
		);
		
		// we keep the next point, make it source
		ComparedSourceSurfaceIndex++;
		ComparedNextSourceSurfaceIndex = ComparedSourceSurfaceIndex + 1;
	}
}

void USANAnySurfaceNavLibrary::KeepShortestDistancePoints(UWorld* World, const USANAnySurfaceNavSettings* Settings, 
	const FCollisionQueryParams& CollisionQueryParams, float AgentRadius, const TArray<FSANSurfaceHitResult>& InRawSurfaceHits, 
	TArray<FSANSurfaceHitResult>& OutFilteredRawSurfaceHits)
{
	using namespace SAN::Library;
	
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
			
			// dist(n, n+1) + dist(n+1, n+2)
			const float DefaultDistance = FVector::Dist(SourceRawSurfaceHit.HitLocation, N1RawSurfaceHit.HitLocation) + FVector::Dist(N1RawSurfaceHit.HitLocation, N2RawSurfaceHit.HitLocation);
			
			// dist(n, n+2)
			const float N2Distance = FVector::Dist(SourceRawSurfaceHit.HitLocation, N2RawSurfaceHit.HitLocation);
			
			FHitResult HitResult;
			if (N2Distance < DefaultDistance 
				&& !IsLineBlocking(World, Settings, CollisionQueryParams, AgentRadius, SourceRawSurfaceHit, N2RawSurfaceHit, HitResult))
			{
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
				
				UE_VLOG_LOCATION(World, VLog::ShortPathFiltering, Display, 
					N1RawSurfaceHit.HitLocation, 15, 
					FColor::Red, TEXT("Def total dist: %.1f"), DefaultDistance
				);
				
				UE_VLOG_SEGMENT_THICK(World, VLog::ShortPathFiltering, Display, 
					SourceRawSurfaceHit.HitLocation, N1RawSurfaceHit.HitLocation, 
					FColor::Red, 5, TEXT("Dist: %.1f"), FVector::Dist(SourceRawSurfaceHit.HitLocation, N1RawSurfaceHit.HitLocation)
				);
				
				UE_VLOG_SEGMENT_THICK(World, VLog::ShortPathFiltering, Display, 
					N1RawSurfaceHit.HitLocation, N2RawSurfaceHit.HitLocation, 
					FColor::Red, 5, TEXT("Dist: %.1f"), FVector::Dist(N1RawSurfaceHit.HitLocation, N2RawSurfaceHit.HitLocation)
				);
				
				UE_VLOG_SEGMENT_THICK(World, VLog::ShortPathFiltering, Display, 
					SourceRawSurfaceHit.HitLocation, N2RawSurfaceHit.HitLocation, 
					FColor::Green, 7, TEXT("N2 dist: %.1f"), N2Distance
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
			UE_VLOG_LOCATION(World, VLog::ShortPathFiltering, Display, 
				InRawSurfaceHits[InSurfaceHitsIndex].HitLocation, 10, 
				FColor::Cyan, TEXT("Base Point [%i], \n N: %s"), OutFilteredRawSurfaceHits.Num() - 1, *FU::Utils::PrintCompactVector(InRawSurfaceHits[InSurfaceHitsIndex].HitNormal)
			);
		}
	}
}

bool USANAnySurfaceNavLibrary::IsLineBlocking(UWorld* World, const USANAnySurfaceNavSettings* Settings, 
	const FCollisionQueryParams& CollisionQueryParams, float AgentRadius, const FSANSurfaceHitResult& Start, 
	const FSANSurfaceHitResult& End, FHitResult& OutHitResult)
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
	
	UE_VLOG_SEGMENT_THICK(World, SAN::Library::VLog::IsLineBlocking, Display, 
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
	
	UE_VLOG_SPHERE(World, SAN::Library::VLog::IsPointBlocked, Display, 
		PointLocation, AgentRadius, 
		bHit ? FColor::Red : FColor::Green, TEXT_EMPTY
	);
	return bHit;
}

bool USANAnySurfaceNavLibrary::FillGaps(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams, float AgentRadius, TArray<FSANSurfaceHitResult>& RawSurfaceHits, TArray<FSANSurfaceHitResult>& SurfaceExtraPoints)
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
			
			const float DebugSphereSize = 20;
			
			// make subdivisions
			const int32 TotalNbOfSubdivisions = FMath::Floor(DistanceToNextPoint / Settings->GapsMinDistanceBetweenSubdivisions);
			// we skip the "last" point in the subdivision since it will be the next nav point
			const int32 NbOfSubdivisions = TotalNbOfSubdivisions - 1;
			
			if (NbOfSubdivisions > 0)
			{
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

void USANAnySurfaceNavLibrary::SmoothSegment(UWorld* World, const USANAnySurfaceNavSettings* Settings, const FCollisionQueryParams& CollisionQueryParams, float AgentRadius, 
	const FSANSurfaceHitResult& StartSurface, const FSANSurfaceHitResult& EndSurface, TArray<FSANSurfaceHitResult>& OutNewSurfaces)
{
	using namespace SAN::Library;
	
	// check if collision between current point and next point for all using subdivisions
	
	// TODO: move to settings
	constexpr float SubdivisionDistance = 30;
	
	const float DistanceBetweenPoints = FVector::Distance(StartSurface.HitLocation, EndSurface.HitLocation);
	
	// make subdivisions
	const int32 TotalNbOfSubdivisions = FMath::Floor(DistanceBetweenPoints / SubdivisionDistance);
	// we skip the "last" point in the subdivision since it will be the next nav point
	const int32 NbOfSubdivisions = TotalNbOfSubdivisions - 1;
	
	const FColor RandDebugColor = FU::Colors::PickRandomColor();
	
	if (NbOfSubdivisions > 0)
	{
		UE_VLOG_SEGMENT_THICK(World, VLog::SmoothPass, Display, 
			StartSurface.HitLocation, EndSurface.HitLocation, 
			RandDebugColor, 10, TEXT("Subdivs: %i"), NbOfSubdivisions
		);
		
		for (int32 SubdivIndx = 1; SubdivIndx < NbOfSubdivisions + 1; ++SubdivIndx)
		{
			FHitResult HitResult;
			const FVector SbdvLocation = FMath::Lerp(StartSurface.HitLocation, EndSurface.HitLocation, (float)SubdivIndx/TotalNbOfSubdivisions);
			if (IsPointBlocked(World, Settings, CollisionQueryParams, AgentRadius, SbdvLocation, HitResult))
			{
				// TODO: maybe snap nearby point to hit surface
				
				// we are colliding, check other points (nothing to do here for now)
			}
			else
			{
				// if nothing collides it means that the agent will be "floating" which isnt wanted
				// so we trace "down" to find a in-between surface
				
				TArray<FSANSurfaceHitResult> BestSurfaceHits;
				float Radius = Settings->SurfaceCollisionSphereMinRadius;
				
				const FSANSurfaceHitResult& PreviousSurface = StartSurface;
				if (GetBestSurface(World, Settings, CollisionQueryParams, SbdvLocation, Radius, PreviousSurface, BestSurfaceHits))
				{
					// store the found surfaces and process them all afterwards since some found surfaces may not be between current start and end
					OutNewSurfaces.Append(BestSurfaceHits);
					
#if ENABLE_VISUAL_LOG
					for (auto& BestSurfaceHit : BestSurfaceHits)
					{
						UE_VLOG_SPHERE(World, SAN::Library::VLog::SmoothPass, Display, 
							BestSurfaceHit.HitLocation, 20, 
							RandDebugColor, TEXT_EMPTY
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
}
