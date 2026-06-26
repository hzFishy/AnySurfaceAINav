// By hzFishy - 2026 - Do whatever you want with it.

#include "Tests/SANTestPathfindingActor.h"

#include "CPathVolume.h"
#include "Core/SANAnySurfaceNavLibrary.h"
#include "Core/SANCore.h"
#include "Draw/FUDraw.h"
#include "Kismet/GameplayStatics.h"


/*----------------------------------------------------------------------------
		Defaults
	----------------------------------------------------------------------------*/
ASANTestPathfindingActor::ASANTestPathfindingActor():
	AgentRadius(10)
{
	PrimaryActorTick.bCanEverTick = false;
	bIsEditorOnlyActor = true;
}

	
	/*----------------------------------------------------------------------------
		Core
	----------------------------------------------------------------------------*/
void ASANTestPathfindingActor::Test3DPathfinding()
{
	if (!DestinationActor.IsValid()) { return; }
	
	const FVector StartLocation = GetActorLocation();
	const FVector EndLocation = DestinationActor->GetActorLocation();
    
	auto* CPathVolume = Cast<ACPathVolume>(UGameplayStatics::GetActorOfClass(GetWorld(), ACPathVolume::StaticClass()));
	FCPathResult CPathResult = CPathVolume->FindPathSynchronous(StartLocation, EndLocation,  0, 0, 2);
	
#if SAN_WITH_DEBUG
	for (auto& PathPoint : CPathResult.UserPath)
	{
		FU::Draw::DrawDebugSphere(
			GetWorld(),
			PathPoint.WorldLocation,
			20,
			FColor::Green,
			10
		);
	}
#endif
}

void ASANTestPathfindingActor::TestAnySurfacePathfinding()
{
	if (!DestinationActor.IsValid()) { return; }
	
	const FVector StartLocation = GetActorLocation();
	const FVector EndLocation = DestinationActor->GetActorLocation();
    
	FSANFindPathRequest Request;
	Request.WorldContextObject = GetWorld();
	Request.StartLocation = StartLocation;
	Request.EndLocation = EndLocation;
	Request.AgentRadius = AgentRadius;
	
	FSANFindPathResult Result;
	USANAnySurfaceNavLibrary::FindAnySurfacePathSync(Request, Result);
}
