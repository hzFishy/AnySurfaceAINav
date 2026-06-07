// By hzFishy - 2026 - Do whatever you want with it.

#include "Tests/SANTestPathfindingActor.h"

#include "Core/SANAnySurfaceNavLibrary.h"
#include "Data/SANAnySurfaceNavSettings.h"
#include "Draw/FUDraw.h"
#include "Pathfinding/Core/Nav3DPathLibrary.h"

	
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
    
	TArray<FNavPathPoint> Points;
	UNav3DPathLibrary::FindNav3DPathExtended(GetWorld(), StartLocation, EndLocation, AgentRadius, Points);
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
	USANAnySurfaceNavLibrary::FindAnySurfacePath(Request, Result);
}
