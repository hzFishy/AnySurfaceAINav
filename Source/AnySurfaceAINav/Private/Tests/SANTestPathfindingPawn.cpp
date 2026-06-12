// By hzFishy - 2026 - Do whatever you want with it.


#include "Tests/SANTestPathfindingPawn.h"
#include "Core/SANCrawlerMovementComponent.h"
#include "Utility/FUUtilities.h"

	
	/*----------------------------------------------------------------------------
		Defaults
	----------------------------------------------------------------------------*/
ASANTestPathfindingPawn::ASANTestPathfindingPawn()
{
	PrimaryActorTick.bCanEverTick = false;
	
	MovementComponent = CreateDefaultSubobject<USANCrawlerMovementComponent>("MovementComponent");
}

	
	/*----------------------------------------------------------------------------
		Core
	----------------------------------------------------------------------------*/
void ASANTestPathfindingPawn::TestMoveToAnySurfacePath()
{
	if (!DestinationActor.IsValid()) { return; }
	
	const FVector StartLocation = GetActorLocation();
	const FVector EndLocation = DestinationActor->GetActorLocation();
	
	MovementComponent->RequestPathFollowFromTo(StartLocation, EndLocation);
}
