// By hzFishy - 2026 - Do whatever you want with it.

#pragma once

#include "GameFramework/Actor.h"
#include "SANTestPathfindingPawn.generated.h"


/** 
 *  It is named with "Pawn" but doesn't depend on the APawn class.
 *  The important part is having a USANCrawlerMovementComponent component which moves a USceneComponent.
 */
UCLASS()
class ANYSURFACEAINAV_API ASANTestPathfindingPawn : public AActor
{
	GENERATED_BODY()

	
	/*----------------------------------------------------------------------------
		Properties
	----------------------------------------------------------------------------*/
protected:
	UPROPERTY(VisibleAnywhere)
	TObjectPtr<USANCrawlerMovementComponent> MovementComponent;
	
	UPROPERTY(EditAnywhere, Category="SAN")
	TSoftObjectPtr<AActor> DestinationActor;
	
	
	/*----------------------------------------------------------------------------
		Defaults
	----------------------------------------------------------------------------*/
public:
	ASANTestPathfindingPawn();
	
	
	/*----------------------------------------------------------------------------
		Core
	----------------------------------------------------------------------------*/
public:
	UFUNCTION(BlueprintCallable, Category="SAN")
	void TestMoveToAnySurfacePath();
};
