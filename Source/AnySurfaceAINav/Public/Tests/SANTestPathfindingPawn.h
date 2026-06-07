// By hzFishy - 2026 - Do whatever you want with it.

#pragma once

#include "Data/SANSurfaceTypes.h"
#include "GameFramework/Pawn.h"
#include "SANTestPathfindingPawn.generated.h"
class USplineComponent;

// TODO: move movement logic to movement component
UCLASS()
class ANYSURFACEAINAV_API ASANTestPathfindingPawn : public APawn
{
	GENERATED_BODY()

	
	/*----------------------------------------------------------------------------
		Properties
	----------------------------------------------------------------------------*/
protected:
	/** cm/s */
	UPROPERTY(EditAnywhere, Category="SAN")
	float MovementSpeed;
	
	UPROPERTY(EditAnywhere, Category="SAN")
	float AgentRadius;
	
	UPROPERTY(EditAnywhere, Category="SAN")
	TSoftObjectPtr<AActor> DestinationActor;
	
	//////////////////////////////////
	// Runtime
	UPROPERTY()
	TObjectPtr<USplineComponent> PathSplineComponent;
	
	bool bProcessRequest;
	
	FSANFindPathResult CachedFindPathResult;
	
	
	/*----------------------------------------------------------------------------
		Defaults
	----------------------------------------------------------------------------*/
public:
	ASANTestPathfindingPawn();
	
	virtual void PostInitializeComponents() override;
	
	virtual void BeginPlay() override;
	
	virtual void Tick(float DeltaSeconds) override;
	
	
	/*----------------------------------------------------------------------------
		Core
	----------------------------------------------------------------------------*/
public:
	void TestMoveToAnySurfacePath();
	
protected:
	/** Copy from UActorFactory::FindActorAlignmentRotation */
	FQuat FindActorAlignmentRotation(const FQuat& InActorRotation, const FVector& InModelAxis, const FVector& InWorldNormal);
};
