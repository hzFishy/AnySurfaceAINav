// By hzFishy - 2026 - Do whatever you want with it.

#pragma once

#include "GameFramework/Actor.h"
#include "SANTestPathfindingActor.generated.h"
class USANAnySurfaceNavSettings;


UCLASS()
class ANYSURFACEAINAV_API ASANTestPathfindingActor : public AActor
{
	GENERATED_BODY()

	
	/*----------------------------------------------------------------------------
		Properties
	----------------------------------------------------------------------------*/
protected:
	UPROPERTY(EditAnywhere, Category="SAN")
	float AgentRadius;
	
	UPROPERTY(EditAnywhere, Category="SAN")
	TSoftObjectPtr<AActor> DestinationActor;
	
	
	/*----------------------------------------------------------------------------
		Defaults
	----------------------------------------------------------------------------*/
public:
	ASANTestPathfindingActor();
	
	
	/*----------------------------------------------------------------------------
		Core
	----------------------------------------------------------------------------*/
public:
	UFUNCTION(CallInEditor, Category="SAN")
	void Test3DPathfinding();
	
	UFUNCTION(CallInEditor, Category="SAN")
	void TestAnySurfacePathfinding();
};
