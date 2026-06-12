// By hzFishy - 2026 - Do whatever you want with it.

#pragma once

#include "Components/ActorComponent.h"
#include "Data/SANMovementPathRequest.h"
#include "SANCrawlerMovementComponent.generated.h"


/** 
 *  Handles the movement of a given USceneComponent along a any surface path result.
 *  It doesn't do anything to handle collisions since the any surface pathfinding algo takes collisions into account.
 *  
 *  TODO: dynamicly update the path if the 3D nav grid is updated close to the cached path result points.
 */
UCLASS(ClassGroup="SAN", DisplayName="Crawler Movement Component", meta=(BlueprintSpawnableComponent))
class ANYSURFACEAINAV_API USANCrawlerMovementComponent : public UActorComponent
{
	GENERATED_BODY()

	
	/*----------------------------------------------------------------------------
		Properties
	----------------------------------------------------------------------------*/
protected:
	/** cm/s */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SAN|Movement")
	float MaxMovementSpeed;
	
	/** Acceleration applied by input (rate of change of velocity) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SAN|Movement")
	float Acceleration;

	/** Deceleration applied when there is no input (rate of change of velocity) */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SAN|Movement")
	float Deceleration;
	
	/**
	 * Setting affecting extra force applied when changing direction, making turns have less drift and become more responsive.
	 * Velocity magnitude is not allowed to increase, that only happens due to normal acceleration. It may decrease with large direction changes.
	 * Larger values apply extra force to reach the target direction more quickly, while a zero value disables any extra turn force.
	 */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SAN|Movement", meta=(ClampMin="0", UIMin="0"))
	float TurningBoost;
	
	/** Optional agent radius to use in given request */
	UPROPERTY(EditAnywhere, Category="SAN|Agent")
	TOptional<float> AgentRadiusOverride;
	
	UPROPERTY(EditAnywhere, Category="SAN|Agent")
	float DistanceOverlap;
	
	
	/////////////////////////////
	/// Runtime
protected:
	EMoveComponentFlags MovementFlags;
	
	UPROPERTY(BlueprintReadOnly)
	TObjectPtr<USceneComponent> RootComponent;
	
	FVector MovementVelocity;
	
	FSANMovementPathRequest CurrentRequest;
	
	bool bProcessingPathRequest;
	
	int32 CurrentMoveIndex;
	
	
	/*----------------------------------------------------------------------------
		Defaults
	----------------------------------------------------------------------------*/
public:
	USANCrawlerMovementComponent();

	virtual void InitializeComponent() override;

public:
	virtual void TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction) override;
	
	
	/*----------------------------------------------------------------------------
		Core
	----------------------------------------------------------------------------*/
public:
	UFUNCTION(BlueprintCallable, Category="SAN|Movement")
	void SetRootComponent(USceneComponent* NewRootComponent);
	
	UFUNCTION(BlueprintCallable, Category="SAN|Movement")
	void RequestPathFollow(FSANFindPathRequest Request);
	
	UFUNCTION(BlueprintCallable, Category="SAN|Movement")
	void RequestPathFollowFromTo(const FVector& StartLocation, const FVector& EndLocation, float AgentRadius = -1);
	
	UFUNCTION(BlueprintCallable, Category="SAN|Movement")
	void FollowPath(const FSANMovementPathRequest& Request);
	
protected:
	void ProcessPathRequest(float DeltaTime);
	
	void CalcVelocity(const FVector& Direction, float DeltaTime);
	
	void ApplyVelocity(float DeltaTime);
	
	
	/*----------------------------------------------------------------------------
		Utils
	----------------------------------------------------------------------------*/
protected:
	bool IsExceedingMaxSpeed(float MaxSpeed) const;
	
	bool IsCloseEnoughToLocation(const FVector& Location) const;
	
	FQuat FindActorAlignmentRotation(const FQuat& InActorRotation, const FVector& InModelAxis, const FVector& InWorldNormal);
};
