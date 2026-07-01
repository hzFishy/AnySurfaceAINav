// By hzFishy - 2026 - Do whatever you want with it.

#pragma once

#include "Components/ActorComponent.h"
#include "Data/SANMovementPathRequest.h"
#include "SANCrawlerMovementComponent.generated.h"
class USANAnySurfaceNavSettings;


/** 
 *  Handles the movement of a given USceneComponent along a any surface path result.
 *  It doesn't do anything to handle collisions since the any surface pathfinding algo takes collisions into account.
 *  The path is processed each tick, the component doesn't tick if no path are actives
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
	/** If true the root component of the owning actor will be set as the "MovingComponent" on component initialization */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SAN|Agent")
	bool bAutoSetRootComponentToOwningActorRoot;
	
	/** Agent radius to use in given request, if 0 will not be used */
	UPROPERTY(EditAnywhere, Category="SAN|Agent", meta=(ClampMin="0", UIMin="0"))
	float AgentRadius;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SAN|Agent", meta=(ClampMin="0", UIMin="0", Units="cm"))
	float DistanceOverlap;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SAN|Ground", meta=(ClampMin="0", UIMin="0", Units="cm"))
	float GroundDetectionDistance;
	
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SAN|Ground", meta=(ClampMin="0", UIMin="0", Units="cm"))
	float GroundHeightOffset;
	
	/** Speed in cm/s, can be dynamicly changed while already processing a path */
	UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="SAN|Movement", meta=(ClampMin="0", UIMin="0", Units="cm/s"))
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
	
	
	/////////////////////////////
	/// Runtime
protected:
	UPROPERTY()
	TObjectPtr<const USANAnySurfaceNavSettings> AnySurfaceNavSettings;
	
	EMoveComponentFlags MovementFlags;
	
	UPROPERTY(BlueprintReadOnly)
	TObjectPtr<USceneComponent> MovingComponent;
	
	FVector MovementVelocity;
	
	FSANMovementPathRequest CurrentRequest;
	
	bool bProcessingPathRequest;
	
	/** starts at -1 (so me first move from our current location to the first path point) */
	int32 CurrentMoveIndex;
	
	/** Latest ground hit result */
	FHitResult GroundHitResult;
	
	
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
	/** This can be unsafe to change at runtime while a movement is being processed */
	UFUNCTION(BlueprintCallable, Category="SAN|Movement")
	void SetMovingComponent(USceneComponent* NewComponent);
	
	UFUNCTION(BlueprintCallable, Category="SAN|Movement")
	void RequestPathFollow(const FSANFindPathRequest& Request);
	
	UFUNCTION(BlueprintCallable, Category="SAN|Movement")
	void RequestPathFollowFromTo(const FVector& StartLocation, const FVector& EndLocation, float AgentRadiusOverride = -1);
	
	UFUNCTION(BlueprintCallable, Category="SAN|Movement")
	void FollowPath(const FSANMovementPathRequest& Request);
	
	UFUNCTION(BlueprintPure, Category="SAN|Movement")
	bool HasValidGround() const;
	
protected:
	void ProcessPathRequest(float DeltaTime);
	
	void QueryGround();
	
	void CalcVelocity(const FVector& Direction, float DeltaTime);
	
	void ApplyVelocityAndRotation(float DeltaTime);
	
	bool IsPointUnreachable(FVector Location) const;
	
	
	/*----------------------------------------------------------------------------
		Utils
	----------------------------------------------------------------------------*/
protected:
	bool IsExceedingMaxSpeed(float MaxSpeed) const;
	
	bool IsCloseEnoughToLocation(const FVector& Location) const;
	
	FQuat FindActorAlignmentRotation(const FQuat& InActorRotation, const FVector& InModelAxis, const FVector& InWorldNormal);
};
