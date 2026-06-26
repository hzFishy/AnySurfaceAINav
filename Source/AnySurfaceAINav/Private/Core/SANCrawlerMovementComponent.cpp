// By hzFishy - 2026 - Do whatever you want with it.

#include "Core/SANCrawlerMovementComponent.h"
#include "Core/SANAnySurfaceNavLibrary.h"
#include "Data/SANAnySurfaceNavSettings.h"
#include "Core/SANCore.h"
#if SAN_WITH_DEBUG
#include "Draw/FUDraw.h"
#endif


namespace SAN::Movement
{
#if SAN_WITH_DEBUG
	namespace Debug
	{
		FU_CMD_AUTOVAR(DebugDisplayGroundCmd, 
			"SAN.Debug.Movement.DisplayGround", "Show debug data of the ground, increase number for more details",
			int32, DebugDisplayGround, 0
		);
	}
#endif
}
	
	
	/*----------------------------------------------------------------------------
		Defaults
	----------------------------------------------------------------------------*/
USANCrawlerMovementComponent::USANCrawlerMovementComponent():
	bAutoSetRootComponentToOwningActorRoot(true),
	DistanceOverlap(30),
	GroundDetectionDistance(50),
	GroundHeightOffset(5),
	MaxMovementSpeed(300),
	Acceleration(4000),
	Deceleration(8000),
	TurningBoost(8),
	MovementFlags(MOVECOMP_NoFlags),
	bProcessingPathRequest(false),
	CurrentMoveIndex(-1)
{
	PrimaryComponentTick.bCanEverTick = true;
	PrimaryComponentTick.bStartWithTickEnabled = false;
	
	bWantsInitializeComponent = true;
}

void USANCrawlerMovementComponent::InitializeComponent()
{
	Super::InitializeComponent();
	
	AnySurfaceNavSettings = GetDefault<USANAnySurfaceNavSettings>();
	
	if (bAutoSetRootComponentToOwningActorRoot)
	{
		MovingComponent = GetOwner()->GetRootComponent();
	}
}

void USANCrawlerMovementComponent::TickComponent(float DeltaTime, ELevelTick TickType, FActorComponentTickFunction* ThisTickFunction)
{
	if (!IsValid(MovingComponent.Get()))
	{
		return;
	}
	
	ProcessPathRequest(DeltaTime);
	
	// we dont tick unless we have something to process
	if (!bProcessingPathRequest)
	{
		SetComponentTickEnabled(false);
		return;
	}
	
	Super::TickComponent(DeltaTime, TickType, ThisTickFunction);
}

	
	/*----------------------------------------------------------------------------
		Core
	----------------------------------------------------------------------------*/
void USANCrawlerMovementComponent::SetMovingComponent(USceneComponent* NewComponent)
{
	MovingComponent = NewComponent;
}

void USANCrawlerMovementComponent::RequestPathFollow(FSANFindPathRequest Request)
{
	if (AgentRadiusOverride.IsSet())
	{
		Request.AgentRadius = AgentRadiusOverride.GetValue();
	}
	
	FSANFindPathResult FindPathResult;
	USANAnySurfaceNavLibrary::FindAnySurfacePathSync(Request, FindPathResult);
	
	FSANMovementPathRequest MoveRequest;
	MoveRequest.CachedPathRequest = Request;
	MoveRequest.CachedPathResult = FindPathResult;
	FollowPath(MoveRequest);
}

void USANCrawlerMovementComponent::RequestPathFollowFromTo(const FVector& StartLocation, const FVector& EndLocation, float AgentRadius)
{
	FSANFindPathRequest Request;
	Request.WorldContextObject = GetWorld();
	Request.StartLocation = StartLocation;
	Request.EndLocation = EndLocation;
	Request.AgentRadius = AgentRadius;
	
	RequestPathFollow(Request);
}

void USANCrawlerMovementComponent::FollowPath(const FSANMovementPathRequest& Request)
{
	if (!Request.IsValid()) { return; }
	
	CurrentRequest = Request;
	bProcessingPathRequest = true;
	CurrentMoveIndex = -1;
	SetComponentTickEnabled(true);
	
	UE_VLOG_SPHERE(this, "SANCrawlerMovement", Display, Request.CachedPathRequest.StartLocation, 50, FColor::Blue, TEXT("StartLoc"));
	UE_VLOG_LOCATION(this, "SANCrawlerMovement", Display, Request.CachedPathRequest.EndLocation, 50, FColor::Red, TEXT("EndLoc"));
}

bool USANCrawlerMovementComponent::HasValidGround() const
{
	return GroundHitResult.bBlockingHit && GroundHitResult.Component.IsValid();
}

void USANCrawlerMovementComponent::ProcessPathRequest(float DeltaTime)
{
	// get ground info
	QueryGround();
	
	// check if we finished
	if (CurrentMoveIndex + 1 >= CurrentRequest.CachedPathResult.SurfaceHitResults.Num())
	{
		UE_VLOG_LOCATION(this, "SANCrawlerMovement", Display, GetOwner()->GetActorLocation(), 5, FColor::Green, TEXT("Finished"));
		bProcessingPathRequest = false;
		return;
	}
	
	const FVector CurrentLocation = GetOwner()->GetActorLocation();
	const auto& NexPoint = CurrentRequest.CachedPathResult.SurfaceHitResults[CurrentMoveIndex + 1];
	const FVector TargetNextLocation = NexPoint.HitLocation + NexPoint.HitNormal * GroundHeightOffset;
	const FVector Direction = TargetNextLocation - CurrentLocation;
	CalcVelocity(Direction.GetSafeNormal(), DeltaTime);
	
	ApplyVelocityAndRotation(DeltaTime);
	
	UE_VLOG_LOCATION(this, "SANCrawlerMovement", Display, CurrentLocation, 2, FColor::Blue, TEXT("CurrLoc"));
	UE_VLOG_SEGMENT_THICK(this, "SANCrawlerMovement", Warning, 
		CurrentMoveIndex >= 0 ? CurrentRequest.CachedPathResult.SurfaceHitResults[CurrentMoveIndex].HitLocation : CurrentRequest.CachedPathRequest.StartLocation, 
		TargetNextLocation, FColor::Red, 10, TEXT_EMPTY
	);
	
	if (IsCloseEnoughToLocation(TargetNextLocation))
	{
		CurrentMoveIndex++;
	}
}

void USANCrawlerMovementComponent::QueryGround()
{
	GroundHitResult.Reset(0, false);
	
	FCollisionShape Shape = FCollisionShape::MakeSphere(GroundDetectionDistance);
	
	const FVector OriginLocation = MovingComponent->GetComponentLocation();
	
	GetWorld()->SweepSingleByProfile(
		GroundHitResult,
		OriginLocation,
		OriginLocation, 
		FQuat::Identity,
		AnySurfaceNavSettings->BlockSurfaceCollisionProfile.Name,
		Shape
	);
	
#if SAN_WITH_DEBUG
	if (SAN::Movement::Debug::DebugDisplayGround > 0)
	{
		FU::Draw::DrawDebugDirectionalArrowFrame(
			GetWorld(),
			OriginLocation,
			GroundHitResult.ImpactNormal * 150,
			FColor::White,
			2,
			2
		);
		
		if (SAN::Movement::Debug::DebugDisplayGround > 1)
		{
			FU::Draw::DrawDebugSphereFrame(
				GetWorld(),
				OriginLocation,
				GroundDetectionDistance,
				FColor::White,
				2,
				1
			);
		}
	}
#endif
}

void USANCrawlerMovementComponent::CalcVelocity(const FVector& Direction, float DeltaTime)
{
	// Similar to UFloatingPawnMovement::ApplyControlInputToVelocity
	
	const FVector RequestedAcceleration = Direction;
	
	const float MaxPawnSpeed = MaxMovementSpeed;
	const bool bExceedingMaxSpeed = IsExceedingMaxSpeed(MaxPawnSpeed);
	
	if (!bExceedingMaxSpeed)
	{
		// Apply change in velocity direction
		if (MovementVelocity.SizeSquared() > 0.f)
		{
			// Change direction faster than only using acceleration, but never increase velocity magnitude.
			const float TimeScale = FMath::Clamp(DeltaTime * TurningBoost, 0.f, 1.f);
			MovementVelocity = MovementVelocity + (RequestedAcceleration * MovementVelocity.Size() - MovementVelocity) * TimeScale;
		}
	}
	else
	{
		// Dampen velocity magnitude based on deceleration.
		if (MovementVelocity.SizeSquared() > 0.f)
		{
			const FVector OldVelocity = MovementVelocity;
			const float VelSize = FMath::Max(MovementVelocity.Size() - FMath::Abs(Deceleration) * DeltaTime, 0.f);
			MovementVelocity = MovementVelocity.GetSafeNormal() * VelSize;
			
			// Don't allow braking to lower us below max speed if we started above it.
			if (bExceedingMaxSpeed && MovementVelocity.SizeSquared() < FMath::Square(MaxPawnSpeed))
			{
				MovementVelocity = OldVelocity.GetSafeNormal() * MaxPawnSpeed;
			}
		}
	}
	
	// Apply acceleration and clamp velocity magnitude.
	const float NewMaxSpeed = (IsExceedingMaxSpeed(MaxPawnSpeed)) ? MovementVelocity.Size() : MaxPawnSpeed;
	MovementVelocity += RequestedAcceleration * FMath::Abs(Acceleration) * DeltaTime;
	MovementVelocity = MovementVelocity.GetClampedToMaxSize(NewMaxSpeed);
}

void USANCrawlerMovementComponent::ApplyVelocityAndRotation(float DeltaTime)
{
	// Move actor
	FVector Delta = MovementVelocity * DeltaTime;
	
	if (!Delta.IsNearlyZero(1e-6f))
	{
		const FVector OldLocation = MovingComponent->GetComponentLocation();
		FQuat Rotation;
		
		if (HasValidGround())
		{
			Rotation = FindActorAlignmentRotation(
				MovingComponent->GetComponentRotation().Quaternion(), 
				FVector(0, 0, 1), 
				GroundHitResult.ImpactNormal
			);
		}
		else
		{
			Rotation = MovingComponent->GetComponentRotation().Quaternion();
		}
		
		MovingComponent->MoveComponent(Delta, Rotation, false);
		
		// Update velocity
		// We don't want position changes to vastly reverse our direction (which can happen due to penetration fixups etc)
		const FVector NewLocation = MovingComponent->GetComponentLocation();
		MovementVelocity = ((NewLocation - OldLocation) / DeltaTime);
	}
	
	// Finalize
	MovingComponent->ComponentVelocity = MovementVelocity;
}

bool USANCrawlerMovementComponent::IsExceedingMaxSpeed(float MaxSpeed) const
{
	// Similar to UMovementComponent::ApplyControlInputToVelocity
	
	MaxSpeed = FMath::Max(0.f, MaxSpeed);
	const float MaxSpeedSquared = FMath::Square(MaxSpeed);

	// Allow 1% error tolerance, to account for numeric imprecision.
	const float OverVelocityPercent = 1.01f;
	return (MovementVelocity.SizeSquared() > MaxSpeedSquared * OverVelocityPercent);
}

bool USANCrawlerMovementComponent::IsCloseEnoughToLocation(const FVector& Location) const
{
	return FVector::Dist(MovingComponent->GetComponentLocation(), Location) <= DistanceOverlap;
}

	
	/*----------------------------------------------------------------------------
		Utils
	----------------------------------------------------------------------------*/
FQuat USANCrawlerMovementComponent::FindActorAlignmentRotation(const FQuat& InActorRotation, const FVector& InModelAxis, const FVector& InWorldNormal)
{
	// Similar to UActorFactory::FindActorAlignmentRotation
	FVector TransformedModelAxis = InActorRotation.RotateVector(InModelAxis);

	const auto InverseActorRotation = InActorRotation.Inverse();
	const auto DestNormalModelSpace = InverseActorRotation.RotateVector(InWorldNormal);

	FQuat DeltaRotation = FQuat::Identity;

	const double VectorDot = InWorldNormal | TransformedModelAxis;
	if (1.f - FMath::Abs(VectorDot) <= KINDA_SMALL_NUMBER)
	{
		if (VectorDot < 0.f)
		{
			// Anti-parallel
			return InActorRotation * FQuat::FindBetween(InModelAxis, DestNormalModelSpace);
		}
	}
	else
	{
		const FVector Z(0.f, 0.f, 1.f);

		// Find a reference axis to measure the relative pitch rotations between the source axis, and the destination axis.
		FVector PitchReferenceAxis = InverseActorRotation.RotateVector(Z);
		if (FMath::Abs(FVector::DotProduct(InModelAxis, PitchReferenceAxis)) > 0.7f)
		{
			PitchReferenceAxis = DestNormalModelSpace;
		}
		
		// Find a local 'pitch' axis to rotate around
		const FVector OrthoPitchAxis = FVector::CrossProduct(PitchReferenceAxis, InModelAxis);
		const double Pitch = FMath::Acos(PitchReferenceAxis | DestNormalModelSpace) - FMath::Acos(PitchReferenceAxis | InModelAxis);//FMath::Asin(OrthoPitchAxis.Size());

		DeltaRotation = FQuat(OrthoPitchAxis.GetSafeNormal(), Pitch);
		DeltaRotation.Normalize();

		// Transform the model axis with this new pitch rotation to see if there is any need for yaw
		TransformedModelAxis = (InActorRotation * DeltaRotation).RotateVector(InModelAxis);

		const float ParallelDotThreshold = 0.98f; // roughly 11.4 degrees (!)
		if (!FVector::Coincident(InWorldNormal, TransformedModelAxis, ParallelDotThreshold))
		{
			const double Yaw = FMath::Atan2(InWorldNormal.X, InWorldNormal.Y) - FMath::Atan2(TransformedModelAxis.X, TransformedModelAxis.Y);

			// Rotation axis for yaw is the Z axis in world space
			const FVector WorldYawAxis = (InActorRotation * DeltaRotation).Inverse().RotateVector(Z);
			DeltaRotation *= FQuat(WorldYawAxis, -Yaw);
		}
	}

	return InActorRotation * DeltaRotation;
}
