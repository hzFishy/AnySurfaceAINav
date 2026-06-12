// By hzFishy - 2026 - Do whatever you want with it.


#include "Tests/SANTestPathfindingPawn.h"
#include "Components/SplineComponent.h"
#include "Core/SANAnySurfaceNavLibrary.h"
#include "Draw/FUDraw.h"
#include "Utility/FUUtilities.h"

	
	/*----------------------------------------------------------------------------
		Defaults
	----------------------------------------------------------------------------*/
ASANTestPathfindingPawn::ASANTestPathfindingPawn():
	MovementSpeed(200),
	bProcessRequest(false)
{
	PrimaryActorTick.bCanEverTick = true;
}

void ASANTestPathfindingPawn::PostInitializeComponents()
{
	Super::PostInitializeComponents();
	
#if WITH_EDITOR
	FU_UTILS_EDITOR_RETURN_NOTGAMEWORLD
#endif
}

void ASANTestPathfindingPawn::BeginPlay()
{
	Super::BeginPlay();
	
	TestMoveToAnySurfacePath();
}

void ASANTestPathfindingPawn::Tick(float DeltaSeconds)
{
	Super::Tick(DeltaSeconds);
	
	if (!bProcessRequest) { return; }
	
	// TODO: detect finished path
	
	//const FVector Direction = PathSplineComponent->GetDirectionAtTime(CurrentElapsedTime, ESplineCoordinateSpace::World, true);
	//const FVector TargetLocation = PathSplineComponent->GetLocationAtTime(CurrentElapsedTime, ESplineCoordinateSpace::World, true);
	
	//const float CurrentPointTime = PathSplineComponent->GetInputKeyValueAtTime(CurrentElapsedTime);
	
	//const auto& Surface = CachedFindPathResult.SurfaceHitResults[FMath::Floor(CurrentPointTime)];
	
	/*FU::Draw::DrawDebugSphere(
		GetWorld(),
		Surface.HitLocation,
		10,
		FColor::Red,
		10
	);
	
	SetActorLocation(TargetLocation);*/
	// TODO: rotate actor so it matches wall normal (lerp smooth between points) and direction
	//SetActorRotation(FindActorAlignmentRotation(GetActorRotation().Quaternion(), FVector(0.f, 0.f, 1.f), Surface.HitNormal));
}

	
	/*----------------------------------------------------------------------------
		Core
	----------------------------------------------------------------------------*/
void ASANTestPathfindingPawn::TestMoveToAnySurfacePath()
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
	if (USANAnySurfaceNavLibrary::FindAnySurfacePath(Request, Result))
	{
		// start move
		//bProcessRequest = true;
		CachedFindPathResult = Result;
		Result.SurfacesToPositions(CachedPositions, AgentRadius);
	}
}

FQuat ASANTestPathfindingPawn::FindActorAlignmentRotation(const FQuat& InActorRotation, const FVector& InModelAxis, const FVector& InWorldNormal)
{
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
