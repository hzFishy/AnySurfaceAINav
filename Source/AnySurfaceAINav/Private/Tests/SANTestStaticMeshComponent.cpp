// By hzFishy - 2026 - Do whatever you want with it.

#include "Tests/SANTestStaticMeshComponent.h"


USANTestStaticMeshComponent::USANTestStaticMeshComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}

void USANTestStaticMeshComponent::BeginPlay()
{
	Super::BeginPlay();
}

bool USANTestStaticMeshComponent::IsNavigationRelevant() const
{
	return true;
}

FBox USANTestStaticMeshComponent::GetNavigationBounds() const
{
	return Super::GetNavigationBounds();
}

void USANTestStaticMeshComponent::GetNavigationData(FNavigationRelevantData& Data) const
{
	Super::GetNavigationData(Data);
}
