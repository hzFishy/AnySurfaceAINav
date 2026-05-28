// Copyright Iron City Dev Team. All Rights Reserved.


#include "Tests/SANTestPrimitiveComponent.h"


USANTestPrimitiveComponent::USANTestPrimitiveComponent()
{
	PrimaryComponentTick.bCanEverTick = false;
}


void USANTestPrimitiveComponent::BeginPlay()
{
	Super::BeginPlay();
}

bool USANTestPrimitiveComponent::IsNavigationRelevant() const
{
	return Super::IsNavigationRelevant();
}

FBox USANTestPrimitiveComponent::GetNavigationBounds() const
{
	return Super::GetNavigationBounds();
}

void USANTestPrimitiveComponent::GetNavigationData(FNavigationRelevantData& Data) const
{
	Super::GetNavigationData(Data);
}
