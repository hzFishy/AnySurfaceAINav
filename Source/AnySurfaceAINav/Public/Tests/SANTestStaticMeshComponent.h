// By hzFishy - 2026 - Do whatever you want with it.

#pragma once

#include "Components/StaticMeshComponent.h"
#include "SANTestStaticMeshComponent.generated.h"


UCLASS(ClassGroup="AnySurfaceAINav", meta=(BlueprintSpawnableComponent))
class ANYSURFACEAINAV_API USANTestStaticMeshComponent : public UStaticMeshComponent
{
	GENERATED_BODY()

public:
	USANTestStaticMeshComponent();
	
protected:
	virtual void BeginPlay() override;
	
public:
	virtual bool IsNavigationRelevant() const override;
	
	virtual FBox GetNavigationBounds() const override;
	
	virtual void GetNavigationData(FNavigationRelevantData& Data) const override;
};
