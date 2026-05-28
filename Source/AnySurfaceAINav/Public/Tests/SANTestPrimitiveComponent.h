// Copyright Iron City Dev Team. All Rights Reserved.

#pragma once

#include "Components/StaticMeshComponent.h"
#include "SANTestPrimitiveComponent.generated.h"


UCLASS(ClassGroup=(Custom), meta=(BlueprintSpawnableComponent))
class ANYSURFACEAINAV_API USANTestPrimitiveComponent : public UStaticMeshComponent
{
	GENERATED_BODY()

public:
	USANTestPrimitiveComponent();
	
protected:
	virtual void BeginPlay() override;
	
	virtual bool IsNavigationRelevant() const override;
	
	virtual FBox GetNavigationBounds() const override;
	
	virtual void GetNavigationData(FNavigationRelevantData& Data) const override;
};
