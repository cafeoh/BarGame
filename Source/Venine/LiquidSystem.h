// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "LiquidSystem.generated.h"

/**
 * 
 */
UCLASS()
class VENINE_API ULiquidSystem : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	
	public:

	static TMap<FString, float> MeshVolume;

	static double LastDebugUpdate;

	UFUNCTION(BlueprintCallable, Category = "LiquidSystem")
	static FVector GetVolumetricSlicingPlane(UStaticMeshComponent *StaticMeshComponent, float Alpha, FVector PlaneNormal, bool Debug);
	
	UFUNCTION(BlueprintCallable, Category = "LiquidSystem")
	static float GetSlicedExitArea(UStaticMeshComponent *StaticMeshComponent, FVector PlanePosition, FVector PlaneNormal);
};
