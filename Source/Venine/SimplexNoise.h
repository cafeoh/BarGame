// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "SimplexNoise.generated.h"

/**
 * 
 */
UCLASS()
class VENINE_API USimplexNoise : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()

public:

	USimplexNoise();

	UFUNCTION(BlueprintCallable, Category = "Noise")
	static float SimplexNoise(FVector Position, float Scale = 1., bool bTurbulence = true, int32 Levels = 6, float OutputMin = -1., float OutputMax = 1., float LevelScale = 2., float FilterWidth = 0.);
	
private:

	static const uint32 NoiseTextureSize = 128;
	static uint32 NoiseTexture[NoiseTextureSize][NoiseTextureSize];
	static bool NoiseTextureGenerated;

	static FVector Texture2DSampleLevel(FVector2D In);
	static FVector GetPerlinNoiseGradientTextureAt(FVector v);
	static float SimplexNoise3D_TEX(FVector EvalPos);
};
