// Fill out your copyright notice in the Description page of Project Settings.

#include "SimplexNoise.h"
#include "Venine.h"


uint32 USimplexNoise::NoiseTexture[USimplexNoise::NoiseTextureSize][USimplexNoise::NoiseTextureSize];
bool USimplexNoise::NoiseTextureGenerated = false;

inline FVector SkewSimplex(FVector In)
{
	return In + FVector::DotProduct(In, FVector(1.0 / 3.0f));
}

inline FVector UnSkewSimplex(FVector In)
{
	return In - FVector::DotProduct(In, FVector(1.0 / 6.0f));
}

inline float min(float x, float y)
{
	if (x < y) return x;
	else return y;

	//return x < y ? x : y;
}

inline float max(float x, float y)
{
	if (x > y) return x;
	else return y;

	//return x > y ? x : y;
}

float saturate(float x)
{
	return min(max(x, 0.), 1.);
}

inline float length2(FVector v)
{
	return FVector::DotProduct(v,v);
}

/*
FVector2D SkewSimplex(FVector2D In)
{
	float skew = (FMath::Sqrt(3.0f) - 1.0f) * 0.5f;
	return In + FVector2D::DotProduct(In, FVector2D(skew,skew));
}
FVector2D UnSkewSimplex(FVector2D In)
{
	float unskew = (3.0f - FMath::Sqrt(3.0f)) / 6.0f;
	return In - FVector2D::DotProduct(In, FVector2D(unskew,unskew));
}

*/

static FVector4 ComputeSimplexWeights3D(FVector OrthogonalPos, FVector &PosA, FVector &PosB, FVector &PosC, FVector &PosD)
{
	FVector OrthogonalPosFloor = FVector(FMath::FloorToInt(OrthogonalPos.X), FMath::FloorToInt(OrthogonalPos.Y), FMath::FloorToInt(OrthogonalPos.Z));

	PosA = OrthogonalPosFloor;
	PosB = PosA + FVector(1, 1, 1);

	OrthogonalPos -= OrthogonalPosFloor;

	float Largest = OrthogonalPos.GetMax();
	float Smallest = OrthogonalPos.GetMin();

	PosC = PosA + FVector(Largest == OrthogonalPos.X, Largest == OrthogonalPos.Y, Largest == OrthogonalPos.Z);
	PosD = PosA + FVector(Smallest != OrthogonalPos.X, Smallest != OrthogonalPos.Y, Smallest != OrthogonalPos.Z);

	FVector4 ret;

	float RG = OrthogonalPos.X - OrthogonalPos.Y;
	float RB = OrthogonalPos.X - OrthogonalPos.Z;
	float GB = OrthogonalPos.Y - OrthogonalPos.Z;


	ret.Z =
		min(max(0, RG), max(0, RB))			// X
		+ min(max(0, -RG), max(0, GB))		// Y
		+ min(max(0, -RB), max(0, -GB));	// Z

	ret.W =
		min(max(0, -RG), max(0, -RB))		// X
		+ min(max(0, RG), max(0, -GB))		// Y
		+ min(max(0, RB), max(0, GB));		// Z

	ret.Y = Smallest;
	ret.X = 1.0f - ret.Y - ret.Z - ret.W;

	return ret;
}

FVector USimplexNoise::Texture2DSampleLevel(FVector2D In)
{

	int x = fmod(In.X, NoiseTextureSize);
	int y = fmod(In.Y, NoiseTextureSize);

	uint32 lookup = NoiseTexture[x][y];

	int R = (lookup & 0xff0000)/0X10000;
	int G = (lookup & 0x00ff00)/0x100;
	int B = (lookup & 0x0000ff);
	FVector result = FVector(R, G, B)/255.;

	
	//UE_LOG(LogTemp, Log, TEXT("%d:%d:%d"), R, G, B);

	return result;
}

FVector USimplexNoise::GetPerlinNoiseGradientTextureAt(FVector v)
{
	
	const FVector2D ZShear = FVector2D(17, 89);

	FVector2D OffsetA = v.Z * ZShear;
	FVector2D TexA = (FVector2D(v.X,v.Y) + OffsetA + 0.5f) / 128.0f;

	return Texture2DSampleLevel( TexA ) * 2 - 1;
}

float USimplexNoise::SimplexNoise3D_TEX(FVector EvalPos)
{

	FVector OrthogonalPos = SkewSimplex(EvalPos);

	//UE_LOG(LogTemp, Log, TEXT("POSORTH : %s"), *(OrthogonalPos.ToString()));

	FVector PosA, PosB, PosC, PosD;
	FVector4 Weights = ComputeSimplexWeights3D(OrthogonalPos, PosA, PosB, PosC, PosD);



	// can be optimized to 1 or 2 texture lookups (4 or 8 channel encoded in 32 bit)
	FVector A = GetPerlinNoiseGradientTextureAt(PosA);
	FVector B = GetPerlinNoiseGradientTextureAt(PosB);
	FVector C = GetPerlinNoiseGradientTextureAt(PosC);
	FVector D = GetPerlinNoiseGradientTextureAt(PosD);

	PosA = UnSkewSimplex(PosA);
	PosB = UnSkewSimplex(PosB);
	PosC = UnSkewSimplex(PosC);
	PosD = UnSkewSimplex(PosD);

	//UE_LOG(LogTemp, Warning, TEXT("EVALPOS : %s"), *(EvalPos.ToString()));

// 	UE_LOG(LogTemp, Log, TEXT("POSA : %s"), *(PosA.ToString()));
// 	UE_LOG(LogTemp, Log, TEXT("POSB : %s"), *(PosB.ToString()));
// 	UE_LOG(LogTemp, Log, TEXT("POSC : %s"), *(PosC.ToString()));
// 	UE_LOG(LogTemp, Log, TEXT("POSD : %s"), *(PosD.ToString()));

	float DistanceWeight;

	DistanceWeight = saturate(.6f - length2(EvalPos - PosA));	
	DistanceWeight *= DistanceWeight; 
	DistanceWeight *= DistanceWeight;
	float a = FVector::DotProduct(A, EvalPos - PosA) * DistanceWeight;

	DistanceWeight = saturate(0.6f - length2(EvalPos - PosB));
	DistanceWeight *= DistanceWeight; 
	DistanceWeight *= DistanceWeight;
	float b = FVector::DotProduct(B, EvalPos - PosB) * DistanceWeight;

	DistanceWeight = saturate(0.6f - length2(EvalPos - PosC));
	DistanceWeight *= DistanceWeight; 
	DistanceWeight *= DistanceWeight;
	float c = FVector::DotProduct(C, EvalPos - PosC) * DistanceWeight;

	DistanceWeight = saturate(0.6f - length2(EvalPos - PosD));
	DistanceWeight *= DistanceWeight; 
	DistanceWeight *= DistanceWeight;
	float d = FVector::DotProduct(D, EvalPos - PosD) * DistanceWeight;

	float res = 32. * (a + b + c + d);

/*	UE_LOG(LogTemp, Log, TEXT("%f"), res);*/

	return res;
}

float USimplexNoise::SimplexNoise(FVector Position, float Scale, bool bTurbulence, int32 Levels, float OutputMin, float OutputMax, float LevelScale, float FilterWidth) {

	Position *= Scale;
	FilterWidth *= Scale;

	float Out = 0.0f;
	float OutScale = 1.0f;
	float InvLevelScale = 1.0f / LevelScale;

	for (int32 i = 0; i < Levels; ++i)
	{
		// fade out noise level that are too high frequent (not done through dynamic branching as it usually requires gradient instructions)
		OutScale *= saturate(1.0 - FilterWidth);

		if (bTurbulence)
		{
			Out += FMath::Abs(SimplexNoise3D_TEX(Position)) * OutScale;
		}
		else
		{
			Out += SimplexNoise3D_TEX(Position) * OutScale;
		}		

		Position *= LevelScale;
		OutScale *= InvLevelScale;
		FilterWidth *= LevelScale;
	}

	if (!bTurbulence)
	{
		// bring -1..1 to 0..1 range
		Out = Out * 0.5f + 0.5f;
	}

	// Out is in 0..1 range
	return FMath::Lerp(OutputMin, OutputMax, Out);
	
}

USimplexNoise::USimplexNoise() {

	if (NoiseTextureGenerated) {
		return;
	}

	FRandomStream RandomStream(12345);

	// Values represent float3 values in the -1..1 range.
	// The vectors are the edge mid point of a cube from -1 .. 1
	static uint32 gradtable[] =
	{
		0x88ffff, 0xff88ff, 0xffff88,
		0x88ff00, 0xff8800, 0xff0088,
		0x8800ff, 0x0088ff, 0x00ff88,
		0x880000, 0x008800, 0x000088,
	};
	for (int32 y = 0; y < NoiseTextureSize; ++y)
	{
		for (int32 x = 0; x < NoiseTextureSize; ++x)
		{
			USimplexNoise::NoiseTexture[x][y] = gradtable[(uint32)(RandomStream.GetFraction() * 11.9999999f)];
		}
	}

	NoiseTextureGenerated = true;

	/*UE_LOG(LogTemp, Warning, TEXT("TEXTURE CONSTRUCTED"));*/
}
