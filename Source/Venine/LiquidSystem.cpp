// Fill out your copyright notice in the Description page of Project Settings.

#include "LiquidSystem.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "DrawDebugHelpers.h"

#define LOG(format, ...) UE_LOG(LogTemp, Log, TEXT(format), __VA_ARGS__)
#define LOGW(format, ...) UE_LOG(LogTemp, Warning, TEXT(format), __VA_ARGS__)
#define LOGE(format, ...) UE_LOG(LogTemp, Error, TEXT(format), __VA_ARGS__)

UWorld *World = NULL;

// Get all edges (bottom -> top) sorted by their bottom edge, from bottom to top
static TArray<TTuple<FVector, FVector, FVector>> GetZSortedTriangles(TMap<uint32, FVector> &TransformVertices, FIndexArrayView &IndexBuffer){

	TArray<TTuple<FVector, FVector, FVector>> Result;
	Result.Reserve(IndexBuffer.Num()/3);

	for (uint32 i = 0; i < (uint32)IndexBuffer.Num(); i += 3) {

		FVector A = TransformVertices[IndexBuffer[i]];
		FVector B = TransformVertices[IndexBuffer[i + 1]];
		FVector C = TransformVertices[IndexBuffer[i + 2]];
		FVector Temp;

		if (A.Z > B.Z){
			Temp = B;
			B = A;
			A = Temp;
		}

		if (B.Z > C.Z) {
			Temp = C;
			C = B;
			B = Temp;
		}

		if (A.Z > B.Z) {
			Temp = B;
			B = A;
			A = Temp;
		}

		Result.Add(TTuple<FVector, FVector, FVector>(A,B,C));
	}

	Result.Sort([](auto A, auto B){return A[0].Z < B.Key.Z;});

	return Result;
}

// Get all edges (bottom -> top) sorted by their bottom edge, from bottom to top
static TArray<TPair<FVector, FVector>> GetZSortedEdges(TMap<uint32, FVector> &TransformVertices, FIndexArrayView &IndexBuffer) {
	TArray<TTuple<uint8, uint8, uint8>> Edges;

	for (uint32 i = 0; i < (uint32)IndexBuffer.Num(); i += 3) {

		uint8 A = IndexBuffer[i];
		uint8 B = IndexBuffer[i + 1];
		uint8 C = IndexBuffer[i + 2];

		if (A < B)	Edges.Add(TPair<uint8, uint8>(A, B));
		else		Edges.Add(TPair<uint8, uint8>(B, A));

		if (B < C)	Edges.Add(TPair<uint8, uint8>(B, C));
		else		Edges.Add(TPair<uint8, uint8>(C, B));

		if (A < C)	Edges.Add(TPair<uint8, uint8>(A, C));
		else		Edges.Add(TPair<uint8, uint8>(C, A));
	}

	TArray<TPair<FVector, FVector>> Result;
	Result.Reserve(Edges.Num());

	for (auto Edge : Edges) {
		FVector A = TransformVertices[Edge.Key];
		FVector B = TransformVertices[Edge.Value];
		if (A.Z < B.Z) {
			Result.Add(TPair<FVector, FVector>(A, B));
		}
		else {
			Result.Add(TPair<FVector, FVector>(B, A));
		}
	}

	Result.Sort([](TPair<FVector, FVector> A, TPair<FVector, FVector> B) {return A.Key.Z < B.Key.Z; });

	return Result;
}

// Get all vertices in a slice and order them clockwise around the average
static TArray<FVector> GetSliceVertices(TArray<TPair<FVector, FVector>> &SortedEdges, float Height, FVector &Center, bool Draw=false){
	TArray<FVector> Vertices;
	Center = FVector::ZeroVector;

	for (auto Edge : SortedEdges) {
		FVector A = Edge.Key;
		FVector B = Edge.Value;

		if (A.Z >= Height) break;
		if (B.Z < Height) continue;

		if (Draw) {
			DrawDebugLine(World, Edge.Key, Edge.Value, FColor::Red, true, 0, 0, 0.03);
		}

		float Alpha = (Height - A.Z) / (B.Z - A.Z);

		FVector V = FMath::Lerp(A, B, Alpha);

		Center = ((Center * Vertices.Num()) + V) / (Vertices.Num() + 1);

		Vertices.Add(V);
	}

	Vertices.Sort([Center](FVector A, FVector B) { return (Center - A).Rotation().Yaw < (Center - B).Rotation().Yaw; });

	return Vertices;
}

// Get area for a slice at given height
static float GetSliceArea(TArray<TPair<FVector, FVector>> &SortedEdges, float Height, bool Draw = false, FColor Color = FColor::Red, float LineSize=0.03){

	FVector Center;
	TArray<FVector> Vertices = GetSliceVertices(SortedEdges, Height, Center);

	float Area = 0;

	for (uint32 i = 0; i < (uint32) Vertices.Num(); i++) {
		FVector A = Vertices[i]-Center;
		FVector B = Vertices[(i+1)%Vertices.Num()] - Center;

		Area += A.X*B.Y - B.X*A.Y;

		if(Draw){
			DrawDebugLine(World, Vertices[i], Vertices[(i+1)%Vertices.Num()], Color, true, 0, 0, LineSize);
		}
	}

	return Area / 2.;
}


// Get horizontal span of flat slice (distance between two furthest points)
static TPair<FVector, FVector> GetSliceSpan(TArray<TPair<FVector, FVector>> &SortedEdges, float Height, bool Draw = false) {

	FVector Center;
	TArray<FVector> Vertices = GetSliceVertices(SortedEdges, Height, Center);

	if(Vertices.Num()<2) return TPair<FVector, FVector> (0.,0.);

	FVector A=Vertices[0];
	FVector B=Vertices[0];

	float tempDist=0;

	// Find first outside vertice
	for (auto V : Vertices){
		float newDist = (V - B).Size();

		if(newDist > tempDist){
			A = V;
			tempDist = newDist;
		}
	}

	tempDist=0;

	// Find furthest vertice from the first
	for (auto V : Vertices) {
		float newDist = (V - A).Size();

		if (newDist > tempDist) {
			B = V;
			tempDist = newDist;
		}
	}

	if (Draw) {
		DrawDebugLine(World, A, B, FColor::Red, true, 0, 0, 0.03);
	}

	return TPair<FVector, FVector>(A,B);
}

void GetBuffers(UStaticMeshComponent *StaticMeshComponent, TArray<FVector> &Vertices, TArray<TPair<FVector, FVector>> &Edges, TPair<float, float> &ZBounds, FVector PlaneNormal = FVector::UpVector){

	if (!StaticMeshComponent) return;
	if (!StaticMeshComponent->IsValidLowLevel()) return;
	if (!StaticMeshComponent->GetStaticMesh()) return;
	if (!StaticMeshComponent->GetStaticMesh()->RenderData) return;
	if (StaticMeshComponent->GetStaticMesh()->RenderData->LODResources.Num() <= 0) return;

	UStaticMesh *StaticMesh = StaticMeshComponent->GetStaticMesh();
	AActor *Owner = StaticMeshComponent->GetOwner();

	World = Owner->GetWorld();

	FVector RotationPlaneNormal = FVector::CrossProduct(PlaneNormal, FVector::UpVector); // Maybe revert vectors?
	RotationPlaneNormal.Normalize();
	float RotationAngle = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(PlaneNormal,FVector::UpVector)));

	FPositionVertexBuffer* VertexBuffer = &StaticMesh->RenderData->LODResources[0].PositionVertexBuffer;
	FIndexArrayView IndexBuffer = StaticMesh->RenderData->LODResources[0].IndexBuffer.GetArrayView();
	LOGE("Size of AIB : %d",StaticMesh->RenderData->LODResources[0].AdjacencyIndexBuffer.GetArrayView().Num());

	TMap<uint32, FVector> VerticeMap;
	VerticeMap.Reserve(VertexBuffer->GetNumVertices());

	TSet<FVector> VerticeSet;

	if (VertexBuffer) {
		for (uint32 Index = 0; Index < VertexBuffer->GetNumVertices(); Index++) {
			FVector Vertex = Owner->GetActorLocation() + Owner->GetActorTransform().TransformVector(VertexBuffer->VertexPosition(Index)).RotateAngleAxis(RotationAngle, RotationPlaneNormal);
			VerticeMap.Add(Index, Vertex);
			VerticeSet.Add(Vertex);

			if (Vertex.Z < ZBounds.Key || Index == 0)
				ZBounds.Key = Vertex.Z;
			if (Vertex.Z > ZBounds.Value || Index == 0)
				ZBounds.Value = Vertex.Z;
		}
	}

	Edges = GetZSortedEdges(VerticeMap, IndexBuffer);

	Vertices = VerticeSet.Array();

	//VerticeMap.GenerateValueArray(Vertices);
	Vertices.Sort([](FVector A, FVector B) {return A.Z < B.Z; });
}

float ULiquidSystem::GetSlicedExitArea(UStaticMeshComponent *StaticMeshComponent, FVector PlanePosition, FVector PlaneNormal) {
	float Area = 0;

	if (StaticMeshComponent) {
		World = StaticMeshComponent->GetOwner()->GetWorld();
	} else {
		return Area;
	}

	TArray<FVector> Vertices;
	TArray<TPair<FVector, FVector>> Edges;
	TPair<float, float> ZBounds;

	GetBuffers(StaticMeshComponent, Vertices, Edges, ZBounds, PlaneNormal);

	FVector RotationPlaneNormal = FVector::CrossProduct(PlaneNormal, FVector::UpVector); // Maybe revert vectors?
	RotationPlaneNormal.Normalize();
	float RotationAngle = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(PlaneNormal, FVector::UpVector)));

	PlanePosition = (PlanePosition - StaticMeshComponent->GetComponentLocation()).RotateAngleAxis(RotationAngle, RotationPlaneNormal) + StaticMeshComponent->GetComponentLocation();

	if(Vertices.Num() < 2) return 0;

	//TPair<FVector, FVector> PreviousSpan = (Vertices[0],Vertices[0]);
	float BaseLength = 0.;
	FVector BasePoint = Vertices[0];
	float PreviousHeight = ZBounds.Key;
	FVector Center;

	FVector OrthoPlanePosition = BasePoint;
	FVector OrthoPlaneNormal = FVector::CrossProduct(StaticMeshComponent->GetUpVector(),FVector::UpVector);

	for (int32 i=0 ; i<Vertices.Num() ; i++){
		if (Vertices[i].Z > PlanePosition.Z){
			Vertices[i] = PlanePosition;
			Vertices.SetNum(i+1);
			break;
		}
	}

	for (auto Vertex : Vertices) {

		TPair<FVector, FVector> Span = GetSliceSpan(Edges, Vertex.Z);

		if (Span == TPair<FVector,FVector>(FVector::ZeroVector,FVector::ZeroVector)){
			continue;
		}

		if(Span.Key == Span.Value){
			Span.Value = Span.Key + OrthoPlaneNormal;
		}

		FVector TopPoint = FMath::LinePlaneIntersection(Span.Key, Span.Value, OrthoPlanePosition, OrthoPlaneNormal);
		float TopLength = (Span.Key-Span.Value).Size();
		float Height = (TopPoint-BasePoint).Size();

		Area += Height*(BaseLength+TopLength)/2.;

		BasePoint = TopPoint;
		BaseLength = TopLength;
		

		//LOGE("%f Area at height %f (%f)",Area, Vertex.Z);
	}

	/*float TargetVolume = Volume*Alpha;
	float ReturnHeight = ZBounds.Value;

	for (uint32 i = 0; i<(uint32)SlicedVolume.Num(); i++) {
		if (SlicedVolume[i].Value > TargetVolume && i>0) {
			float a = (TargetVolume - SlicedVolume[i - 1].Value) / (SlicedVolume[i].Value - SlicedVolume[i - 1].Value);
			ReturnHeight = FMath::Lerp(SlicedVolume[i - 1].Key, SlicedVolume[i].Key, a);
			break;
		}
	}

	Result = FMath::LinePlaneIntersection(StaticMeshComponent->GetComponentLocation(), StaticMeshComponent->GetComponentLocation() + StaticMeshComponent->GetUpVector(), FVector(0., 0., ReturnHeight), FVector::UpVector);*/

	return Area;
}

float FrustumVolume(float BaseArea, float TopArea, float Height){
	return (Height / 3.) * (TopArea + BaseArea + FMath::Sqrt(TopArea*BaseArea));
}


FVector ULiquidSystem::GetVolumetricSlicingPlane(UStaticMeshComponent *StaticMeshComponent, float Alpha, FVector PlaneNormal){

	FVector Result;

	if (StaticMeshComponent){
		World = StaticMeshComponent->GetOwner()->GetWorld();
	}else{
		return Result;
	}

	TArray<FVector> Vertices;
	TArray<TPair<FVector, FVector>> Edges;
	TPair<float, float> ZBounds;

	GetBuffers(StaticMeshComponent, Vertices, Edges, ZBounds, PlaneNormal);

	TArray<TPair<float,float>> SlicedVolume;
	SlicedVolume.Reserve(Vertices.Num());

	float Volume = 0;
	float PreviousArea = 0;
	float PreviousHeight = ZBounds.Key;

	for(auto Vertex : Vertices){
		float Height = Vertex.Z;
		float SliceArea = GetSliceArea(Edges, Height, true);
		LOGW("Area at Height %f : %f (Vertex %s)", Height-ZBounds.Key, SliceArea, *Vertex.ToString());
		//Volume += ((Height - PreviousHeight) / 3.) * (SliceArea + PreviousArea + FMath::Sqrt(SliceArea*PreviousArea));
		Volume += FrustumVolume(PreviousArea,SliceArea,Height-PreviousHeight);

		SlicedVolume.Add(TPair<float,float>(Height,Volume));

		PreviousArea = SliceArea;
		PreviousHeight = Height;

	}

	float TargetVolume = Volume*Alpha;
	float ReturnHeight = ZBounds.Value;

	LOGE("Volume : %f",Volume);

	for(uint32 i=0 ; i<(uint32)SlicedVolume.Num() ; i++){
		if(SlicedVolume[i].Value > TargetVolume && i>0){
			float ResidualVolume = TargetVolume - SlicedVolume[i-1].Value;
			float SectionVolume = SlicedVolume[i].Value - SlicedVolume[i-1].Value;

			float BottomArea = GetSliceArea(Edges, SlicedVolume[i-1].Key, true, FColor::Yellow, .1);
			float TopArea = GetSliceArea(Edges, SlicedVolume[i].Key, true, FColor::Green, .1);

			float CubicAlpha = (TopArea-BottomArea) / (TopArea>BottomArea?TopArea:BottomArea);
			float Alpha = ResidualVolume/SectionVolume;

			//LOGE("vvv %f vvv",Alpha);

			if(CubicAlpha>=0){
				//Alpha = FMath::Lerp(Alpha,1-FMath::Pow(1-Alpha,3.),CubicAlpha);
			}else{
				//Alpha = FMath::Lerp(FMath::Pow(Alpha, 2.)/2 + Alpha/2, Alpha, 1+CubicAlpha);
			}
			//LOGE("%f -> %f",CubicAlpha,Alpha);

			//LOGW("Error : %f / %f",FMath::Lerp(SlicedVolume[i-1].Value,SlicedVolume[i].Value,Alpha),TargetVolume);


			//float a = (TargetVolume - SlicedVolume[i - 1].Value)/(SlicedVolume[i].Value - SlicedVolume[i - 1].Value);
			ReturnHeight = FMath::Lerp(SlicedVolume[i - 1].Key, SlicedVolume[i].Key, Alpha);
			GetSliceArea(Edges, ReturnHeight, true, FColor::Blue, .1);
			break;
		}
	}

	FVector RotationPlaneNormal = FVector::CrossProduct(PlaneNormal, FVector::UpVector);
	RotationPlaneNormal.Normalize();
	float RotationAngle = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(PlaneNormal, FVector::UpVector)));

	Result = FMath::LinePlaneIntersection(StaticMeshComponent->GetComponentLocation(), StaticMeshComponent->GetComponentLocation() + StaticMeshComponent->GetUpVector(), FVector(0., 0., ReturnHeight), FVector::UpVector);
	Result = (Result - StaticMeshComponent->GetComponentLocation()).RotateAngleAxis(-RotationAngle, RotationPlaneNormal) + StaticMeshComponent->GetComponentLocation();

	//DrawDebugPoint(World, Result, 10, FColor::Green, true, 0, 0);
	//DrawDebugLine(World, Result, OldResult, FColor::Red, true, 0, 0, 0.03);

	return Result;
}