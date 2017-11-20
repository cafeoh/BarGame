// Fill out your copyright notice in the Description page of Project Settings.

#include "LiquidSystem.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "DrawDebugHelpers.h"

#define LOG(format, ...) UE_LOG(LogTemp, Log, TEXT(format), __VA_ARGS__)
#define LOGW(format, ...) UE_LOG(LogTemp, Warning, TEXT(format), __VA_ARGS__)
#define LOGE(format, ...) UE_LOG(LogTemp, Error, TEXT(format), __VA_ARGS__)

UWorld *World = NULL;
AActor *Owner = NULL;
const TArray<FColor>ColorMap = { FColor::Red, FColor::Yellow, FColor::Green, FColor::Cyan, FColor::Blue, FColor::Magenta };

TMap< FString, float > ULiquidSystem::MeshVolume;

// Get all edges (bottom -> top) sorted by their bottom edge, from bottom to top
static TArray<TTuple<FVector, FVector, FVector>> GetZSortedTriangles(TArray<FVector> &TransformedVertices, FIndexArrayView &IndexBuffer){

	TArray<TTuple<FVector, FVector, FVector>> Result;
	Result.Reserve(IndexBuffer.Num()/3);

	for (uint32 i = 0; i < (uint32)IndexBuffer.Num(); i += 3) {

		FVector A = TransformedVertices[IndexBuffer[i]];
		FVector B = TransformedVertices[IndexBuffer[i + 1]];
		FVector C = TransformedVertices[IndexBuffer[i + 2]];
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

	Result.Sort([](auto A, auto B){return A.Get<0>().Z < B.Get<0>().Z;});

	return Result;
}

// Get all edges (bottom -> top) sorted by their bottom edge, from bottom to top
static TArray<TPair<FVector, FVector>> GetZSortedEdges(TMap<uint32, FVector> &TransformVertices, FIndexArrayView &IndexBuffer) {
	TArray<TPair<uint8, uint8>> Edges;

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

/*
#ifdef 0
// Get all vertices in a slice and order them clockwise around the average
static TArray<FVector> GetSliceVertices(TArray<TTuple<FVector, FVector, FVector>> &Triangles, float Height, FVector &Center, bool Draw=false){
	TArray<FVector> Vertices;
	Center = FVector::ZeroVector;

	for (auto Triangle : Triangles) {
		FVector A = Triangle.Get<0>();
		FVector B = Triangle.Get<1>();
		FVector C = Triangle.Get<2>();

		if (A.Z >= Height) break;
		if (C.Z < Height) continue;

		FVector V1 = FMath::Lerp(A, C, (Height - A.Z) / (C.Z - A.Z));
		FVector V2;

		if(Height > B.Z){
			V2 = FMath::Lerp(B, C, (Height - B.Z) / (C.Z - B.Z));
		}else{
			V2 = FMath::Lerp(A, B, (Height - A.Z) / (B.Z - A.Z));
		}

		Center = ((Center * Vertices.Num()) + V) / (Vertices.Num() + 1);

		Vertices.Add(V);
	}

	Vertices.Sort([Center](FVector A, FVector B) { return (Center - A).Rotation().Yaw < (Center - B).Rotation().Yaw; });

	return Vertices;
}
#endif

// Get area for a slice at given height
#ifdef 0
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
#endif
*/

// Get horizontal span of flat slice (distance between two furthest points)
static TTuple<FVector, FVector> GetSliceSpan(TArray<TTuple<FVector, FVector, FVector>> &Triangles, float Height, bool Draw = false) {

	FVector Center;
	TArray<FVector> Vertices;
	Center = FVector::ZeroVector;

	// Get all vertices at height
	for (auto Triangle : Triangles) {
		FVector A = Triangle.Get<0>();
		FVector B = Triangle.Get<1>();
		FVector C = Triangle.Get<2>();

		if (A.Z >= Height) break;
		if (C.Z < Height) continue;

		Vertices.Add(FMath::Lerp(A, C, (Height - A.Z) / (C.Z - A.Z)));

		if (Height > B.Z) {
			Vertices.Add(FMath::Lerp(B, C, (Height - B.Z) / (C.Z - B.Z)));
		} else {
			Vertices.Add(FMath::Lerp(A, B, (Height - A.Z) / (B.Z - A.Z)));
		}
	}

	if(Vertices.Num()<2) return TTuple<FVector, FVector> (0.,0.);

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

	return TTuple<FVector, FVector>(A,B);
}

TTuple<FVector,float> SavedRotationData = TTuple<FVector, float> {FVector::ZeroVector, 0};

FVector RotateVectorWithNormal(FVector V, FVector PlaneNormal = FVector::UpVector){
	if(SavedRotationData.Get<0>().IsZero()){
		FVector RotationPlaneNormal = FVector::CrossProduct(PlaneNormal, FVector::UpVector); // Maybe revert vectors?
		RotationPlaneNormal.Normalize();
		SavedRotationData = TTuple<FVector,float> {RotationPlaneNormal, FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(PlaneNormal, FVector::UpVector)))};
	}
	return Owner->GetActorLocation() + Owner->GetActorTransform().TransformVector(V).RotateAngleAxis(SavedRotationData.Get<1>(), SavedRotationData.Get<0>());
}

FVector UnrotateVectorWithNormal(FVector V, FVector PlaneNormal = FVector::UpVector) {
	if (SavedRotationData.Get<0>().IsZero()) {
		FVector RotationPlaneNormal = FVector::CrossProduct(PlaneNormal, FVector::UpVector); // Maybe revert vectors?
		RotationPlaneNormal.Normalize();
		SavedRotationData = TTuple<FVector, float> { RotationPlaneNormal, FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(PlaneNormal, FVector::UpVector))) };
	}
	return (V - Owner->GetActorLocation()).RotateAngleAxis(SavedRotationData.Get<1>(), -SavedRotationData.Get<0>()) + Owner->GetActorLocation();
	//return Owner->GetActorLocation() + Owner->GetActorTransform().TransformVector(V).RotateAngleAxis(-SavedRotationData.Get<1>(), SavedRotationData.Get<0>());
}

void GetBuffers(UStaticMeshComponent *StaticMeshComponent, TArray<FVector> &Vertices, TArray<TTuple<FVector, FVector, FVector>> &Triangles, TPair<float, float> &ZBounds, FVector PlaneNormal = FVector::UpVector){

	// Get Mesh Info
	if (!StaticMeshComponent) return;
	if (!StaticMeshComponent->IsValidLowLevel()) return;
	if (!StaticMeshComponent->GetStaticMesh()) return;
	if (!StaticMeshComponent->GetStaticMesh()->RenderData) return;
	if (StaticMeshComponent->GetStaticMesh()->RenderData->LODResources.Num() <= 0) return;

	UStaticMesh *StaticMesh = StaticMeshComponent->GetStaticMesh();
	
	Owner = StaticMeshComponent->GetOwner();
	World = Owner->GetWorld();

	// Rotate vertices with plane normal
	/*FVector RotationPlaneNormal = FVector::CrossProduct(PlaneNormal, FVector::UpVector); // Maybe revert vectors?
	RotationPlaneNormal.Normalize();
	float RotationAngle = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(PlaneNormal,FVector::UpVector)));*/

	FPositionVertexBuffer* VertexBuffer = &StaticMesh->RenderData->LODResources[0].PositionVertexBuffer;
	FIndexArrayView IndexBuffer = StaticMesh->RenderData->LODResources[0].IndexBuffer.GetArrayView();

	TSet<FVector> VerticeSet;

	if (VertexBuffer) {
		for (uint32 Index = 0; Index < VertexBuffer->GetNumVertices(); Index++) {
			//FVector Vertex = Owner->GetActorLocation() + Owner->GetActorTransform().TransformVector(VertexBuffer->VertexPosition(Index)).RotateAngleAxis(RotationAngle, RotationPlaneNormal);
			FVector Vertex = RotateVectorWithNormal(VertexBuffer->VertexPosition(Index),PlaneNormal);
			Vertices.Add(Vertex);
			VerticeSet.Add(Vertex);

			if (Vertex.Z < ZBounds.Key || Index == 0)
				ZBounds.Key = Vertex.Z;
			if (Vertex.Z > ZBounds.Value || Index == 0)
				ZBounds.Value = Vertex.Z;
		}
	}

	Triangles = GetZSortedTriangles(Vertices, IndexBuffer);

	Vertices = VerticeSet.Array();

	//VerticeMap.GenerateValueArray(Vertices);
	Vertices.Sort([](FVector A, FVector B) {return A.Z < B.Z; });
}

float PyramidVolume(TArray<FVector> &Vertices, float Height){
	// THIS ONLY GOES FOR FLAT SHAPES WITH Z UP
	float Area = 0;

	for (uint32 i = 0; i < (uint32)Vertices.Num(); i++) {
		FVector A = Vertices[i];
		FVector B = Vertices[(i + 1) % Vertices.Num()];

		Area += A.X*B.Y - B.X*A.Y;
	}
	Area = FMath::Abs(Area / 2.);

	return Area * Height / 3.;
}

float ComputePyramidVolume(TArray<FVector> &Base, FVector Top, bool DrawDebug=false, float Inset=0., FColor Color = FColor::Red){
	
	if(Base.Num() < 3) return 0;

	FPlane BasePlane = FPlane(Base[0], Base[1], Base[2]);

	if(Base.Num() > 3 && BasePlane.IsZero()){
		for (int i = 0; i<Base.Num() && BasePlane.IsZero(); i++) {
			BasePlane = FPlane(Base[i], Base[(i + 1) % Base.Num()], Base[(i + 2) % Base.Num()]);
		}
	}

	FVector BasePoint = FVector::PointPlaneProject(Top, BasePlane);
	float Height = (FVector::PointPlaneProject(Top, BasePlane) - Top).Size();

	// Pyramid of height 0
	if (Height < 0.0001) return 0;

	FVector NewUpVector = BasePlane.GetSafeNormal();
	FVector NewRightVector = FVector::CrossProduct(NewUpVector, FVector::UpVector);
	FVector NewForwardVector;

	if (!NewRightVector.Normalize()) NewRightVector = FVector::RightVector;

	NewForwardVector = FVector::CrossProduct(NewUpVector, NewRightVector);
	NewForwardVector.Normalize();

	TArray<FVector> TransformedBase;
	TransformedBase.Reserve(Base.Num());

	for (auto BaseVertex : Base){
		//FVector TransformedBaseVertex = FVector::ForwardVector * (BaseVertex * NewForwardVector).Size() + FVector::RightVector * (BaseVertex * NewRightVector);
		FVector TransformedBaseVertex = FVector::DotProduct(BaseVertex,NewForwardVector)*FVector::ForwardVector + FVector::DotProduct(BaseVertex,NewRightVector)*FVector::RightVector;
		TransformedBase.Add(TransformedBaseVertex);
	}
	float Volume = PyramidVolume(TransformedBase, Height);

	if (!DrawDebug) return Volume;

	// DEBUG DRAWING PART //

	Top = UnrotateVectorWithNormal(Top);

	float Alpha = FMath::Clamp(Volume*2.f, 0.f, 1.f);

	FVector Center = Top;

	for (auto &BaseVertex : Base){
		BaseVertex = UnrotateVectorWithNormal(BaseVertex);
		Center += BaseVertex;
	}
	
	Center /= Base.Num()+1;

	for (auto &BaseVertex : Base) {
		BaseVertex = BaseVertex - (BaseVertex - Center)*Inset;
	}
	Top = Top - (Top - Center)*Inset;

	for (int32 i=0 ; i<Base.Num() ; i++){
		DrawDebugLine(World, Base[i], Base[(i+1)%Base.Num()], Color, true, 0, 0, 0.03*(Alpha));
		DrawDebugLine(World, Base[i], Top, Color, true, 0, 0, 0.03*(Alpha));
	}

	return Volume;
}

int32 SectionIndex = 0;

float ComputeSectionVolume(FVector BottomPoint, FVector TopPoint, TArray<TTuple<FVector, FVector, FVector>> &Triangles, bool Draw){

	float Volume=0;

	FVector BottomAverage = FVector::ZeroVector;

	float BottomHeight = BottomPoint.Z;
	float TopHeight = TopPoint.Z;

	SavedRotationData = TTuple<FVector,float> {FVector::ZeroVector,0};

	if(FMath::IsNearlyEqual(BottomHeight,TopHeight)){
		return 0;
	}

	FColor Color = ColorMap[SectionIndex%ColorMap.Num()];

	TArray<TArray<FVector>> SideFaces;
	TSet<FVector> BaseFace;

	for (auto Triangle : Triangles) {
		FVector A = Triangle.Get<0>();
		FVector B = Triangle.Get<1>();
		FVector C = Triangle.Get<2>();

		if (C.Z <= BottomHeight) continue;
		if (A.Z >= TopHeight) break;

		FVector T1;
		FVector T2;
		FVector B1;
		FVector B2;

		TSet<FVector> SideFace;

		T1 = FMath::Lerp(A, C, (TopHeight - A.Z) / (C.Z - A.Z));
		B1 = FMath::Lerp(A, C, (BottomHeight - A.Z) / (C.Z - A.Z));

		if (FMath::IsNearlyEqual(TopHeight, B.Z)) {
			T2 = B;
		}
		else if (TopHeight < B.Z) {
			T2 = FMath::Lerp(A, B, (TopHeight - A.Z) / (B.Z - A.Z));
		}
		else {
			T2 = FMath::Lerp(B, C, (TopHeight - B.Z) / (C.Z - B.Z));
		}

		if (FMath::IsNearlyEqual(BottomHeight, B.Z)) {
			B2 = B;
		}
		else if (BottomHeight < B.Z) {
			B2 = FMath::Lerp(A, B, (BottomHeight - A.Z) / (B.Z - A.Z));
		}
		else {
			B2 = FMath::Lerp(B, C, (BottomHeight - B.Z) / (C.Z - B.Z));
		}

		SideFace.Add(T2);
		SideFace.Add(T1);
		SideFace.Add(B1);
		SideFace.Add(B2);
		if (BottomHeight < B.Z && TopHeight > B.Z) {
			LOGE("SHOULD HIT THIS, EXTRA TRIANGLE VERTICE");
		}

		//TopAverage = (TopAverage*SideFaces.Num()*2 + (T1+T2))/((SideFaces.Num()+1)*2);

		SideFaces.Add(SideFace.Array());
		BaseFace.Add(B1);
		BaseFace.Add(B2);
	}

	for (auto Vertex : BaseFace) {
		BottomAverage += Vertex;
	}
	BottomAverage /= BaseFace.Num();

	BaseFace.Sort([BottomAverage](FVector A, FVector B) { return (BottomAverage - A).Rotation().Yaw < (BottomAverage - B).Rotation().Yaw; });
	TArray<FVector> BaseFaceArray = BaseFace.Array();

	float PrevVolume = Volume;

	for (int32 i = 0; i<SideFaces.Num(); i++) {
		Volume += ComputePyramidVolume(SideFaces[i], TopPoint, Draw, .1, Color);
	}
	Volume += ComputePyramidVolume(BaseFaceArray, TopPoint, Draw, .1, Color);

	if(Volume> 0.0001 && Draw){
		SectionIndex++;
	}

	return Volume;
}

FVector ULiquidSystem::GetVolumetricSlicingPlane(UStaticMeshComponent *StaticMeshComponent, float Alpha, FVector PlaneNormal, bool Debug){

	FVector Result = FVector::ZeroVector;	

	if (StaticMeshComponent){
		World = StaticMeshComponent->GetOwner()->GetWorld();
	}else{
		return Result;
	}

	TArray<FVector> Vertices;
	TArray<TTuple<FVector, FVector, FVector>> Triangles;
	TPair<float, float> ZBounds;

	GetBuffers(StaticMeshComponent, Vertices, Triangles, ZBounds, PlaneNormal);

	TArray<TPair<float,float>> SlicedVolume;
	SlicedVolume.Reserve(Vertices.Num());

	FString MeshName = StaticMeshComponent->GetStaticMesh()->GetFullName();

	float Volume = 0;
	float TotalVolume = 0;
	float PreviousArea = 0;
	float ReturnHeight = ZBounds.Key;

	LOGW("%d triangles",Triangles.Num());

	if(MeshVolume.Contains(MeshName)){
		TotalVolume = MeshVolume[MeshName];
	}else{
		SectionIndex=0;
		for (int32 i = 1; i < Vertices.Num(); i++) {
			TotalVolume += ComputeSectionVolume(Vertices[i - 1], Vertices[i], Triangles, false);
		}
		MeshVolume.Add(MeshName,TotalVolume);
		LOG("Couldn't find volume for %s, computed %f",*(MeshName),TotalVolume);
	}

	float TargetVolume = TotalVolume*Alpha;

	const float AcceptableError = 0.01f;
	const uint32 Iterations = 10;

	SectionIndex = 0;
	for(int32 i=1 ; i < Vertices.Num() ; i++){
		if (Volume >= TargetVolume - AcceptableError){
			LOG("Volume : %f/%f after iteration %d/%d", Volume, TargetVolume, i, Vertices.Num());
			Result = Vertices[i-1];
			break;
		}

		float SectionVolume = ComputeSectionVolume(Vertices[i-1], Vertices[i],Triangles, false);
		if (Volume + SectionVolume <= TargetVolume + AcceptableError){
			if(Debug)
				ComputeSectionVolume(Vertices[i - 1], Vertices[i], Triangles, true); // DEBUG LINE REMOVE THIS
			Volume += SectionVolume;
		}else{
			FVector TempBot = Vertices[i-1];
			FVector TempTop = Vertices[i];

			SectionVolume = ComputeSectionVolume(Vertices[i - 1], (TempBot + TempTop) / 2, Triangles, false);

			for(int32 iter=0 ; iter<Iterations; iter++){
				if(FMath::IsNearlyEqual(SectionVolume+Volume,TargetVolume, AcceptableError)){
					//LOGW("[%d] NEARLY EQUAL AT ITER %d (shouldn't be 0)",i,iter);
					break;
				}else if(SectionVolume+Volume > TargetVolume){
					TempTop = (TempBot + TempTop) / 2;
				}else{
					TempBot = (TempBot + TempTop) / 2;
				}
				SectionVolume = ComputeSectionVolume(Vertices[i - 1], (TempBot + TempTop) / 2, Triangles, false);
			}
			if (Debug)
				SectionVolume = ComputeSectionVolume(Vertices[i - 1], (TempBot + TempTop) / 2, Triangles, true);  // DEBUG LINE REMOVE THIS
			Volume += SectionVolume;
			Result = (TempBot + TempTop)/2;
			break;
		}
	}

	Result = UnrotateVectorWithNormal(Result, PlaneNormal);

	LOG("Volume : %f/%f\nResult : %s", Volume, TargetVolume, *(Result.ToString()));

	//PlaneNormal = FVector::UpVector;

	//return Result;

	/*FVector RotationPlaneNormal = FVector::CrossProduct(PlaneNormal, FVector::UpVector);
	RotationPlaneNormal.Normalize();
	float RotationAngle = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(PlaneNormal, FVector::UpVector)));

	Result = FMath::LinePlaneIntersection(StaticMeshComponent->GetComponentLocation(), StaticMeshComponent->GetComponentLocation() + StaticMeshComponent->GetUpVector(), FVector(0., 0., ReturnHeight), FVector::UpVector);
	Result = (Result - StaticMeshComponent->GetComponentLocation()).RotateAngleAxis(-RotationAngle, RotationPlaneNormal) + StaticMeshComponent->GetComponentLocation();
	*/

	if (Debug)
		DrawDebugPoint(World, Result, 10, FColor::Green, true, 0, 0);
	//DrawDebugLine(World, Result, OldResult, FColor::Red, true, 0, 0, 0.03);

	return Result;
}

float ULiquidSystem::GetSlicedExitArea(UStaticMeshComponent *StaticMeshComponent, FVector PlanePosition, FVector PlaneNormal) {
	float Area = 0;

	if (StaticMeshComponent) {
		World = StaticMeshComponent->GetOwner()->GetWorld();
	}
	else {
		return Area;
	}


	TArray<FVector> Vertices;
	TArray<TTuple<FVector, FVector, FVector>> Triangles;
	TPair<float, float> ZBounds;

	GetBuffers(StaticMeshComponent, Vertices, Triangles, ZBounds, PlaneNormal);

	FVector RotationPlaneNormal = FVector::CrossProduct(PlaneNormal, FVector::UpVector); // Maybe revert vectors?
	RotationPlaneNormal.Normalize();
	float RotationAngle = FMath::RadiansToDegrees(FMath::Acos(FVector::DotProduct(PlaneNormal, FVector::UpVector)));

	PlanePosition = (PlanePosition - StaticMeshComponent->GetComponentLocation()).RotateAngleAxis(RotationAngle, RotationPlaneNormal) + StaticMeshComponent->GetComponentLocation();

	if (Vertices.Num() < 2) return 0;

	//TPair<FVector, FVector> PreviousSpan = (Vertices[0],Vertices[0]);
	float BaseLength = 0.;
	FVector BasePoint = Vertices[0];
	float PreviousHeight = ZBounds.Key;
	FVector Center;

	FVector OrthoPlanePosition = BasePoint;
	FVector OrthoPlaneNormal = FVector::CrossProduct(StaticMeshComponent->GetUpVector(), FVector::UpVector);

	for (int32 i = 0; i < Vertices.Num(); i++) {
		if (Vertices[i].Z > PlanePosition.Z) {
			Vertices[i] = PlanePosition;
			Vertices.SetNum(i + 1);
			break;
		}
	}

	for (auto Vertex : Vertices) {

		TTuple<FVector, FVector> Span = GetSliceSpan(Triangles, Vertex.Z);

		FVector A = Span.Get<0>();
		FVector B = Span.Get<1>();

		if (Span == TTuple<FVector, FVector>(FVector::ZeroVector, FVector::ZeroVector)) {
			continue;
		}

		if (A == B) {
			B = A + OrthoPlaneNormal;
		}

		FVector TopPoint = FMath::LinePlaneIntersection(A, B, OrthoPlanePosition, OrthoPlaneNormal);
		float TopLength = (A - B).Size();
		float Height = (TopPoint - BasePoint).Size();

		Area += Height*(BaseLength + TopLength) / 2.;

		BasePoint = TopPoint;
		BaseLength = TopLength;


		//LOGE("%f Area at height %f (%f)",Area, Vertex.Z);
	}

	LOGE("Area : %f",Area);

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