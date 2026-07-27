#include "CableSimContactManifold.h"

namespace CableSim
{
	uint64 PackContactFeatureId(const FCollisionFeatureId& FeatureId)
	{
		const uint64 Low =
			static_cast<uint64>(static_cast<uint32>(FeatureId.ShapeIndex))
			^ (static_cast<uint64>(static_cast<uint32>(FeatureId.Index0)) << 16)
			^ (static_cast<uint64>(static_cast<uint32>(FeatureId.Index1)) << 32)
			^ (static_cast<uint64>(static_cast<uint8>(FeatureId.Type)) << 48);
		return (FeatureId.ObjectToken * 0x9E3779B97F4A7C15ull) ^ Low;
	}

	FVector3d FContactManifoldCompiler::ClosestPointOnTriangle(
		const FVector3d& Point,
		const FVector3d& A,
		const FVector3d& B,
		const FVector3d& C)
	{
		const FVector3d AB = B - A;
		const FVector3d AC = C - A;
		const FVector3d AP = Point - A;
		const double D1 = FVector3d::DotProduct(AB, AP);
		const double D2 = FVector3d::DotProduct(AC, AP);
		if (D1 <= 0.0 && D2 <= 0.0)
		{
			return A;
		}
		const FVector3d BP = Point - B;
		const double D3 = FVector3d::DotProduct(AB, BP);
		const double D4 = FVector3d::DotProduct(AC, BP);
		if (D3 >= 0.0 && D4 <= D3)
		{
			return B;
		}
		const FVector3d CP = Point - C;
		const double D5 = FVector3d::DotProduct(AB, CP);
		const double D6 = FVector3d::DotProduct(AC, CP);
		if (D6 >= 0.0 && D5 <= D6)
		{
			return C;
		}
		const double VC = D1 * D4 - D3 * D2;
		if (VC <= 0.0 && D1 >= 0.0 && D3 <= 0.0)
		{
			const double V = D1 / (D1 - D3);
			return A + AB * V;
		}
		const double VB = D5 * D2 - D1 * D6;
		if (VB <= 0.0 && D2 >= 0.0 && D6 <= 0.0)
		{
			const double V = D2 / (D2 - D6);
			return A + AC * V;
		}
		const double VA = D3 * D6 - D5 * D4;
		if (VA <= 0.0 && (D4 - D3) >= 0.0 && (D5 - D6) >= 0.0)
		{
			const double V = (D4 - D3) / ((D4 - D3) + (D5 - D6));
			return B + (C - B) * V;
		}
		const double Denominator = 1.0 / (VA + VB + VC);
		const double V = VB * Denominator;
		const double W = VC * Denominator;
		return A + AB * V + AC * W;
	}

	void FContactManifoldCompiler::CompileNodeContacts(
		const int32 ParticleIndex,
		const FVector3d& NodePosition,
		const FVector3d& NodePreviousPosition,
		const TConstArrayView<FCollisionTriangle> Triangles,
		const FManifoldConfig& Config,
		TArray<FContactConstraint>& OutContacts)
	{
		const double Radius = FMath::Max(Config.NodeRadius, 0.0);
		const double GatherDistance = Radius + FMath::Max(Config.ActiveBand, 0.0);
		const double MergeCosine = FMath::Clamp(Config.MergeNormalCosine, -1.0, 1.0);
		const double MergeOffset = FMath::Max(Config.MergeOffsetTolerance, 0.0);

		struct FCandidate
		{
			FVector3d Normal;
			double PlaneOffset = 0.0;
			double Distance = 0.0;
			FCollisionFeatureId FeatureId;
			bool bStatic = false;
		};
		TArray<FCandidate, TInlineAllocator<16>> Candidates;

		for (const FCollisionTriangle& Triangle : Triangles)
		{
			if (!Triangle.IsFinite())
			{
				continue;
			}
			const FVector3d Normal = Triangle.CalculateNormal();
			if (Normal.IsNearlyZero())
			{
				continue;
			}
			const FVector3d ClosestPoint = ClosestPointOnTriangle(
				NodePosition,
				Triangle.Vertices[0],
				Triangle.Vertices[1],
				Triangle.Vertices[2]);
			const double Distance = FVector3d::Distance(NodePosition, ClosestPoint);
			if (Distance > GatherDistance)
			{
				continue;
			}

			const double PlaneOffset = FVector3d::DotProduct(Triangle.Vertices[0], Normal);
			bool bMerged = false;
			for (FCandidate& Candidate : Candidates)
			{
				if (FVector3d::DotProduct(Candidate.Normal, Normal) >= MergeCosine
					&& FMath::Abs(Candidate.PlaneOffset - PlaneOffset) <= MergeOffset)
				{
					if (Distance < Candidate.Distance)
					{
						Candidate.Normal = Normal;
						Candidate.PlaneOffset = PlaneOffset;
						Candidate.Distance = Distance;
						Candidate.FeatureId = Triangle.Id;
						Candidate.bStatic = Triangle.bStaticObject;
					}
					bMerged = true;
					break;
				}
			}
			if (!bMerged)
			{
				Candidates.Add({Normal, PlaneOffset, Distance, Triangle.Id, Triangle.bStaticObject});
			}
		}

		Candidates.Sort([](const FCandidate& First, const FCandidate& Second)
		{
			return First.Distance < Second.Distance;
		});

		const int32 PlaneCount = FMath::Min(Candidates.Num(), FMath::Max(Config.MaxPlanesPerNode, 0));
		for (int32 Index = 0; Index < PlaneCount; ++Index)
		{
			const FCandidate& Candidate = Candidates[Index];
			FContactConstraint& Contact = OutContacts.AddDefaulted_GetRef();
			Contact.FeatureId = PackContactFeatureId(Candidate.FeatureId);
			Contact.ParticleIndex = ParticleIndex;
			Contact.Normal = Candidate.Normal;
			Contact.MinimumNormalCoordinate = Candidate.PlaneOffset + Radius;
			Contact.FrictionAnchorPosition = NodePreviousPosition;
			Contact.bHasFrictionAnchor = true;
		}
	}
}
