FRotator UMathExtensionLibrary_BP::GridSnap_Rotation(const FRotator& rotator, const float gridDeg)
{
	// We assume grid size values of max 90

	const float gridRad = FMath::DegreesToRadians(gridDeg);
	FQuat quat = rotator.Quaternion();

	FVector forward = quat.GetForwardVector();
	FVector right = quat.GetRightVector();

	// Forward
	// To get the forward vector, we snap the forward vector of the quaternion
	// to a linear representation of the rotation.
	FVector forwardResult;
	{
		// Bring the vector into linear space
		forwardResult = FVector(
			FMath::Asin(forward.X)
			, FMath::Asin(forward.Y)
			, FMath::Asin(forward.Z)
		);

		// Snap in linear space
		forwardResult = FVector(
			FMath::GridSnap(forwardResult.X, gridRad)
			, FMath::GridSnap(forwardResult.Y, gridRad)
			, FMath::GridSnap(forwardResult.Z, gridRad)
		);

		// Back to sine space
		forwardResult = FVector(
			FMath::Sin(forwardResult.X)
			, FMath::Sin(forwardResult.Y)
			, FMath::Sin(forwardResult.Z)
		);

		// Ensure our forward vector is not Zero
		if(forwardResult.IsNearlyZero())
		{
		   TArray<float> maxHelper = { FMath::Abs(forward.X), FMath::Abs(forward.Y), FMath::Abs(forward.Z) };
		   int32 maxIndex = INDEX_NONE;
		   FMath::Max(maxHelper, &maxIndex);
		   forwardResult[maxIndex] = FMath::Sign(forward[maxIndex]);
		}

		// Snapping each component of the vector to the grid does not yet place the vector
		// in the rotation grid (when z != 0). We need to make a correction that also normalizes
		// the vector again.
		// E.g 45deg rotated around the y axis and then 45deg rotated around the z axis.
		// Cause of the component snapping, all components of the vector are now 0.707 (sin space).
		// Only the z component is valid to have 0.707, X and Y must be adjusted.
		// The proper result must be FVector(0.5, 0.5, 0.707)
		float sizeXYTarget = FMath::Sqrt(1 - FMath::Square(forwardResult.Z));
		FVector2D vec2D = FVector2D(forwardResult);
		float size2d = vec2D.Size();
		if (FMath::IsNearlyZero(size2d))
		{
			vec2D *= 0;
		}
		else
		{
			vec2D *= sizeXYTarget / size2d;
		}

		forwardResult.Normalize();
	}

	// Right
	// To get the right vector we rotate in grid distance around the forward vector
	// and take the vector that is closest to the original right vector.
	FVector rightResult;
	{
		FVector rightTemp;
		if (forwardResult.Equals(FVector(0.f, 0.f, 1.f)))
		{
			rightTemp = FVector(0.f, 1.f, 0.f);
		}
		else if (forwardResult.Equals(FVector(0.f, 0.f, -1.f)))
		{
			rightTemp = FVector(0.f, -1.f, 0.f);
		}
		else
		{
			rightTemp = FVector(0.f, 0.f, 1.f) ^ forwardResult;
			rightTemp.Normalize();
		}
		FVector bestMatch = rightTemp;
		float distClosest = FVector::DistSquared(rightTemp, right);

		bool bInversed = false;
		bool bWasCloser = false;
		int32 rotMultiplier = 0;
		while (true)
		{
			rotMultiplier = rotMultiplier + (bInversed ? -1 : 1);
			FVector rightRotated = rightTemp.RotateAngleAxis(gridDeg * rotMultiplier, forwardResult);
			float dist = FVector::DistSquared(rightRotated, right);
			if (dist < distClosest || FMath::IsNearlyEqual(dist, distClosest, KINDA_SMALL_NUMBER))
			{
				bWasCloser = true;
				distClosest = dist;
				bestMatch = rightRotated;
			}
			else if (dist > distClosest)
			{
				// Getting further away from our target
				if (!bInversed)
				{
					// First time, inverse rotation
					bInversed = true;
				}
				else if(bWasCloser)
				{
					// Have been closest possible already and getting further away again: closest possible found
					break;
				}
			}
		}

		rightResult = bestMatch;
	}

	// Up
	FVector upResult;
	{
		upResult = forwardResult ^ rightResult;
		upResult.Normalize();
	}

	FRotator out = UKismetMathLibrary::MakeRotationFromAxes(forwardResult, rightResult, upResult);
	ensure(!out.ContainsNaN());
	return out;
}
