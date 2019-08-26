FRotator GridSnap_Rotation(const FRotator& rotator, const float gridSize)
{
    // We assume grid size values of max 90

    const float gridRadians = FMath::DegreesToRadians(gridSize);
    FQuat quat = rotator.Quaternion();

    auto EnsureNonZero = [](const FVector& original, FVector& target) {
        if (target.IsNearlyZero())
        {
            TArray<float> maxHelper = { FMath::Abs(original.X), FMath::Abs(original.Y), FMath::Abs(original.Z) };
            int32 maxIndex = INDEX_NONE;
            FMath::Max(maxHelper, &maxIndex);
            target[maxIndex] = FMath::Sign(original[maxIndex]) * 1.f;
        }
    };

    TArray<FVector> directions = { quat.GetForwardVector(), quat.GetRightVector(), quat.GetUpVector() };

    TArray<FVector> asin;
    asin.Reserve(3);
    for (const FVector& vec : directions)
    {
        asin.Add(FVector(
                     FMath::Asin(vec.X)
                     , FMath::Asin(vec.Y)
                     , FMath::Asin(vec.Z)
                 ));
    }

    TArray<FVector> snapped;
    snapped.Reserve(3);
    for (const FVector& vec : asin)
    {
        snapped.Add(FVector(
                        FMath::GridSnap(vec.X, gridRadians)
                        , FMath::GridSnap(vec.Y, gridRadians)
                        , FMath::GridSnap(vec.Z, gridRadians)
                    ));
    }

    TArray<float> snapDeltaSum;
    snapDeltaSum.Reserve(3);
    for (int32 i = 0; i < 3; ++i)
    {
        FVector tempAbs = (snapped[i] - asin[i]).GetAbs();
        snapDeltaSum.Add(tempAbs.X + tempAbs.Y + tempAbs.Z);
    }

    TArray<int32> indexMin;
    indexMin.Reserve(3);
    for (int32 i = 0; i<3; ++i)
    {
        int32 min = INDEX_NONE;
        FMath::Min(snapDeltaSum, &min);
        indexMin.Add(min);
        snapDeltaSum[min] = BIG_NUMBER; // Don't return that field again.
    }

	// Snap the 2 vectors with the lowest deltaSum as those are closest to their snap position.
    TArray<FVector> results;
    results.SetNum(3);
    for (int32 i = 0; i<2; ++i)
    {
		int32 index = indexMin[i];
        results[index] = FVector(
                             FMath::Sin(snapped[index].X)
                             , FMath::Sin(snapped[index].Y)
                             , FMath::Sin(snapped[index].Z)
                         );
		EnsureNonZero(directions[index], results[index]);
		results[index].Normalize();
    }

	// Calculate the last vector through cross product
	if(indexMin[2] == 0)
	{
		results[0] = results[1] ^ results[2];
	}
	else if (indexMin[2] == 1)
	{
		results[1] = results[2] ^ results[0];
	}
	else if (indexMin[2] == 2)
	{
		results[2] = results[0] ^ results[1];
	}

    return UKismetMathLibrary::MakeRotationFromAxes(results[0], results[1], results[2]);
}
