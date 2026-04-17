// SensorsComponent.cpp - see header for description.

#include "SensorsComponent.h"
#include "OtterPawn.h"
#include "Engine/Engine.h"
#include "DrawDebugHelpers.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "HAL/PlatformFileManager.h"
#include <cmath>

USensorsComponent::USensorsComponent()
{
    PrimaryComponentTick.bCanEverTick = true;
}

void USensorsComponent::BeginPlay()
{
    Super::BeginPlay();

    OwnerPawn = Cast<AOtterPawn>(GetOwner());
    if (!OwnerPawn)
    {
        UE_LOG(LogTemp, Error, TEXT("SensorsComponent: Owner is not an OtterPawn!"));
        return;
    }

    // Initialize latency buffers.
    // Buffer size = (latency / sample_period) + margin.
    // We store ground truth at physics rate (200 Hz = 5 ms).
    const float PhysDt = 0.005f;

    int32 AHRS_BufSize = FMath::CeilToInt(AHRS_LatencyMs / (PhysDt * 1000.0f)) + 10;
    int32 GPS_BufSize = FMath::CeilToInt(GPS_LatencyMs / (PhysDt * 1000.0f)) + 10;
    int32 LiDAR_BufSize = FMath::CeilToInt(LiDAR_LatencyMs / (PhysDt * 1000.0f)) + 10;

    AHRS_Buffer.Init(AHRS_BufSize);
    GPS_Buffer.Init(GPS_BufSize);
    LiDAR_Buffer.Init(LiDAR_BufSize);

    // Initialize LiDAR buffer with empty scans
    int32 NumRays = FMath::CeilToInt(360.0f / LiDAR_AngularResDeg);
    TArray<float> EmptyScan;
    EmptyScan.SetNum(NumRays);
    for (int32 i = 0; i < NumRays; ++i) EmptyScan[i] = LiDAR_MaxRange;
    for (int32 i = 0; i < LiDAR_BufSize; ++i) LiDAR_Buffer.Push(EmptyScan);

    // Initialize GT buffers with zeros
    FGroundTruth ZeroGT = {};
    for (int32 i = 0; i < AHRS_BufSize; ++i) AHRS_Buffer.Push(ZeroGT);
    for (int32 i = 0; i < GPS_BufSize; ++i) GPS_Buffer.Push(ZeroGT);

    // Setup CSV file
    if (bEnableCSVLog)
    {
        FString Timestamp = FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S"));
        CSVFilePath = FPaths::ProjectSavedDir() / TEXT("Logs") / FString::Printf(TEXT("SensorLog_%s.csv"), *Timestamp);
        bCSVHeaderWritten = false;
    }
}

void USensorsComponent::TickComponent(float DeltaTime, ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!OwnerPawn) return;

    const float GameTime = GetWorld()->GetTimeSeconds();

    // Read current ground truth and push to all latency buffers
    FGroundTruth GT = ReadGroundTruth(DeltaTime);
    AHRS_Buffer.Push(GT);
    GPS_Buffer.Push(GT);

    // AHRS sampling
    AHRS_Accumulator += DeltaTime;
    float AHRS_Period = 1.0f / AHRS_Rate;
    if (AHRS_Accumulator >= AHRS_Period)
    {
        AHRS_Accumulator -= AHRS_Period;
        SampleAHRS(GameTime);
    }

    // GPS sampling
    GPS_Accumulator += DeltaTime;
    float GPS_Period = 1.0f / GPS_Rate;
    if (GPS_Accumulator >= GPS_Period)
    {
        GPS_Accumulator -= GPS_Period;
        SampleGPS(GameTime);
    }

    // LiDAR sampling
    LiDAR_Accumulator += DeltaTime;
    float LiDAR_Period = 1.0f / LiDAR_Rate;
    if (LiDAR_Accumulator >= LiDAR_Period)
    {
        LiDAR_Accumulator -= LiDAR_Period;

        // Perform sweep and push to buffer
        TArray<float> RawScan = PerformLiDARSweep();
        LiDAR_Buffer.Push(RawScan);

        SampleLiDAR(GameTime);
    }

    // CSV logging
    if (bEnableCSVLog)
    {
        CSV_Accumulator += DeltaTime;
        float CSV_Period = 1.0f / CSV_Rate;
        if (CSV_Accumulator >= CSV_Period)
        {
            CSV_Accumulator -= CSV_Period;
            WriteCSVLine(GameTime, GT);
        }
    }

    // Debug visualization
    if (bShowSensorHUD) DrawSensorHUD(GT);
    if (bDrawLiDARRays) DrawLiDARVisualization();
}

void USensorsComponent::EndPlay(const EEndPlayReason::Type EndPlayReason)
{
    Super::EndPlay(EndPlayReason);
    UE_LOG(LogTemp, Log, TEXT("SensorsComponent: CSV log saved to %s"), *CSVFilePath);
}

// ──────────────────────────────────────────────────────────────
// Ground truth reader
// ──────────────────────────────────────────────────────────────

FGroundTruth USensorsComponent::ReadGroundTruth(float DeltaTime)
{
    FGroundTruth GT;

    GT.Eta_x = OwnerPawn->GetEta_x();
    GT.Eta_y = OwnerPawn->GetEta_y();
    GT.Eta_z = OwnerPawn->GetEta_z();
    GT.Phi = OwnerPawn->GetPhi();
    GT.Theta = OwnerPawn->GetTheta();
    GT.Psi = OwnerPawn->GetPsi();
    GT.u = OwnerPawn->GetU();
    GT.v = OwnerPawn->GetV();
    GT.w = OwnerPawn->GetW();
    GT.p = OwnerPawn->GetP();
    GT.q = OwnerPawn->GetQ();
    GT.r = OwnerPawn->GetR();
    GT.LeftThrust = OwnerPawn->GetLeftThrust();
    GT.RightThrust = OwnerPawn->GetRightThrust();
    GT.GameTime = GetWorld()->GetTimeSeconds();

    // Compute linear acceleration by finite difference
    if (DeltaTime > 0.0f)
    {
        GT.AccX = (GT.u - PrevU) / DeltaTime;
        GT.AccY = (GT.v - PrevV) / DeltaTime;
        GT.AccZ = (GT.w - PrevW) / DeltaTime;
    }
    PrevU = GT.u;
    PrevV = GT.v;
    PrevW = GT.w;

    return GT;
}

// ──────────────────────────────────────────────────────────────
// Gaussian noise (Box-Muller transform)
// ──────────────────────────────────────────────────────────────

float USensorsComponent::GaussianNoise(float Sigma)
{
    if (Sigma <= 0.0f) return 0.0f;

    float U1 = FMath::FRand();  // uniform [0, 1)
    float U2 = FMath::FRand();

    // Clamp U1 away from zero to avoid log(0)
    U1 = FMath::Max(U1, 1e-6f);

    float Z = FMath::Sqrt(-2.0f * FMath::Loge(U1)) * FMath::Cos(2.0f * PI * U2);
    return Z * Sigma;
}

// ──────────────────────────────────────────────────────────────
// AHRS sampling
// ──────────────────────────────────────────────────────────────

void USensorsComponent::SampleAHRS(float GameTime)
{
    // Read delayed ground truth
    const float PhysDt = 0.005f;
    int32 DelayTicks = FMath::RoundToInt((AHRS_LatencyMs / 1000.0f) / PhysDt);
    const FGroundTruth& GT = AHRS_Buffer.ReadDelayed(DelayTicks);

    // Check for failure
    if (FMath::FRand() < AHRS_FailureProbability)
    {
        LatestAHRS.bValid = false;
        return;
    }

    float NoiseDegToRad = FMath::DegreesToRadians(AHRS_OrientationNoiseDeg);

    LatestAHRS.bValid = true;
    LatestAHRS.Timestamp = GT.GameTime;
    LatestAHRS.Roll = GT.Phi + GaussianNoise(NoiseDegToRad);
    LatestAHRS.Pitch = GT.Theta + GaussianNoise(NoiseDegToRad);
    LatestAHRS.Yaw = GT.Psi + GaussianNoise(NoiseDegToRad);
    LatestAHRS.AngVelX = GT.p + GaussianNoise(AHRS_GyroNoiseRadS);
    LatestAHRS.AngVelY = GT.q + GaussianNoise(AHRS_GyroNoiseRadS);
    LatestAHRS.AngVelZ = GT.r + GaussianNoise(AHRS_GyroNoiseRadS);
    LatestAHRS.AccX = GT.AccX + GaussianNoise(AHRS_AccelNoiseMS2);
    LatestAHRS.AccY = GT.AccY + GaussianNoise(AHRS_AccelNoiseMS2);
    LatestAHRS.AccZ = GT.AccZ + GaussianNoise(AHRS_AccelNoiseMS2);
}

// ──────────────────────────────────────────────────────────────
// GPS sampling
// ──────────────────────────────────────────────────────────────

void USensorsComponent::SampleGPS(float GameTime)
{
    const float PhysDt = 0.005f;
    int32 DelayTicks = FMath::RoundToInt((GPS_LatencyMs / 1000.0f) / PhysDt);
    const FGroundTruth& GT = GPS_Buffer.ReadDelayed(DelayTicks);

    if (FMath::FRand() < GPS_FailureProbability)
    {
        LatestGPS.bValid = false;
        LatestGPS.NumSatellites = FMath::RandRange(0, 3);
        LatestGPS.HDOP = 99.0f;
        return;
    }

    LatestGPS.bValid = true;
    LatestGPS.Timestamp = GT.GameTime;
    LatestGPS.X = GT.Eta_x + GaussianNoise(GPS_PositionNoiseM);
    LatestGPS.Y = GT.Eta_y + GaussianNoise(GPS_PositionNoiseM);
    LatestGPS.NumSatellites = FMath::RandRange(7, 12);
    LatestGPS.HDOP = FMath::FRandRange(0.8f, 2.5f);
}

// ──────────────────────────────────────────────────────────────
// LiDAR sweep and sampling
// ──────────────────────────────────────────────────────────────

TArray<float> USensorsComponent::PerformLiDARSweep()
{
    int32 NumRays = FMath::CeilToInt(360.0f / LiDAR_AngularResDeg);
    TArray<float> Distances;
    Distances.SetNum(NumRays);

    UWorld* World = GetWorld();
    if (!World || !OwnerPawn)
    {
        for (int32 i = 0; i < NumRays; ++i) Distances[i] = LiDAR_MaxRange;
        return Distances;
    }

    const FVector Origin = OwnerPawn->GetActorLocation();
    const FRotator ActorRot = OwnerPawn->GetActorRotation();

    FCollisionQueryParams TraceParams;
    TraceParams.AddIgnoredActor(OwnerPawn);

    for (int32 i = 0; i < NumRays; ++i)
    {
        float AngleDeg = i * LiDAR_AngularResDeg;
        FRotator RayRot = ActorRot + FRotator(0.0f, AngleDeg, 0.0f);
        FVector Direction = RayRot.Vector();
        FVector End = Origin + Direction * LiDAR_MaxRange * 100.0f; // m to cm

        FHitResult Hit;
        bool bHit = World->LineTraceSingleByChannel(Hit, Origin, End, ECC_Visibility, TraceParams);

        if (bHit)
        {
            Distances[i] = Hit.Distance / 100.0f; // cm to m
        }
        else
        {
            Distances[i] = LiDAR_MaxRange;
        }
    }

    return Distances;
}

void USensorsComponent::SampleLiDAR(float GameTime)
{
    const float PhysDt = 1.0f / LiDAR_Rate;  // LiDAR buffer is pushed at LiDAR rate
    int32 DelayTicks = FMath::RoundToInt((LiDAR_LatencyMs / 1000.0f) / PhysDt);
    DelayTicks = FMath::Clamp(DelayTicks, 0, LiDAR_Buffer.Size - 1);
    const TArray<float>& RawScan = LiDAR_Buffer.ReadDelayed(DelayTicks);

    if (FMath::FRand() < LiDAR_FailureProbability)
    {
        LatestLiDAR.bValid = false;
        return;
    }

    LatestLiDAR.bValid = true;
    LatestLiDAR.Timestamp = GameTime - (LiDAR_LatencyMs / 1000.0f);
    LatestLiDAR.AngularResolutionDeg = LiDAR_AngularResDeg;
    LatestLiDAR.Distances.SetNum(RawScan.Num());

    for (int32 i = 0; i < RawScan.Num(); ++i)
    {
        float Dist = RawScan[i];
        if (Dist < LiDAR_MaxRange)
        {
            Dist += GaussianNoise(LiDAR_NoiseM);
            Dist = FMath::Max(Dist, 0.0f);
        }
        LatestLiDAR.Distances[i] = Dist;
    }
}

// ──────────────────────────────────────────────────────────────
// LiDAR debug visualization
// ──────────────────────────────────────────────────────────────

void USensorsComponent::DrawLiDARVisualization()
{
    if (!LatestLiDAR.bValid || !OwnerPawn) return;

    UWorld* World = GetWorld();
    if (!World) return;

    const FVector Origin = OwnerPawn->GetActorLocation();
    const FRotator ActorRot = OwnerPawn->GetActorRotation();
    const int32 NumRays = LatestLiDAR.Distances.Num();

    for (int32 i = 0; i < NumRays; ++i)
    {
        float AngleDeg = i * LatestLiDAR.AngularResolutionDeg;
        FRotator RayRot = ActorRot + FRotator(0.0f, AngleDeg, 0.0f);
        FVector Direction = RayRot.Vector();

        float Dist = LatestLiDAR.Distances[i];
        FVector End = Origin + Direction * Dist * 100.0f; // m to cm

        FColor RayColor = (Dist < LiDAR_MaxRange) ? FColor::Red : FColor(0, 100, 0, 80);
        float Thickness = (Dist < LiDAR_MaxRange) ? 2.0f : 0.5f;

        DrawDebugLine(World, Origin, End, RayColor, false, 0.0f, 0, Thickness);
    }
}

// ──────────────────────────────────────────────────────────────
// CSV logging
// ──────────────────────────────────────────────────────────────

void USensorsComponent::WriteCSVLine(float GameTime, const FGroundTruth& GT)
{
    if (!bEnableCSVLog) return;

    // Write header on first call
    if (!bCSVHeaderWritten)
    {
        FString Header = TEXT("time,gt_x,gt_y,gt_yaw,gt_u,gt_v,gt_r,")
            TEXT("gps_x,gps_y,gps_valid,")
            TEXT("ahrs_yaw,ahrs_p,ahrs_q,ahrs_r,ahrs_valid,")
            TEXT("left_thrust,right_thrust\n");
        FFileHelper::SaveStringToFile(Header, *CSVFilePath,
            FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(),
            FILEWRITE_None);
        bCSVHeaderWritten = true;
    }

    FString Line = FString::Printf(
        TEXT("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,")
        TEXT("%.4f,%.4f,%d,")
        TEXT("%.4f,%.4f,%.4f,%.4f,%d,")
        TEXT("%.2f,%.2f\n"),
        GameTime,
        GT.Eta_x, GT.Eta_y, GT.Psi,
        GT.u, GT.v, GT.r,
        LatestGPS.X, LatestGPS.Y, LatestGPS.bValid ? 1 : 0,
        LatestAHRS.Yaw, LatestAHRS.AngVelX, LatestAHRS.AngVelY, LatestAHRS.AngVelZ,
        LatestAHRS.bValid ? 1 : 0,
        GT.LeftThrust, GT.RightThrust
    );

    FFileHelper::SaveStringToFile(Line, *CSVFilePath,
        FFileHelper::EEncodingOptions::AutoDetect, &IFileManager::Get(),
        FILEWRITE_Append);
}

// ──────────────────────────────────────────────────────────────
// Sensor debug HUD
// ──────────────────────────────────────────────────────────────

void USensorsComponent::DrawSensorHUD(const FGroundTruth& GT)
{
    if (!GEngine) return;

    // Count LiDAR detections
    int32 LiDARHits = 0;
    for (float D : LatestLiDAR.Distances)
    {
        if (D < LiDAR_MaxRange) LiDARHits++;
    }

    FString SensorText = FString::Printf(
        TEXT("\n\n\n\n\n\n\n\n")
        TEXT("=== SENSORS ===\n")
        TEXT("[GPS] %s  X=%.2f Y=%.2f  (GT: %.2f, %.2f)  Sats=%d HDOP=%.1f\n")
        TEXT("[AHRS] %s  Yaw=%.1f deg  (GT: %.1f)  r=%.3f rad/s\n")
        TEXT("[LiDAR] %s  Rays=%d  Hits=%d  Range=%.0fm\n")
        TEXT("[CSV] %s"),
        LatestGPS.bValid ? TEXT("OK") : TEXT("FAIL"),
        LatestGPS.X, LatestGPS.Y,
        GT.Eta_x, GT.Eta_y,
        LatestGPS.NumSatellites, LatestGPS.HDOP,
        LatestAHRS.bValid ? TEXT("OK") : TEXT("FAIL"),
        FMath::RadiansToDegrees(LatestAHRS.Yaw),
        FMath::RadiansToDegrees(GT.Psi),
        LatestAHRS.AngVelZ,
        LatestLiDAR.bValid ? TEXT("OK") : TEXT("FAIL"),
        LatestLiDAR.Distances.Num(), LiDARHits, LiDAR_MaxRange,
        bEnableCSVLog ? TEXT("Recording") : TEXT("Off")
    );

    GEngine->AddOnScreenDebugMessage(-1, 0.0f, FColor::Cyan, SensorText);
}
