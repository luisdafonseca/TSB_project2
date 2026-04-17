// SensorsComponent.h - Simulated sensor layer for OTTER USV
// TSB Recruitment Phase 2 - Autonomous Systems Simulator
//
// Implements AHRS, GPS, and LiDAR sensors with:
//   - Gaussian noise (configurable sigma per sensor)
//   - Latency via circular buffer (configurable delay)
//   - Intermittent and total sensor failure (configurable probability)
//   - CSV data logging for post-simulation analysis
//
// Attach this component to BP_Otter in the Blueprint editor.
// All parameters are UPROPERTY and editable without recompilation.

#pragma once

#include "CoreMinimal.h"
#include "Components/ActorComponent.h"
#include "SensorsComponent.generated.h"

// ──────────────────────────────────────────────────────────────
// Sensor reading structs
// ──────────────────────────────────────────────────────────────

USTRUCT(BlueprintType)
struct FAHRSReading
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly) bool bValid = false;
    UPROPERTY(BlueprintReadOnly) float Timestamp = 0.0f;
    UPROPERTY(BlueprintReadOnly) float Roll = 0.0f;       // rad
    UPROPERTY(BlueprintReadOnly) float Pitch = 0.0f;      // rad
    UPROPERTY(BlueprintReadOnly) float Yaw = 0.0f;        // rad
    UPROPERTY(BlueprintReadOnly) float AngVelX = 0.0f;    // rad/s (p)
    UPROPERTY(BlueprintReadOnly) float AngVelY = 0.0f;    // rad/s (q)
    UPROPERTY(BlueprintReadOnly) float AngVelZ = 0.0f;    // rad/s (r)
    UPROPERTY(BlueprintReadOnly) float AccX = 0.0f;       // m/s^2
    UPROPERTY(BlueprintReadOnly) float AccY = 0.0f;       // m/s^2
    UPROPERTY(BlueprintReadOnly) float AccZ = 0.0f;       // m/s^2
};

USTRUCT(BlueprintType)
struct FGPSReading
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly) bool bValid = false;
    UPROPERTY(BlueprintReadOnly) float Timestamp = 0.0f;
    UPROPERTY(BlueprintReadOnly) float X = 0.0f;          // m (NED North)
    UPROPERTY(BlueprintReadOnly) float Y = 0.0f;          // m (NED East)
    UPROPERTY(BlueprintReadOnly) int32 NumSatellites = 0;
    UPROPERTY(BlueprintReadOnly) float HDOP = 99.0f;      // Horizontal Dilution of Precision
};

USTRUCT(BlueprintType)
struct FLiDARScan
{
    GENERATED_BODY()

    UPROPERTY(BlueprintReadOnly) bool bValid = false;
    UPROPERTY(BlueprintReadOnly) float Timestamp = 0.0f;
    UPROPERTY(BlueprintReadOnly) TArray<float> Distances;  // one per ray, MAX_RANGE if no hit
    UPROPERTY(BlueprintReadOnly) float AngularResolutionDeg = 2.0f;
};

// ──────────────────────────────────────────────────────────────
// Ground truth struct (read from OtterPawn)
// ──────────────────────────────────────────────────────────────

struct FGroundTruth
{
    float Eta_x, Eta_y, Eta_z;
    float Phi, Theta, Psi;
    float u, v, w;
    float p, q, r;
    float AccX, AccY, AccZ;     // linear acceleration (body frame)
    float LeftThrust, RightThrust;
    float GameTime;
};

// ──────────────────────────────────────────────────────────────
// Latency buffer template
// ──────────────────────────────────────────────────────────────

template<typename T>
struct TLatencyBuffer
{
    TArray<T> Buffer;
    int32 WriteIndex = 0;
    int32 Size = 0;

    void Init(int32 InSize)
    {
        Size = FMath::Max(InSize, 1);
        Buffer.SetNum(Size);
        WriteIndex = 0;
    }

    void Push(const T& Value)
    {
        Buffer[WriteIndex] = Value;
        WriteIndex = (WriteIndex + 1) % Size;
    }

    // Returns value from N ticks ago. N=0 is current, N=Size-1 is oldest.
    const T& ReadDelayed(int32 DelayTicks) const
    {
        int32 Idx = (WriteIndex - 1 - FMath::Clamp(DelayTicks, 0, Size - 1) + Size) % Size;
        return Buffer[Idx];
    }
};

// ──────────────────────────────────────────────────────────────
// Main component
// ──────────────────────────────────────────────────────────────

UCLASS(ClassGroup = (Custom), meta = (BlueprintSpawnableComponent))
class TSB_PROJECT2_API USensorsComponent : public UActorComponent
{
    GENERATED_BODY()

public:
    USensorsComponent();

    virtual void BeginPlay() override;
    virtual void TickComponent(float DeltaTime, ELevelTick TickType,
        FActorComponentTickFunction* ThisTickFunction) override;
    virtual void EndPlay(const EEndPlayReason::Type EndPlayReason) override;

    // ──── Public accessors for latest readings ────
    UFUNCTION(BlueprintCallable, Category = "Sensors")
    FAHRSReading GetLatestAHRS() const { return LatestAHRS; }

    UFUNCTION(BlueprintCallable, Category = "Sensors")
    FGPSReading GetLatestGPS() const { return LatestGPS; }

    UFUNCTION(BlueprintCallable, Category = "Sensors")
    FLiDARScan GetLatestLiDAR() const { return LatestLiDAR; }

protected:
    // ──── AHRS parameters ────
    UPROPERTY(EditAnywhere, Category = "Sensors|AHRS") float AHRS_Rate = 100.0f;           // Hz
    UPROPERTY(EditAnywhere, Category = "Sensors|AHRS") float AHRS_LatencyMs = 20.0f;       // ms
    UPROPERTY(EditAnywhere, Category = "Sensors|AHRS") float AHRS_OrientationNoiseDeg = 0.5f; // deg sigma
    UPROPERTY(EditAnywhere, Category = "Sensors|AHRS") float AHRS_GyroNoiseRadS = 0.05f;   // rad/s sigma
    UPROPERTY(EditAnywhere, Category = "Sensors|AHRS") float AHRS_AccelNoiseMS2 = 0.05f;   // m/s^2 sigma
    UPROPERTY(EditAnywhere, Category = "Sensors|AHRS") float AHRS_FailureProbability = 0.005f; // per sample

    // ──── GPS parameters ────
    UPROPERTY(EditAnywhere, Category = "Sensors|GPS") float GPS_Rate = 10.0f;               // Hz
    UPROPERTY(EditAnywhere, Category = "Sensors|GPS") float GPS_LatencyMs = 150.0f;         // ms
    UPROPERTY(EditAnywhere, Category = "Sensors|GPS") float GPS_PositionNoiseM = 2.0f;      // m sigma
    UPROPERTY(EditAnywhere, Category = "Sensors|GPS") float GPS_FailureProbability = 0.01f; // per sample

    // ──── LiDAR parameters ────
    UPROPERTY(EditAnywhere, Category = "Sensors|LiDAR") float LiDAR_Rate = 10.0f;           // Hz
    UPROPERTY(EditAnywhere, Category = "Sensors|LiDAR") float LiDAR_LatencyMs = 100.0f;     // ms
    UPROPERTY(EditAnywhere, Category = "Sensors|LiDAR") float LiDAR_MaxRange = 40.0f;       // m
    UPROPERTY(EditAnywhere, Category = "Sensors|LiDAR") float LiDAR_AngularResDeg = 2.0f;   // degrees between rays
    UPROPERTY(EditAnywhere, Category = "Sensors|LiDAR") float LiDAR_NoiseM = 0.03f;         // m sigma
    UPROPERTY(EditAnywhere, Category = "Sensors|LiDAR") float LiDAR_FailureProbability = 0.005f;

    // ──── Visualization ────
    UPROPERTY(EditAnywhere, Category = "Sensors|Debug") bool bShowSensorHUD = true;
    UPROPERTY(EditAnywhere, Category = "Sensors|Debug") bool bDrawLiDARRays = true;

    // ──── CSV logging ────
    UPROPERTY(EditAnywhere, Category = "Sensors|Logging") bool bEnableCSVLog = true;
    UPROPERTY(EditAnywhere, Category = "Sensors|Logging") float CSV_Rate = 10.0f;           // Hz

private:
    // Latest readings
    FAHRSReading LatestAHRS;
    FGPSReading LatestGPS;
    FLiDARScan LatestLiDAR;

    // Timing accumulators
    float AHRS_Accumulator = 0.0f;
    float GPS_Accumulator = 0.0f;
    float LiDAR_Accumulator = 0.0f;
    float CSV_Accumulator = 0.0f;

    // Latency buffers
    TLatencyBuffer<FGroundTruth> AHRS_Buffer;
    TLatencyBuffer<FGroundTruth> GPS_Buffer;
    TLatencyBuffer<TArray<float>> LiDAR_Buffer;

    // CSV file handle
    FString CSVFilePath;
    bool bCSVHeaderWritten = false;

    // Cached owner
    class AOtterPawn* OwnerPawn = nullptr;

    // Previous velocities for acceleration computation
    float PrevU = 0.0f, PrevV = 0.0f, PrevW = 0.0f;

    // Internal methods
    FGroundTruth ReadGroundTruth(float DeltaTime);

    void SampleAHRS(float GameTime);
    void SampleGPS(float GameTime);
    void SampleLiDAR(float GameTime);

    TArray<float> PerformLiDARSweep();

    void WriteCSVLine(float GameTime, const FGroundTruth& GT);
    void DrawSensorHUD(const FGroundTruth& GT);
    void DrawLiDARVisualization();

    // Gaussian noise helper (Box-Muller)
    static float GaussianNoise(float Sigma);
};
