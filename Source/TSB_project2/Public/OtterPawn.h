// OtterPawn.h - OTTER USV with Fossen dynamics (3-DOF / 6-DOF)
// TSB Recruitment Phase 2 - Autonomous Systems Simulator
//
// References:
//   [1] T. I. Fossen, "Handbook of Marine Craft Hydrodynamics and Motion
//       Control", 2nd Ed., Wiley 2021.  Eq. (6.1)-(6.44), Ch. 4 (hydrostatics).
//   [2] T. I. Fossen, "otter.py" - Marine Systems Simulator Python model of the
//       Maritime Robotics OTTER USV.
//       https://github.com/cybergalactic/PythonVehicleSimulator
//       (file: src/python_vehicle_simulator/vehicles/otter.py)
//   [3] Maritime Robotics, OTTER product datasheet (L = 2.0 m, m = 55 kg).
//
// All mass/inertia/damping/added-mass parameters are from the official
// ASV Physics Parameters table. Hydrostatic parameters (GM_T, GM_L, Awp)
// are derived from the OTTER catamaran geometry using naval-architecture
// standard formulas (Fossen 2021, Ch. 4.2) - see OtterPawn.cpp for the
// full derivation.

#pragma once

#include "CoreMinimal.h"
#include "GameFramework/Pawn.h"
#include "GameFramework/SpringArmComponent.h"
#include "Camera/CameraComponent.h"
#include "Components/StaticMeshComponent.h"
#include "OtterPawn.generated.h"

UCLASS()
class TSB_PROJECT2_API AOtterPawn : public APawn
{
    GENERATED_BODY()

public:
    AOtterPawn();
    virtual void Tick(float DeltaTime) override;
    virtual void SetupPlayerInputComponent(UInputComponent* PlayerInputComponent) override;

    // ──── Public getters for SensorsComponent ────
    float GetEta_x() const { return Eta_x; }
    float GetEta_y() const { return Eta_y; }
    float GetEta_z() const { return Eta_z; }
    float GetPhi()   const { return Phi; }
    float GetTheta() const { return Theta; }
    float GetPsi()   const { return Psi; }
    float GetU()     const { return u; }
    float GetV()     const { return v; }
    float GetW()     const { return w; }
    float GetP()     const { return p; }
    float GetQ()     const { return q; }
    float GetR()     const { return r; }
    float GetLeftThrust()  const { return LastF_left; }
    float GetRightThrust() const { return LastF_right; }

protected:
    virtual void BeginPlay() override;

    // Components
    UPROPERTY(VisibleAnywhere, Category = "Components") USceneComponent* SceneRoot;
    UPROPERTY(VisibleAnywhere, Category = "Components") UStaticMeshComponent* HullMesh;
    UPROPERTY(VisibleAnywhere, Category = "Components") USpringArmComponent* SpringArm;
    UPROPERTY(VisibleAnywhere, Category = "Components") UCameraComponent* Camera;

    // Global parameters (official ASV table + Maritime Robotics datasheet)
    UPROPERTY(EditAnywhere, Category = "OTTER|Global") float Mass = 55.0f;    // kg  [3]
    UPROPERTY(EditAnywhere, Category = "OTTER|Global") float FluidDensity = 1025.0f;  // kg/m^3 (seawater)
    UPROPERTY(EditAnywhere, Category = "OTTER|Global") float Gravity = 9.81f;    // m/s^2

    // Inertia (official ASV table)
    UPROPERTY(EditAnywhere, Category = "OTTER|Inertia") float Ix = 12.4643f;  // kg.m^2
    UPROPERTY(EditAnywhere, Category = "OTTER|Inertia") float Iy = 18.15f;    // kg.m^2
    UPROPERTY(EditAnywhere, Category = "OTTER|Inertia") float Iz = 15.95f;    // kg.m^2

    // Linear damping (official ASV table)
    UPROPERTY(EditAnywhere, Category = "OTTER|Damping") float Xu = 77.0f;
    UPROPERTY(EditAnywhere, Category = "OTTER|Damping") float Yv = 137.0f;
    UPROPERTY(EditAnywhere, Category = "OTTER|Damping") float Zw = 546.0f;

    // Angular damping (official ASV table)
    UPROPERTY(EditAnywhere, Category = "OTTER|Damping") float Kp = 54.0f;
    UPROPERTY(EditAnywhere, Category = "OTTER|Damping") float Mq = 246.0f;
    UPROPERTY(EditAnywhere, Category = "OTTER|Damping") float Nr = 46.0f;

    // Added mass (official ASV table)
    UPROPERTY(EditAnywhere, Category = "OTTER|AddedMass") float Xu_dot = 5.2815f;
    UPROPERTY(EditAnywhere, Category = "OTTER|AddedMass") float Yv_dot = 82.5f;
    UPROPERTY(EditAnywhere, Category = "OTTER|AddedMass") float Zw_dot = 55.0f;
    UPROPERTY(EditAnywhere, Category = "OTTER|AddedMass") float Kp_dot = 2.4929f;
    UPROPERTY(EditAnywhere, Category = "OTTER|AddedMass") float Mq_dot = 14.52f;
    UPROPERTY(EditAnywhere, Category = "OTTER|AddedMass") float Nr_dot = 27.115f;

    // Thrusters (BlueRobotics T200) - matches Fossen otter.py y_pont = 0.395 m
    UPROPERTY(EditAnywhere, Category = "OTTER|Thrusters") float ThrusterLeverArm = 0.395f;
    UPROPERTY(EditAnywhere, Category = "OTTER|Thrusters") float ThrusterVerticalOffset = 0.05f;
    UPROPERTY(EditAnywhere, Category = "OTTER|Thrusters") float MaxCurrent = 20.0f;

    // ──── Hull geometry (Fossen otter.py [2], OTTER datasheet [3]) ─────────
    // These are the OFFICIAL OTTER catamaran dimensions. They drive the
    // hydrostatic restoring forces in 6DOF via naval-architecture formulas
    // (see ComputeHydrostatics() in OtterPawn.cpp).
    UPROPERTY(EditAnywhere, Category = "OTTER|Hull") float L_hull = 2.0f;    // m  pontoon length  [2][3]
    UPROPERTY(EditAnywhere, Category = "OTTER|Hull") float B_pont = 0.25f;   // m  pontoon width   [2]
    UPROPERTY(EditAnywhere, Category = "OTTER|Hull") float y_pont = 0.395f;  // m  pontoon offset  [2]
    UPROPERTY(EditAnywhere, Category = "OTTER|Hull") float BG = 0.173f;  // m  CoG above CoB   [2] (rg_z - T/2)

    // Simulation control
    UPROPERTY(EditAnywhere, Category = "OTTER|Sim") float PhysicsTimeStep = 0.005f;
    UPROPERTY(EditAnywhere, Category = "OTTER|Sim") bool  bUse3DOFMode = true;

    UPROPERTY(EditAnywhere, Category = "OTTER|Debug") bool bShowDebugHUD = true;
    UPROPERTY(EditAnywhere, Category = "OTTER|Debug") bool bDrawDebugVectors = true;

private:
    // State: kinematics (NED, metres / radians)
    float Eta_x = 0.0f, Eta_y = 0.0f, Eta_z = 0.0f;
    float Phi = 0.0f, Theta = 0.0f, Psi = 0.0f;

    // State: dynamics (body frame)
    float u = 0.0f, v = 0.0f, w = 0.0f;
    float p = 0.0f, q = 0.0f, r = 0.0f;

    // Inputs
    float ThrottleInput = 0.0f;
    float SteeringInput = 0.0f;
    float LeftThrusterInput = 0.0f;
    float RightThrusterInput = 0.0f;

    float LastF_left = 0.0f;
    float LastF_right = 0.0f;
    float PhysicsAccumulator = 0.0f;

    // Derived hydrostatic coefficients (computed in BeginPlay from hull geometry)
    float G33 = 0.0f;   // heave stiffness    [N/m]
    float G44 = 0.0f;   // roll  stiffness    [N.m/rad]
    float G55 = 0.0f;   // pitch stiffness    [N.m/rad]
    float GM_T = 0.0f;  // transverse metacentric height [m] (diagnostic)
    float GM_L = 0.0f;  // longitudinal metacentric height [m] (diagnostic)

    void OnThrottle(float Value);
    void OnSteering(float Value);

    float T200ThrustFromCurrent(float CurrentAmps) const;

    void ComputeThrust(float LeftInput, float RightInput,
        float& OutTauX, float& OutTauY, float& OutTauZ,
        float& OutTauK, float& OutTauM, float& OutTauN);

    void ComputeFossenDynamics(
        float TauX, float TauY, float TauZ,
        float TauK, float TauM, float TauN,
        float& du, float& dv, float& dw,
        float& dp, float& dq, float& dr) const;

    void ComputeHydrostatics();

    void IntegrateStep(float dt);
    void UpdateKinematics(float dt);
    void ApplyStateToTransform();
    void DrawDebugHUD();
    void DrawDebugVectors();
};