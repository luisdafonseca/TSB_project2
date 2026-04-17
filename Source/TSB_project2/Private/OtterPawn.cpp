// OtterPawn.cpp - OTTER USV with Fossen dynamics (3-DOF / 6-DOF)
// TSB Recruitment Phase 2 - Autonomous Systems Simulator
//
// Hydrostatic parameters for 6DOF are DERIVED from the OTTER catamaran
// geometry using the standard formulas of Fossen (2021, Ch. 4.2) and as
// implemented in the official MSS otter.py model [2]. The derivation is
// documented in ComputeHydrostatics() below.
//
// References:
//   [1] Fossen, "Handbook of Marine Craft Hydrodynamics and Motion Control",
//       2nd Ed., Wiley 2021.
//   [2] Fossen, "otter.py" in PythonVehicleSimulator (MSS), 2021.
//       https://github.com/cybergalactic/PythonVehicleSimulator

#include "OtterPawn.h"
#include "Components/InputComponent.h"
#include "Engine/Engine.h"
#include "DrawDebugHelpers.h"
#include <cmath>

// ─────────────────────────────────────────────────────────────────────────────
// Constructor  (kept minimal - Blueprint controls camera positioning)
// ─────────────────────────────────────────────────────────────────────────────

AOtterPawn::AOtterPawn()
{
    PrimaryActorTick.bCanEverTick = true;

    SceneRoot = CreateDefaultSubobject<USceneComponent>(TEXT("SceneRoot"));
    SetRootComponent(SceneRoot);

    HullMesh = CreateDefaultSubobject<UStaticMeshComponent>(TEXT("HullMesh"));
    HullMesh->SetupAttachment(SceneRoot);
    HullMesh->SetSimulatePhysics(false);

    SpringArm = CreateDefaultSubobject<USpringArmComponent>(TEXT("SpringArm"));
    SpringArm->SetupAttachment(SceneRoot);

    Camera = CreateDefaultSubobject<UCameraComponent>(TEXT("Camera"));
    Camera->SetupAttachment(SpringArm);

    AutoPossessPlayer = EAutoReceiveInput::Player0;
}

// ─────────────────────────────────────────────────────────────────────────────
// BeginPlay  -  compute hydrostatic coefficients from hull geometry
// ─────────────────────────────────────────────────────────────────────────────

void AOtterPawn::BeginPlay()
{
    Super::BeginPlay();
    ComputeHydrostatics();
    UE_LOG(LogTemp, Log, TEXT("OtterPawn: %s mode"),
        bUse3DOFMode ? TEXT("3DOF") : TEXT("6DOF"));
    UE_LOG(LogTemp, Log, TEXT("  Hydrostatics: GM_T=%.2f m, GM_L=%.2f m"),
        GM_T, GM_L);
    UE_LOG(LogTemp, Log, TEXT("  G33=%.1f N/m, G44=%.1f N.m/rad, G55=%.1f N.m/rad"),
        G33, G44, G55);
}

// ─────────────────────────────────────────────────────────────────────────────
// ComputeHydrostatics - derive G33, G44, G55 from the OTTER catamaran geometry
//
// Reference: Fossen (2021, Ch. 4.2) and MSS otter.py [2].
//
// Step 1 - Displaced volume (Archimedes at equilibrium):
//     V_nabla = Mass / FluidDensity
//
// Step 2 - Waterplane area (two rectangular pontoons):
//     A_wp = 2 * L_hull * B_pont
//
// Step 3 - Draft at equilibrium (W = B condition):
//     T = V_nabla / A_wp
//
// Step 4 - Transverse waterplane second moment of area:
//   Two pontoons, each at lateral offset y_pont from the centreline.
//   Parallel-axis theorem:
//     I_T = 2 * ( L_hull * B_pont^3 / 12  +  L_hull * B_pont * y_pont^2 )
//
// Step 5 - Longitudinal waterplane second moment of area:
//     I_L = 2 * ( B_pont * L_hull^3 / 12 )
//
// Step 6 - Metacentric radii (Fossen 2021, eq. 4.21):
//     BM_T = I_T / V_nabla
//     BM_L = I_L / V_nabla
//
// Step 7 - Metacentric heights (Fossen 2021, eq. 4.23):
//     GM_T = BM_T - BG
//     GM_L = BM_L - BG
//   where BG is the vertical distance between centre of buoyancy and
//   centre of gravity (BG > 0 when G is above B).
//
// Step 8 - Linear restoring coefficients (Fossen 2021, Table 4.1):
//     G33 = rho * g * A_wp            (heave stiffness)
//     G44 = rho * g * V_nabla * GM_T  (roll  stiffness)
//     G55 = rho * g * V_nabla * GM_L  (pitch stiffness)
//
// Numerical values for the nominal OTTER (m=55 kg, rho=1025):
//     V_nabla = 0.0537 m^3
//     A_wp    = 1.0 m^2
//     T       = 0.0537 m (5.4 cm draft)
//     I_T     = 0.161 m^4  ->  BM_T = 3.00 m
//     I_L     = 0.333 m^4  ->  BM_L = 6.20 m
//     GM_T    = 2.83 m,  GM_L = 6.03 m
//     G33     = 10056 N/m
//     G44     = 1527  N.m/rad
//     G55     = 3254  N.m/rad
//
// The Otter is extremely stable (GM_T = 2.83 m is very large for a 2 m
// catamaran), which is physically correct for this vessel design.
// ─────────────────────────────────────────────────────────────────────────────

void AOtterPawn::ComputeHydrostatics()
{
    float V_nabla = Mass / FluidDensity;
    float A_wp = 2.0f * L_hull * B_pont;

    float I_T = 2.0f * (L_hull * B_pont * B_pont * B_pont / 12.0f
        + L_hull * B_pont * y_pont * y_pont);
    float I_L = 2.0f * (B_pont * L_hull * L_hull * L_hull / 12.0f);

    float BM_T = I_T / V_nabla;
    float BM_L = I_L / V_nabla;

    GM_T = BM_T - BG;
    GM_L = BM_L - BG;

    G33 = FluidDensity * Gravity * A_wp;
    G44 = FluidDensity * Gravity * V_nabla * GM_T;
    G55 = FluidDensity * Gravity * V_nabla * GM_L;
}

// ─────────────────────────────────────────────────────────────────────────────
// Input
// ─────────────────────────────────────────────────────────────────────────────

void AOtterPawn::SetupPlayerInputComponent(UInputComponent* PlayerInputComponent)
{
    Super::SetupPlayerInputComponent(PlayerInputComponent);
    PlayerInputComponent->BindAxis("Throttle", this, &AOtterPawn::OnThrottle);
    PlayerInputComponent->BindAxis("Steering", this, &AOtterPawn::OnSteering);
}

void AOtterPawn::OnThrottle(float Value) { ThrottleInput = Value; }
void AOtterPawn::OnSteering(float Value) { SteeringInput = Value; }

// ─────────────────────────────────────────────────────────────────────────────
// Tick
// ─────────────────────────────────────────────────────────────────────────────

void AOtterPawn::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    PhysicsAccumulator += DeltaTime;
    while (PhysicsAccumulator >= PhysicsTimeStep)
    {
        IntegrateStep(PhysicsTimeStep);
        PhysicsAccumulator -= PhysicsTimeStep;
    }

    ApplyStateToTransform();

    if (bShowDebugHUD)    DrawDebugHUD();
    if (bDrawDebugVectors) DrawDebugVectors();
}

// ─────────────────────────────────────────────────────────────────────────────
// T200 thrust curve  (BlueRobotics performance data, quadratic fit)
// ─────────────────────────────────────────────────────────────────────────────

float AOtterPawn::T200ThrustFromCurrent(float I) const
{
    I = FMath::Clamp(I, -MaxCurrent, MaxCurrent);
    if (I >= 0.0f)
        return  0.006f * I * I + 0.347f * I;
    else
        return -0.005f * I * I + 0.276f * I;
}

// ─────────────────────────────────────────────────────────────────────────────
// Compute thrust generalised forces tau = [X, Y, Z, K, M, N]
// ─────────────────────────────────────────────────────────────────────────────

void AOtterPawn::ComputeThrust(float LeftInput, float RightInput,
    float& OutTauX, float& OutTauY, float& OutTauZ,
    float& OutTauK, float& OutTauM, float& OutTauN)
{
    float I_L = LeftInput * MaxCurrent;
    float I_R = RightInput * MaxCurrent;

    float F_L = T200ThrustFromCurrent(I_L);
    float F_R = T200ThrustFromCurrent(I_R);

    LastF_left = F_L;
    LastF_right = F_R;

    OutTauX = F_L + F_R;
    OutTauY = 0.0f;
    OutTauZ = 0.0f;
    OutTauK = 0.0f;
    OutTauM = -(F_L + F_R) * ThrusterVerticalOffset;   // nose-up pitch
    OutTauN = (F_R - F_L) * ThrusterLeverArm;         // differential yaw
}

// ─────────────────────────────────────────────────────────────────────────────
// Fossen Dynamics  (3DOF fallback + full 6DOF)
//
// Equation of motion:
//   M * nu_dot = tau - C(nu)*nu - D*nu - g(eta)
//
// 6DOF restoring forces g(eta) use the geometry-derived G33, G44, G55.
// Linearised for small attitude (|phi|,|theta| < 30 deg):
//     g_Z = G33 * z
//     g_K = G44 * phi
//     g_M = G55 * theta
// ─────────────────────────────────────────────────────────────────────────────

void AOtterPawn::ComputeFossenDynamics(
    float TauX, float TauY, float TauZ,
    float TauK, float TauM, float TauN,
    float& du, float& dv, float& dw,
    float& dp, float& dq, float& dr) const
{
    if (bUse3DOFMode)
    {
        float m11 = Mass + Xu_dot;
        float m22 = Mass + Yv_dot;
        float m66 = Iz + Nr_dot;

        float CRB_X = Mass * r * v;
        float CRB_Y = -Mass * r * u;

        float CA_X = Yv_dot * v * r;
        float CA_Y = -Xu_dot * u * r;
        float CA_N = (Xu_dot - Yv_dot) * u * v;

        du = (TauX - CRB_X - CA_X - Xu * u) / m11;
        dv = (TauY - CRB_Y - CA_Y - Yv * v) / m22;
        dr = (TauN - -CA_N - Nr * r) / m66;
        dw = 0.0f; dp = 0.0f; dq = 0.0f;
    }
    else
    {
        float m11 = Mass + Xu_dot;
        float m22 = Mass + Yv_dot;
        float m33 = Mass + Zw_dot;
        float m44 = Ix + Kp_dot;
        float m55 = Iy + Mq_dot;
        float m66 = Iz + Nr_dot;

        // Rigid-body Coriolis  (Fossen 2021, eq. 6.16, diagonal M_RB)
        float CRB_X = Mass * (v * r - w * q);
        float CRB_Y = Mass * (w * p - u * r);
        float CRB_Z = Mass * (u * q - v * p);
        float CRB_K = (Iy - Iz) * q * r;
        float CRB_M = (Iz - Ix) * p * r;
        float CRB_N = (Ix - Iy) * p * q;

        // Added-mass Coriolis  (Fossen 2021, eq. 6.33, diagonal M_A)
        float CA_X = Yv_dot * v * r - Zw_dot * w * q;
        float CA_Y = -Xu_dot * u * r + Zw_dot * w * p;
        float CA_Z = Xu_dot * u * q - Yv_dot * v * p;
        float CA_K = (Mq_dot - Nr_dot) * q * r;
        float CA_M = (Nr_dot - Kp_dot) * p * r;
        float CA_N = (Kp_dot - Mq_dot) * p * q;

        // Linear damping
        float D_X = Xu * u, D_Y = Yv * v, D_Z = Zw * w;
        float D_K = Kp * p, D_M = Mq * q, D_N = Nr * r;

        // Hydrostatic restoring forces  (from geometry via ComputeHydrostatics)
        float g_Z = G33 * Eta_z;    // heave spring
        float g_K = G44 * Phi;      // roll restoring
        float g_M = G55 * Theta;    // pitch restoring

        du = (TauX - CRB_X - CA_X - D_X) / m11;
        dv = (TauY - CRB_Y - CA_Y - D_Y) / m22;
        dw = (TauZ - CRB_Z - CA_Z - D_Z - g_Z) / m33;
        dp = (TauK - CRB_K - CA_K - D_K - g_K) / m44;
        dq = (TauM - CRB_M - CA_M - D_M - g_M) / m55;
        dr = (TauN - CRB_N - CA_N - D_N) / m66;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Integration step
// ─────────────────────────────────────────────────────────────────────────────

void AOtterPawn::IntegrateStep(float dt)
{
    float LeftInput = ThrottleInput - SteeringInput;
    float RightInput = ThrottleInput + SteeringInput;
    LeftInput = FMath::Clamp(LeftInput, -1.0f, 1.0f);
    RightInput = FMath::Clamp(RightInput, -1.0f, 1.0f);

    float TauX, TauY, TauZ, TauK, TauM, TauN;
    ComputeThrust(LeftInput, RightInput, TauX, TauY, TauZ, TauK, TauM, TauN);

    float du, dv, dw, dp, dq, dr;
    ComputeFossenDynamics(TauX, TauY, TauZ, TauK, TauM, TauN,
        du, dv, dw, dp, dq, dr);

    u += du * dt; v += dv * dt; w += dw * dt;
    p += dp * dt; q += dq * dt; r += dr * dt;

    UpdateKinematics(dt);
}

// ─────────────────────────────────────────────────────────────────────────────
// Kinematics: eta_dot = J(eta) * nu  (Fossen 2021, eq. 2.40)
// ─────────────────────────────────────────────────────────────────────────────

void AOtterPawn::UpdateKinematics(float dt)
{
    if (bUse3DOFMode)
    {
        float sPsi = FMath::Sin(Psi);
        float cPsi = FMath::Cos(Psi);
        Eta_x += (u * cPsi - v * sPsi) * dt;
        Eta_y += (u * sPsi + v * cPsi) * dt;
        Psi += r * dt;
    }
    else
    {
        // Clamp pitch to avoid Gimbal-lock singularity in T matrix
        float ThetaSafe = FMath::Clamp(Theta,
            FMath::DegreesToRadians(-89.0f),
            FMath::DegreesToRadians(89.0f));

        float sPhi = FMath::Sin(Phi), cPhi = FMath::Cos(Phi);
        float sTh = FMath::Sin(ThetaSafe), cTh = FMath::Cos(ThetaSafe);
        float sPsi = FMath::Sin(Psi), cPsi = FMath::Cos(Psi);
        float tTh = sTh / cTh;

        // ZYX Euler rotation matrix R(phi,theta,psi)  (body -> NED)
        float R11 = cPsi * cTh;
        float R12 = cPsi * sTh * sPhi - sPsi * cPhi;
        float R13 = cPsi * sTh * cPhi + sPsi * sPhi;
        float R21 = sPsi * cTh;
        float R22 = sPsi * sTh * sPhi + cPsi * cPhi;
        float R23 = sPsi * sTh * cPhi - cPsi * sPhi;
        float R31 = -sTh;
        float R32 = cTh * sPhi;
        float R33 = cTh * cPhi;

        float x_dot = R11 * u + R12 * v + R13 * w;
        float y_dot = R21 * u + R22 * v + R23 * w;
        float z_dot = R31 * u + R32 * v + R33 * w;

        // Angular velocity transformation T(phi,theta)
        float phi_dot = p + (q * sPhi + r * cPhi) * tTh;
        float theta_dot = q * cPhi - r * sPhi;
        float psi_dot = (q * sPhi + r * cPhi) / cTh;

        Eta_x += x_dot * dt;
        Eta_y += y_dot * dt;
        Eta_z += z_dot * dt;
        Phi += phi_dot * dt;
        Theta += theta_dot * dt;
        Psi += psi_dot * dt;

        Phi = FMath::Clamp(Phi, FMath::DegreesToRadians(-45.0f), FMath::DegreesToRadians(45.0f));
        Theta = FMath::Clamp(Theta, FMath::DegreesToRadians(-45.0f), FMath::DegreesToRadians(45.0f));
    }

    while (Psi > PI) Psi -= 2.0f * PI;
    while (Psi < -PI) Psi += 2.0f * PI;
}

// ─────────────────────────────────────────────────────────────────────────────
// Apply state to UE5 actor transform  (NED m -> UE5 cm, flip z)
// ─────────────────────────────────────────────────────────────────────────────

void AOtterPawn::ApplyStateToTransform()
{
    float UE_X = Eta_x * 100.0f;
    float UE_Y = Eta_y * 100.0f;
    float UE_Z = -Eta_z * 100.0f;

    SetActorLocation(FVector(UE_X, UE_Y, UE_Z));

    if (bUse3DOFMode)
    {
        SetActorRotation(FRotator(0.0f, FMath::RadiansToDegrees(Psi), 0.0f));
    }
    else
    {
        float RollDeg = FMath::RadiansToDegrees(Phi);
        float PitchDeg = -FMath::RadiansToDegrees(Theta);
        float YawDeg = FMath::RadiansToDegrees(Psi);
        SetActorRotation(FRotator(PitchDeg, YawDeg, RollDeg));
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Debug HUD
// ─────────────────────────────────────────────────────────────────────────────

void AOtterPawn::DrawDebugHUD()
{
    if (!GEngine) return;

    FString HUDText;
    if (bUse3DOFMode)
    {
        HUDText = FString::Printf(
            TEXT("=== OTTER 3DOF ===\n")
            TEXT("Position  x=%.2f  y=%.2f m   yaw=%.1f deg\n")
            TEXT("Velocity  u=%.3f  v=%.3f m/s   r=%.3f rad/s\n")
            TEXT("Thrust    L=%.1f N   R=%.1f N"),
            Eta_x, Eta_y, FMath::RadiansToDegrees(Psi),
            u, v, r,
            LastF_left, LastF_right);
    }
    else
    {
        HUDText = FString::Printf(
            TEXT("=== OTTER 6DOF ===\n")
            TEXT("Position  x=%.2f  y=%.2f  z=%.3f m\n")
            TEXT("Attitude  roll=%.2f  pitch=%.2f  yaw=%.1f deg\n")
            TEXT("Linear    u=%.3f  v=%.3f  w=%.4f m/s\n")
            TEXT("Angular   p=%.3f  q=%.3f  r=%.3f rad/s\n")
            TEXT("Thrust    L=%.1f N   R=%.1f N\n")
            TEXT("GM_T=%.2f m  GM_L=%.2f m"),
            Eta_x, Eta_y, Eta_z,
            FMath::RadiansToDegrees(Phi),
            FMath::RadiansToDegrees(Theta),
            FMath::RadiansToDegrees(Psi),
            u, v, w, p, q, r,
            LastF_left, LastF_right,
            GM_T, GM_L);
    }

    GEngine->AddOnScreenDebugMessage(1, 0.0f, FColor::Green, HUDText);
}

// ─────────────────────────────────────────────────────────────────────────────
// Debug vectors
// ─────────────────────────────────────────────────────────────────────────────

void AOtterPawn::DrawDebugVectors()
{
    UWorld* World = GetWorld();
    if (!World) return;

    FVector Origin = GetActorLocation();
    FVector Forward = GetActorForwardVector();
    FVector Right = GetActorRightVector();

    DrawDebugLine(World, Origin,
        Origin + Forward * u * 100.0f * 5.0f,
        FColor::Blue, false, -1.0f, 0, 3.0f);

    DrawDebugLine(World, Origin,
        Origin + Right * v * 100.0f * 5.0f,
        FColor::Yellow, false, -1.0f, 0, 3.0f);

    DrawDebugLine(World,
        Origin + Right * (-ThrusterLeverArm * 100.0f),
        Origin + Right * (-ThrusterLeverArm * 100.0f) + Forward * LastF_left * 5.0f,
        FColor::Red, false, -1.0f, 0, 4.0f);

    DrawDebugLine(World,
        Origin + Right * (ThrusterLeverArm * 100.0f),
        Origin + Right * (ThrusterLeverArm * 100.0f) + Forward * LastF_right * 5.0f,
        FColor::Green, false, -1.0f, 0, 4.0f);
}