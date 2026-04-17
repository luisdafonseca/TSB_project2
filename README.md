# TSB Autonomous Systems — Phase 2 Simulator

**Técnico Solar Boat | Autonomous Systems Recruitment**  
OTTER USV Marine Simulator — Unreal Engine 5.3.2 (C++)

---

## Overview

This project is a real-time marine simulator for the Maritime Robotics OTTER USV, built with Unreal Engine 5.3.2 and C++. The physics are based on Fossen's equations of motion (Handbook of Marine Craft Hydrodynamics and Motion Control, 2nd Ed., Wiley 2021), with support for both 3-DOF and 6-DOF modes.

The OTTER is a 2-metre, 55 kg catamaran USV designed for hydrographic surveying in sheltered waters. It is driven by two T200 BlueRobotics thrusters in a differential thrust configuration.

---

## What's Implemented

### Core
- 3-DOF Fossen dynamics (surge, sway, yaw)
- Nonlinear T200 thruster model calibrated from official BlueRobotics performance data
- WASD keyboard control via differential thrust
- Ocean plane environment with sky atmosphere

### Sensors (Recommended deliverable)
- AHRS — attitude estimation with Gaussian noise and configurable failure modes
- GPS — NED position with noise, signal latency, and dropout simulation
- LiDAR — 180-ray planar scan with configurable range and noise
- All sensor data logged to CSV at `Saved/Logs/SensorLog_YYYYMMDD_HHMMSS.csv`
- Real-time HUD showing vessel state during Play

### 6-DOF (Optional bonus)
The simulator can be switched to full 6-DOF by unchecking `bUse3DOFMode` in the Blueprint — no recompilation needed. The 6-DOF mode adds heave, roll, and pitch with hydrostatic restoring forces derived from the OTTER hull geometry. More detail in the Physics section below.

---

## Physics

### The Fossen Model

The full Fossen equation of motion for a marine vessel is (Fossen 2021, eq. 6.1):

```
M * nu_dot + C(nu)*nu + D(nu)*nu + g(eta) = tau + tau_wind + tau_wave + tau_current
```

The right-hand side has four terms:
- `tau` — thruster forces (implemented)
- `tau_wind` — wind loads
- `tau_wave` — wave excitation forces
- `tau_current` — ocean current forces

The last three are not included in this simulator. Here is why:

**Wind** forces require the vessel's aerodynamic resistance coefficients, which are not part of the provided OTTER parameter set and would need dedicated wind tunnel or CFD characterisation. The OTTER is designed for sheltered water surveys where wind is a secondary effect.

**Wave** excitation forces (both first-order and drift) require hydrodynamic RAO (Response Amplitude Operator) data, typically obtained by running the hull geometry through tools like WAMIT or ShipX. These are not available for the OTTER without a dedicated hydrodynamic study. The project specification also explicitly states that wave generation is not expected.

**Ocean current** can be added by replacing the body-frame velocity `nu` with the relative velocity `nu_r = nu - nu_c`, where `nu_c` is the current velocity rotated into the body frame. This is a clean extension but falls outside the scope of the required and optional deliverables.

### What Is Solved

The simulator integrates the following reduced form at 200 Hz (fixed timestep of 0.005 s):

```
(M_RB + M_A) * nu_dot = tau - (C_RB(nu) + C_A(nu))*nu - D*nu - g(eta)
```

`M_RB` and `M_A` are both diagonal matrices. All mass, inertia, damping and added-mass values come directly from the official ASV Physics Parameters table provided in the specification. The Coriolis matrices `C_RB` and `C_A` are computed analytically following Fossen (2021), eq. 6.16 and 6.33 respectively.

### Hydrostatics (6-DOF only)

The restoring force vector `g(eta)` uses linearised hydrostatics valid for small roll and pitch angles — a reasonable assumption for a surface vessel operating in calm conditions. The stiffness coefficients G33, G44 and G55 are not arbitrary: they are calculated from the OTTER catamaran geometry following the standard naval architecture formulas in Fossen (2021, Ch. 4.2), using the same approach as the official MSS `otter.py` model.

The key geometric inputs are the pontoon dimensions (L = 2.0 m, B_pont = 0.25 m) and lateral offset (y_pont = 0.395 m), taken directly from Fossen's published model. From these, the waterplane second moments of area give:

| Quantity | Value |
|---|---|
| Waterplane area A_wp | 1.0 m² |
| Transverse metacentric height GM_T | 2.83 m |
| Longitudinal metacentric height GM_L | 6.04 m |
| Heave stiffness G33 | 10056 N/m |
| Roll stiffness G44 | 1528 N·m/rad |
| Pitch stiffness G55 | 3258 N·m/rad |

The large metacentric heights (GM_T = 2.83 m is unusually high for a 2 m vessel) are physically correct — they reflect the extreme transverse stability of the catamaran hull form, with the two pontoons separated by nearly 0.8 m. In practice this means the OTTER barely inclines under its own thrusters, which is consistent with observed real-world behaviour.

### T200 Thrust Curve

The quadratic fit to the official BlueRobotics current-to-thrust data, accounting for the forward/reverse asymmetry:

- Forward: `F = 0.006 * I^2 + 0.347 * I` [N]
- Reverse: `F = -0.005 * I^2 - 0.276 * |I|` [N]

Current is clamped to the operational range of ±20 A.

---

## Project Structure

```
TSB_project2/
├── Source/TSB_project2/
│   ├── Public/
│   │   └── OtterPawn.h          # vessel pawn, Fossen 3DOF/6DOF
│   └── Private/
│       ├── OtterPawn.cpp        # dynamics, kinematics, thruster model
│       ├── SensorsComponent.h   # AHRS / GPS / LiDAR declarations
│       └── SensorsComponent.cpp # sensor simulation with noise and failure
├── Content/
│   ├── HolodeckContent/         # OTTER mesh and materials
│   ├── BP_Otter.uasset          # Blueprint (parent class: OtterPawn)
│   └── SimLevel.umap            # main simulation level
└── Saved/Logs/
    └── SensorLog_*.csv
```

---

## Build & Run

### Requirements
- Unreal Engine 5.3.2
- Visual Studio 2022 with the C++ game development workload
- Windows 10 or 11

### Steps

1. Extract or clone the repository
2. Right-click `TSB_project2.uproject` → **Generate Visual Studio project files**
3. Open `TSB_project2.sln` in Visual Studio
4. Set the configuration to **Development Editor | Win64**
5. Close Unreal Engine if it is open
6. Build with `Ctrl+Shift+B`
7. Open `TSB_project2.uproject` and press **Play**

### Controls

| Key | Action |
|---|---|
| W | Forward thrust |
| S | Reverse thrust |
| A | Turn left |
| D | Turn right |

---

## Configuration

All physical parameters and simulation options are exposed as editable properties in the BP_Otter Blueprint (Class Defaults tab). No code changes are needed to tune parameters. The most useful ones:

- **bUse3DOFMode** — uncheck to enable 6-DOF
- **PhysicsTimeStep** — integration step size (default 0.005 s)
- **bShowDebugHUD** — toggle on-screen state readout
- **bDrawDebugVectors** — toggle velocity and thrust arrows in the viewport

---

## References

1. T. I. Fossen, *Handbook of Marine Craft Hydrodynamics and Motion Control*, 2nd Ed., Wiley, 2021.
2. T. I. Fossen, `otter.py` — Python Vehicle Simulator (MSS). https://github.com/cybergalactic/PythonVehicleSimulator
3. Maritime Robotics, OTTER USV datasheet. https://www.maritimerobotics.com/otter
4. BlueRobotics, T200 Thruster performance data. https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r3-rp/