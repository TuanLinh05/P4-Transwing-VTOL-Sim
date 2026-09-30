# Transwing models for Gazebo

## P4 asset-based estimated model

`Ref/P4_clone` is not just a single decorative mesh: its glTF hierarchy contains
separable wings, four propellers and a symmetric folding animation. The project
now includes an asset-based Gazebo model built from that information:

- textured fuselage, moving wing and propeller meshes;
- two mirrored oblique fold joints measured from the animation;
- four rotor force models that move with the wings;
- four ailerons and two ruddervators;
- estimated lift/drag, MTOW mass distribution and inertias;
- IMU, NavSat and air-pressure sensors;
- zero-gravity articulation and gravity-enabled flight worlds.

Detailed provenance, limitations and commands are in
[`gazebo/P4_ESTIMATION_NOTES.md`](gazebo/P4_ESTIMATION_NOTES.md).

## ArduPlane QuadPlane and Control Station

The separate `p4_transwing_ardupilot` variant integrates the estimated P4
geometry with the official `ardupilot_gazebo` JSON plugin:

- Quad-X Motor1–4 mapping for the four folded hover rotors;
- one common MotorTilt output duplicated to both synchronized fold joints;
- one aileron output duplicated with mirrored signs to four surfaces;
- ArduPlane V-tail left/right outputs;
- isolated P4 EEPROM and DataFlash state;
- the MiniHawk-derived local dashboard, MAVLink mission tools and Windows
  one-click launcher.

Start from Windows with
`P4-Transwing-Control-Station.cmd`, or follow [`SETUP.md`](SETUP.md).

Run the disarmed end-to-end check from WSL:

```bash
# Run from the repository root, wherever it was cloned.
bash Simulation/gazebo/smoke_test_p4_ardupilot_wsl.sh
```

Only the zero-wind, zero-payload `baseline` case is enabled. Experiment
campaigns and live weather are locked until the provisional aerodynamic,
rotor and transition parameters are identified.

Run the articulation model from WSL:

```bash
bash Simulation/gazebo/run_p4_articulation_wsl.sh
```

Then, in a second terminal:

```bash
bash Simulation/gazebo/test_p4_commands.sh cruise
bash Simulation/gazebo/test_p4_commands.sh transition
bash Simulation/gazebo/test_p4_commands.sh hover
bash Simulation/gazebo/test_p4_commands.sh demo
```

Run the gravity-enabled model with:

```bash
bash Simulation/gazebo/run_p4_flight_wsl.sh
```

This version is an engineering estimate, not a validated P4 digital twin.
`motors-hover` verifies force and direction but does not replace an attitude
controller.

## Primitive surrogate

This directory contains a primitive-geometry VTOL model inspired by the
kinematic principle of a Transwing. It is intentionally not a dimensional copy
of the PteroDynamics P4.

## What is implemented

- Two movable outer wing panels.
- One mirrored oblique revolute joint per wing, so both panels fold aft.
- Four propellers rigidly attached to the moving wings.
- Ailerons attached to the moving wing panels.
- Conventional elevator and rudder.
- IMU and NavSat sensors.
- A zero-gravity articulation world and command script.
- No STL or external mesh dependencies.

The hinge axis is chosen so one 120-degree motion performs both required
transformations:

| Configuration | Fold angle | Wing orientation | Propeller thrust axis |
| --- | ---: | --- | --- |
| Cruise | 0 deg | Horizontal, spanwise | Forward |
| Transition | 60 deg | Oblique | Oblique |
| Hover | 120 deg | Vertical, along fuselage | Upward |

This is the main difference from merely renaming a tiltrotor angle: the wing,
rotor positions, rotor axes, mass distribution, and attached control surfaces
all move as one physical assembly.

## Run from WSL

All launchers resolve their location automatically. Run them from the
repository root; moving or renaming the repository needs no source edits.

Terminal 1:

```bash
bash Simulation/gazebo/run_articulation_wsl.sh
```

Press **Play** in Gazebo.

Terminal 2:

```bash
bash Simulation/gazebo/test_transwing_joints.sh cruise
bash Simulation/gazebo/test_transwing_joints.sh transition
bash Simulation/gazebo/test_transwing_joints.sh hover
bash Simulation/gazebo/test_transwing_joints.sh demo
```

Control-surface checks:

```bash
bash Simulation/gazebo/test_transwing_joints.sh roll-left
bash Simulation/gazebo/test_transwing_joints.sh roll-right
bash Simulation/gazebo/test_transwing_joints.sh pitch-up
bash Simulation/gazebo/test_transwing_joints.sh yaw-left
bash Simulation/gazebo/test_transwing_joints.sh neutral
```

Headless smoke test:

```bash
bash Simulation/gazebo/smoke_test_wsl.sh
```

## Scope and next milestone

The current world validates geometry, hierarchy, joint directions, joint
limits, and actuator topics. Gravity is intentionally disabled and rotor thrust
is not yet enabled.

The next milestone is an ArduPlane/QuadPlane wrapper with:

1. LiftDrag terms attached to the moving wing links.
2. Four calibrated rotor force models.
3. Fold-angle output mapping.
4. A transition schedule based on airspeed and fold angle.
5. A custom or generalized mixer for the folded rotor positions.

Do not use this first model to estimate P4 range, endurance, structural loads,
or the real transition corridor.
