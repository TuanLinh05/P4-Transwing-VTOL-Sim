# Estimated P4 Transwing Gazebo model

This model turns the animated 3D asset in `Ref/P4_clone` into an articulated
Gazebo Harmonic model. It is suitable for geometry, joint, actuator-topic and
early controller-integration work. It is **not** a validated digital twin and
must not be used for real-aircraft loads, performance or flight-safety
decisions.

## Evidence and assumptions

| Quantity or feature | Value used | Status |
| --- | ---: | --- |
| Maximum takeoff mass | model 40.5 kg; UAFM 40.37 kg; 2025 brochure 41 kg | published baseline, rounded model |
| Wingspan | 3.99288 m (13.1 ft) | PteroDynamics X-P4 UAFM |
| Wing area | 0.836 m² (9 ft²) | PteroDynamics X-P4 UAFM |
| Fuselage length | 1.951 m (6.4 ft) | PteroDynamics X-P4 UAFM |
| Maximum propeller speed | 733.04 rad/s (7000 rpm) | PteroDynamics X-P4 UAFM |
| Four ailerons, two ruddervators, four propellers | modeled | PteroDynamics X-P4 UAFM and 3D asset |
| Fold travel | 2.05943 rad (117.997°) | measured from the glTF animation |
| Fold axes and hinge locations | per-side values in metadata | derived from the glTF animation |
| Rotor and wing locations | per-side values in metadata | derived from the glTF node hierarchy |
| Mass distribution and inertia tensors | total mass constrained to 40.5 kg | estimated |
| Collision geometry | boxes and cylinders | estimated |
| Lift/drag coefficients and centers of pressure | SDF values | estimated |
| Open-loop propeller thrust coefficient | 2.8e-4 N/(rad/s)² | estimated |
| ArduPilot SITL effective thrust coefficient | 4.2e-4 N/(rad/s)² | calibrated to ground takeoff |
| ArduPilot SITL hover output | 0.59–0.60 | measured in QLOITER at 40.5 kg |
| Folded-hover rotor centroid / model CG | x = -0.3720 m / -0.3707 m | simulation-balanced estimate |
| Hover feed-forward speeds | 500 rad/s inner, 730 rad/s outer | estimated open-loop trim |
| Motor | Hacker Q80-7M-V2, 210 KV, 5.5 kW, 1.035 kg | PteroDynamics X-P4 UAFM |
| Propeller | custom folding 22 × 15 in | PteroDynamics X-P4 UAFM |
| Fixed-wing propulsion | two of four propulsors active | PteroDynamics X-P4 UAFM; not yet modeled |

Primary dimensional reference:
[PteroDynamics X-P4 Unmanned Aircraft Flight Manual](https://downloads.regulations.gov/FAA-2024-2404-0001/attachment_10.pdf).
The structured source ledger is `Simulation/config/p4_reference.json`.

## Ground-takeoff calibration

The first ArduPilot-coupled blade-section model reused the open-loop
`2.8e-4 N/(rad/s)^2` estimate. During a real ground takeoff command, the two
outer rotors approached saturation while the four measured rotor speeds
produced only about 291 N, below the 40.5 kg model weight of about 397 N.
ArduPilot consequently reported `Potential VTOL Thrust Loss (4)` and the
aircraft remained on the ground.

For the closed-loop SITL model only, each of the two representative sections
per propeller is therefore calibrated from 0.00414 to 0.00621 m². This gives
an effective coefficient near `4.2e-4 N/(rad/s)^2`. A ground-start QLOITER
test then reached 4.64 m for a 5 m request and held 4.65–4.66 m at about
58–59% throttle. This calibration proves simulation controllability; it is
not a measured production X-P4 propeller curve.

The QLOITER completion band is scaled from 0.4 m at a 5 m request to a
maximum of 0.8 m for higher requests. This prevents the RC climb override from
remaining active while it chases a small simulated barometric residual. In a
20 m ground-start test, the operation handed over at 19.22 m and the final
hover stayed within 0.15 m over five seconds; pitch in the final pre-hover
window was about -9.6 to +7.5 degrees. Larger short pitch excursions can
still occur during lift-off because the folded-wing centre of gravity and
inertia are estimates.

## Folded-hover centre-of-gravity calibration

The first mass estimate placed the complete folded-aircraft CG at
`x=-0.4967 m`, while the equal-thrust centroid of the four rotor axes is
`x=-0.3720 m`. DataFlash log `00000010.BIN` showed the resulting imbalance:
the aft motor pair averaged about 1869 PWM during climb and 1854 PWM near the
top, versus 1572 and 1567 PWM for the forward pair. Lift-off pitch ranged from
-13.3 to +30.8 degrees and the aft pair approached saturation.

Because the production mass distribution is not published, the estimated
fuselage/payload CG is moved from `x=-0.20 m` to `x=+0.10 m`. Total mass remains
40.5 kg and the resulting model CG is `x=-0.3707 m`, within about 1.3 mm of the
rotor centroid. This is a hover-control calibration, not a measured X-P4 CG.
With the static motor bias removed, the provisional pitch-rate integrator is
reduced from the ArduPlane default 0.25 to 0.12 to avoid carrying an obsolete
correction through each low-frequency pitch reversal. P and D are left at
their defaults for this first isolated controller change.

## Transition evidence from the 2021 presentation

At 00:09, the slide “Transition Mechanics – 50%” states 10–20 m/s airspeed,
angle of attack below 20 degrees, meaningful power reduction from ETL plus
wing lift after 30% transition, active control surfaces, elevator-led pitch
control and aileron roll damping.

The current surrogate maps normalized 50% tilt to `-1.029716127 rad` and
retains it as a presentation reference. `Q_TILT_MAX=20` is now a shallower
airspeed-wait hold, while the post-airspeed TIMER phase commands the complete
fold. The 27 m/s floor follows the published 26.75 m/s X-P4 stall speed. The
early baseline used `Q_ASSIST_SPEED=32 m/s` as a transition margin, but logs
`00000043.BIN` and `00000044.BIN` showed that a pitot-speed dip during a turn
could re-enter transition and move the synchronized wings. The current
identification baseline uses `Q_ASSIST_SPEED=-1`; deliberate QRTL/QLOITER/QLAND
recovery and normal transitions remain available. This is a simulation safety
calibration, not a published P4 transition speed. The estimated wing model
stalls at about 16 degrees. These are useful
consistency checks, not proof of
the production X-P4 transition envelope. The UAFM separately publishes a
52-knot fixed-wing stall speed.

The Gazebo/SITL pitot path is configured as `ARSPD_TYPE=100`. `Q_OPTIONS=4096`
is retained as a guard for binaries that classify that input as synthetic, but
it did not suppress the observed speed-QAssist event in the current build.
Automatic speed assist is consequently disabled with `Q_ASSIST_SPEED=-1`
during airframe identification; deliberate `ABORT`, `QLOITER` and `QLAND`
recovery commands are unaffected.
Angle-based assist is disabled with `Q_ASSIST_ANGLE=0` for the same reason:
`00000038.BIN` recorded `Angle assist` at roll/pitch excursions, followed by
MotorTilt reversing from 90° to 20° and repeatedly restarting transition.

The SITL pitot is selected for fixed-wing TECS with `ARSPD_USE=1`. DataFlash
tests `00000015.BIN` and `00000016.BIN` also verified V-tail direction:
reversing both outputs made the post-airspeed pitch-down divergence occur
earlier, so the original non-reversed mixer direction is retained. The
remaining longitudinal trim error is in the estimated moving-wing/rotor
aerodynamics, not a production-aircraft calibration.

`00000044.BIN` further showed that the QRTL heading capture requested the
full 45-degree bank limit and the model overshot it to -59 degrees. The
surrogate now uses a 25-degree bank ceiling, `NAVL1_PERIOD=35`,
`NAVL1_DAMPING=0.90`, a 45 deg/s roll-rate limit and reduced/damped roll PID
values. This preserves the verified aileron direction while keeping QRTL turns
inside the current aerodynamic model's stable envelope.

At about 50% fold, the estimated rotor thrust line creates a pitch-down moment
arm near 0.06 m as the rotor centroid moves forward of the instantaneous CG.
At the simulated thrust level this is roughly 24 N m, while the original
0.10 m^2-per-side V-tail estimate supplied only about 14--18 N m for the
four-degree correction seen in the log. The effective aerodynamic area is
therefore raised to 0.18 m^2 per side. This remains below the geometric
0.72 x 0.34 m collision-panel footprint (0.2448 m^2 per side) and is an
authority calibration, not a measured production tail area.

With the larger effective tail, `00000018.BIN` held pitch above the attitude
abort limit but still lost altitude: the front motor pair reached about
1950 PWM while both V-tail outputs moved only to about 1548 PWM. ArduPlane's
default `MIXING_GAIN=0.5` was halving the common elevator command before the
V-tail functions reached the two surfaces. The SITL baseline uses
`MIXING_GAIN=1.0`; servo limits continue to bound each ruddervator.

The deeper cause was a coordinate-frame error in the main-wing LiftDrag
configuration. Gazebo Sim 8 defines `cp` relative to a link's inertial center
of gravity, but the original SDF supplied the intended link-frame positions
`(0.02, +/-0.72, 0)`. Subtracting each wing CoG
`(0.15, +/-0.72, -0.03)` gives the correct plugin value
`(-0.13, 0, 0.03)`. This puts the main-wing force at the intended aerodynamic
center instead of ahead of the moving wing CoG with an extra spanwise offset.

After the wing lift-sign and incidence corrections, `00000028.BIN` briefly
reduced vertical speed to almost zero at full fold, but pitch then diverged to
about -11.8 degrees while the controller requested +15 degrees. The effective
V-tail area is therefore raised from 0.18 to the full geometric collision
panel footprint, 0.2448 m2 per side. This is the upper geometry-consistent
surrogate bound and restores pitch authority without inventing extra tail
area.

`00000029.BIN` then showed that the physical tail still had unused travel
while the stock fixed-wing pitch loop accumulated correction too slowly:
near the safety abort, PIDP reported roughly 0.8 percent P, 9.6 percent FF and
14.6 percent I with pitch near -10 degrees. The SITL identification baseline
uses `PTCH_RATE_P=0.08` and equal `PTCH_RATE_I=PTCH_RATE_FF=0.50`, following
ArduPlane's recommendation that pitch I match FF. These are surrogate gains
and remain bounded by the existing servo limits and transition abort monitor.

DataFlash `00000020.BIN` then exposed a separate controller-envelope mismatch.
The model accelerated from the 27 m/s transition threshold to about 44 m/s,
but ArduPlane still had its default `SCALING_SPEED=15`. Its logged fixed-wing
speed-scaling factor fell to 0.35-0.54, leaving the V-tail at only about eight
percent elevator while desired pitch rose to eight degrees. The SITL baseline
therefore now sets `SCALING_SPEED=34`, equal to the modeled cruise airspeed.

The first full-length FBWB test with the corrected transition-assist threshold
(`00000036.BIN`) still showed a slow energy-exchange oscillation: TECS varied
throttle roughly 21–84 percent while altitude and speed corrected out of
phase. The simulation therefore uses height-biased, slower TECS settings
(`TECS_SPDWEIGHT=0.50`, `TECS_TIME_CONST=8`, `TECS_THR_DAMP=0.80`, and
`TECS_PITCH_MIN=-5 deg`). They only stabilize this estimated model; they are
not inferred production P4 flight-control data.

With gain scaling corrected, `00000021.BIN` reached `Transition FW done` and
increased common V-tail output to about 20 percent, but the final 10 deg/s
unfold removed vertical rotor support while pitch fell from about -4 to
-10 degrees. A 5 deg/s experiment (`00000022.BIN`) was worse: it lost nearly
20 m while only reaching mid travel even though pitch remained above
-5.2 degrees. Initial SITL identification therefore uses
`Q_TILT_RATE_DN=20 deg/s` to cross that aerodynamic dead zone promptly after
the 27 m/s threshold.

`00000026.BIN` exposed the remaining full-wing lift failure. Gazebo Sim's
LiftDrag implementation uses `alpha = alpha0` when velocity is parallel to
the configured forward vector; it does not interpret `a0` as a conventional
negative zero-lift angle. The original `a0=-0.035 rad` therefore generated
negative CL at zero geometric incidence. `00000027.BIN` confirmed the sign
correction but also showed that `a0=+0.035 rad` supplied only about 95 N at
31 m/s and zero geometric incidence, far below the approximately 397 N weight
of the 40.5 kg model. Main wings and ailerons therefore use `a0=+0.11 rad`.
With the modeled 0.836 m2 area and lift slopes, this leaves roughly two degrees
of cruise angle of attack for the wing to support the full weight. It is an
effective incidence/camber calibration, not a measured X-P4 airfoil datum.

The complete dashboard/API path passed the bounded transition smoke test in
DataFlash `00000030.BIN`: 29.3 m start altitude, 15.9 m minimum altitude,
42.8 m/s maximum groundspeed, 0.1 degree maximum absolute roll and
14.6 degrees maximum absolute pitch. ArduPilot reported both transition
airspeed reached and fixed-wing transition done, the model then held FBWB for
20 seconds, and the test recovered through QLOITER/QLAND before stopping the
stack.

## Asset conversion

`tools/prepare_p4_meshes.py` reads the glTF buffers, node transforms, skin-free
mesh primitives and animation channels directly. It separates the asset into:

- fuselage and V-tail;
- left and right moving wing assemblies, including their nacelles;
- four propeller visuals.

The converter exports textured OBJ files, maps the source axes to Gazebo FLU
(`+X` forward, `+Y` left, `+Z` up), and applies one uniform scale so the
unfolded mesh span is 3.99288 m. Its measured transforms are stored in
`meshes/p4_geometry_metadata.json`.

The source asset is visually plausible and contains a useful folding animation,
but its proportions are not an exact match for the flight-manual dimensions.
After span calibration, its visual fuselage is about 2.45 m long instead of the
documented 1.95 m. The physics collision body therefore uses the documented
fuselage length while the detailed mesh is retained as the visual.

Regenerate meshes from the source asset on Windows:

```powershell
python Simulation/gazebo/tools/prepare_p4_meshes.py
```

## Model convention

The safe startup state is:

- `hover = 0 rad`: wings folded and propeller thrust axes approximately upward;
- `transition = -1.029716127 rad`;
- `cruise = -2.059432253 rad`: wings extended and propeller thrust axes forward.

The mesh, wing inertia, nacelles, rotors and attached ailerons move together.
This is different from a tiltrotor-only surrogate: the complete wing assembly
changes pose.

Actuator indices follow the flight-manual ordering:

1. right inner;
2. right outer;
3. left outer;
4. left inner.

## Running in Gazebo Harmonic / WSL

Articulation world, with gravity disabled:

```bash
# Run from the repository root, wherever it was cloned.
bash Simulation/gazebo/run_p4_articulation_wsl.sh
```

Gravity and estimated propulsion/aerodynamics:

```bash
bash Simulation/gazebo/run_p4_flight_wsl.sh
```

In a second terminal:

```bash
bash Simulation/gazebo/test_p4_commands.sh cruise
bash Simulation/gazebo/test_p4_commands.sh transition
bash Simulation/gazebo/test_p4_commands.sh hover
bash Simulation/gazebo/test_p4_commands.sh demo
bash Simulation/gazebo/test_p4_commands.sh motors-idle
bash Simulation/gazebo/test_p4_commands.sh motors-stop
```

`motors-hover` is a feed-forward force/direction check, not a hover controller:

```bash
bash Simulation/gazebo/test_p4_commands.sh hover
bash Simulation/gazebo/test_p4_commands.sh motors-hover
```

Stop the motors promptly when running this manually. Without a closed-loop
attitude controller, small modeling and trim errors grow and the vehicle will
eventually depart from hover.

## Verification

```bash
bash Simulation/gazebo/smoke_test_p4_wsl.sh
bash Simulation/gazebo/smoke_test_p4_flight_wsl.sh
```

The articulation test checks both fold endpoints, four motor systems, mesh
loading and joint-state publication. The flight test is intentionally short:
it verifies upward thrust and that the fold joints remain locked under rotor
load before open-loop attitude error becomes dominant.

## Recommended next step

ArduPilot SITL is now connected. The next identification work is:

1. validate the SITL pitot/airspeed path and transition scheduler;
2. model real two-motor cruise selection after full transition;
3. identify closed-loop roll damping through the 30–50% transition region;
4. add linear-actuator position feedback, homing and synchronization faults;
5. identify coefficients from propeller, wind-tunnel or flight data;
6. validate center of gravity and the full inertia tensor.
