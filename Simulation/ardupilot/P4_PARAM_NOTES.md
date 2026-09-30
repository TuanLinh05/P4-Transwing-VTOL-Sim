# P4 Transwing SITL parameter notes

`p4_transwing_sitl.param` is a simulation-only starting point derived from the
MiniHawk ArduPlane/QuadPlane setup and the P4 asset-based Gazebo model.

## Intentional changes from MiniHawk

| Area | MiniHawk | P4 Transwing |
| --- | --- | --- |
| VTOL frame | TRI | Quad-X |
| Tilting motors | front two | all four |
| Tilt mask | 3 | 15 |
| Tilt type | vectored yaw | continuous |
| Tilt outputs | independent left/right | one synchronized wing-fold output |
| Fixed-wing roll | two elevons | four ailerons driven from one output |
| Fixed-wing pitch/yaw | elevons | two ArduPlane V-tail outputs |
| Hover thrust fraction | MiniHawk-calibrated | 0.60 measured P4 SITL start value |

The common MotorTilt output is deliberate. A real Transwing mechanism should
not receive independent left and right transition commands without an
additional synchronization and fault-detection layer.

ArduPilot treats MotorTilt as a normalized hover-to-forward command. The SDF
maps that normalized travel onto the asset's measured 2.05943 rad oblique wing
fold; therefore `Q_TILT_MAX` is a controller scheduling approximation, not a
claim that the P4 hinge angle equals a conventional tiltrotor nacelle angle.

The 2021 ArduPilot-hosted presentation *Transwing by Tim Whitehand* shows a
50% transition condition at 10–20 m/s with angle of attack below 20 degrees,
active control surfaces, weak propulsor pitch authority and substantial roll
damping. The midpoint `-1.029716127 rad`, 27 m/s wing-support floor and
conservative 16-degree SDF stall angle remain evidence-linked references.
`Q_TILT_MAX=20` is instead a simulation safety calibration: it holds a shallow
tilt while airspeed builds, then the TIMER phase continues to full fold. It
does **not** prove these values are the production X-P4 transition schedule.

The X-P4 UAFM separately publishes a 52-knot fixed-wing stall speed. Therefore
`AIRSPEED_MIN=27` is placed just above the corresponding 26.75 m/s stall
estimate. Flight log `00000034.BIN` then showed that removing VTOL lift at
that exact threshold made the estimated airframe descend before its V-tail
could recover. The initial baseline therefore used `Q_ASSIST_SPEED=32` as a
transition margin, while `AIRSPEED_CRUISE=34` is the fixed-wing target.
Logs `00000043.BIN` and `00000044.BIN` demonstrated that this is unsafe for
the current surrogate: `ARSPD_TYPE=100` provides a valid SITL pitot stream, so
the speed-based QAssist remains eligible even with `Q_OPTIONS=4096`. A brief
speed dip during FBWB/QRTL then re-entered transition and moved both wings.
The current identification baseline uses `Q_ASSIST_SPEED=-1` to disable that
automatic assist completely; deliberate `QRTL`/`QLOITER`/`QLAND` commands and
normal transitions remain available. `Q_OPTIONS=4096` is retained as a
second-layer guard for ArduPilot builds that classify the stream as synthetic.
The baseline also sets `Q_ASSIST_ANGLE=0`: log `00000038.BIN` showed angle
assist reversing MotorTilt from 90° to 20° after a transient roll excursion,
then repeatedly re-entering transition while still in FBWB.
After the guard change, `00000039.BIN` held MotorTilt at 90 degrees for 18.7 s
of fixed-wing flight with no repeated transition messages, roll within 0.16
degrees and pitch within 7.14 degrees.
`ARSPD_TYPE=100` enables the SITL pitot path; without it ArduPilot remains in
AIRSPEED_WAIT indefinitely. `SCALING_SPEED=34` makes the fixed-wing rate-loop
gain schedule agree with this envelope. The ArduPlane default of 15 m/s cut
pitch authority to roughly 35-54 percent in the first transition logs.
`00000044.BIN` also identified a lateral-mode limitation: QRTL commanded a
92-degree heading correction and immediately saturated the `-45` degree bank
limit. The surrogate rolled in the correct direction but overshot to `-59`
degrees, before growing into an oscillation. The calibrated return envelope is
therefore limited to 25 degrees bank, with a slower L1 period (35 s), higher
L1 damping (0.90), a 45 deg/s roll-rate limit and reduced roll FF/P/I gains
with a small D term. These values are a SITL stability envelope, not X-P4
flight-control data.
DataFlash `00000029.BIN` showed the stock fixed-wing pitch loop reaching only
about 0.8 percent P, 9.6 percent FF and 14.6 percent I while pitch remained
near -10 degrees. The surrogate uses `PTCH_RATE_P=0.08` and equal
`PTCH_RATE_I=PTCH_RATE_FF=0.50`; servo limits remain the hard authority bound.
The first longer FBWB checks (`00000036.BIN`) then exposed a phugoid in the
estimated airframe: throttle cycled from roughly 21 to 84 percent while TECS
traded too much height for a small speed correction. The SITL baseline uses
`TECS_SPDWEIGHT=0.50`, `TECS_TIME_CONST=8`, `TECS_THR_DAMP=0.80` and
`TECS_PITCH_MIN=-5` to prioritize height, damp throttle response and prevent
an excessive automatic pitch-down. These are not real-aircraft TECS values.
The compound fold has a mid-travel aerodynamic dead zone in this surrogate:
the wing is not yet useful in fixed-wing lift while rotor vertical thrust has
already fallen. `Q_TILT_RATE_DN=20` crosses that interval promptly after
airspeed is established; it is a SITL identification value, not a production
X-P4 schedule.
The complete dashboard/API path passed the bounded transition test in
DataFlash `00000030.BIN`: ArduPilot completed the transition, FBWB remained
inside the configured attitude and altitude-loss limits for 20 seconds, and
the recovery returned through QLOITER/QLAND. Broad automated experiment
campaigns remain locked because the coefficients are still surrogate values.

The disarmed integration smoke test loads the complete override file into
ArduPlane and verifies the Quad-X, tilt, aileron, V-tail, hover-thrust and
QLOITER limit values through the MAVProxy parameter snapshot.

The closed-loop ground-takeoff test reached 4.64 m for a 5 m request, then
held altitude at about 58-59% output. `Q_P_ANGLE_MAX=6`,
`Q_LOIT_ANG_MAX=6`, `Q_LOIT_ACC_MAX_M=1.0` and `Q_A_ANG_PIT_P=2.5`
make the folded-wing hover position loop deliberately gentle while the inertia
and centre of gravity remain estimated. These values prove baseline SITL
controllability only; they are not flight-hardware gains.

The folded-hover CG calibration moves the complete model CG from x=-0.4967 m
to x=-0.3707 m, beside the four-rotor centroid at x=-0.3720 m. It removes the
previous 250-300 PWM rear-motor bias. With that static bias gone,
`Q_A_RAT_PIT_I=0.12` replaces the 0.25 default to reduce pitch-rate integral
lag. A repeat 5 m test measured lift-off pitch -1.4 to +0.4 degrees, climb
-0.9 to +0.6 degrees, and hover -1.0 to +0.3 degrees with 0.02 m altitude
range over five seconds. These are deterministic SITL calibration results,
not gains or CG data for a real aircraft.

## Values still requiring identification

- complete mass distribution and inertia tensor;
- real center of gravity in both configurations;
- propeller thrust and reaction-torque curves;
- servo travel versus physical fold angle;
- aerodynamic coefficients in folded, transitional and cruise states;
- motor/servo latency;
- hover and fixed-wing attitude gains;
- transition airspeed and schedule.
- two-motor cruise selection (the X-P4 uses all four propulsors in VTOL but
  turns off one pair in fixed-wing cruise);
- simulated pitot/airspeed calibration and wind sensitivity for
  `ARSPD_TYPE=100`;

Do not copy this file to flight hardware. Validate actuator directions while
disarmed, then use a restrained simulation-only motor test before attempting
QSTABILIZE or QLOITER.
