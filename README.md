# Enginio2D

A four-stroke engine simulator: crank-angle thermodynamics, rigid-body crank
dynamics, and an exhaust note synthesised from the simulated port flows.
SFML 3.1.0 for graphics and audio.

## Build

CLion: just open the folder — `SFML_DIR` defaults to `C:/SFML-3.1.0/lib/cmake/SFML`.
The SFML DLLs are copied next to the executable after each build.

Otherwise run `build.bat`, which loads the MSVC environment and uses the CMake
and Ninja that ship with Visual Studio, then launches the app. `build.bat norun`
builds only.

By hand, from a Developer Command Prompt:

```
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo && cmake --build build
```

(Windows PowerShell 5.1 has no `&&` — separate the commands with `;` or run them
one at a time.)

The physics runs at the audio sample rate, so Debug builds compile with `/O2`
too — an unoptimised build will not keep up with the audio thread.

## Controls

| Key | Action |
| --- | --- |
| `W` / `Up` | throttle (ramps like a pedal, not a switch) |
| `S` | starter motor |
| `Down` / `B` | brake |
| `Q` / `E` | shift down / up |
| `0`-`6`, `N` | select gear directly (0 or N is neutral) |
| `I` | ignition on/off |
| `Esc` | quit |

Hold `S` for a moment, and it fires and settles to idle around 870 rpm. Select
first with `1`, then throttle: the clutch feeds itself in and the car pulls
away.

## How it works

### Thermodynamics — `src/sim/Engine.cpp`

Each cylinder is a single zone of a two-component gas (unburned charge and
combustion products), integrated in crank angle:

- **Volume** from the exact slider-crank relation; torque comes from
  `tau = p dV/dtheta`, so the gas loads fall out of the same geometry.
- **Gas properties** vary with temperature and composition. Specific heat is
  linear in T for each component, which inverts to a temperature in closed
  form; effective gamma therefore falls from ~1.40 cold to ~1.22 at flame
  temperature, as it does in a real cylinder.
- **Valve flow** through a compressible orifice, area = min(valve curtain,
  port throat) times a lift-dependent discharge coefficient shaped like
  flow-bench data, choked below the critical pressure ratio. Both directions
  are modelled, so reversion and overlap backflow happen on their own. Cam
  profiles are baked into a 0.1 degree lift table at construction.
- **Runners** are inertances, not restrictions: each intake and exhaust port
  has a gas column with momentum plus a small volume, so the charge rams and
  the exhaust extracts. This is what produces VE near 1.0 at the torque peak
  and the fall-off either side of it.
- **Combustion** as a Wiebe function (a = 5, m = 2) burning only the *fresh*
  part of the charge, with an ignition delay and a burn duration that
  lengthens with speed and with dilution. Residual gas therefore slows the
  flame, cuts efficiency, and past `misfireLimit` puts it out entirely.
- **Heat loss** through the Woschni correlation including its combustion
  velocity term, referenced to the state trapped at intake valve closing.
- **Intake plenum** behind the throttle plate, and an **exhaust collector**
  that every runner feeds and which vents to atmosphere, so back pressure and
  residual gas are simulated rather than assumed.
- **Control**: spark advance and AFR follow small maps against speed and load,
  and a PID idle-air valve with an anti-stall dashpot holds the idle, the way
  an ECU does. Without the derivative term the idle limit-cycles.

Crank dynamics are `I dw/dt = tau_gas + tau_recip - tau_friction - tau_load`.
Reciprocating piston inertia gives the crank its within-cycle speed ripple, and
friction uses a Chen-Flynn FMEP (rising with peak pressure and with the square
of piston speed) plus accessory drag.

### Sound — `src/audio/EngineSound.cpp`

The note is not sampled and not oscillator-based. The sim advances one step per
audio sample, and:

- **Every cylinder gets its own primary pipe**, driven by that cylinder's own
  simulated exhaust runner pressure, with slightly unequal lengths as on a real
  manifold. The pulses arrive separated in time and interfere in the collector
  the way a 4-into-1 does, instead of stacking into one tone.
- Each pipe is a **digital waveguide** (delay lines out and back) with a
  low-passed open-end reflection standing in for radiation impedance *and* a
  wall-loss filter on the return traverse. Those wall losses are most of the
  difference between something that sounds like a pipe and something that
  sounds like a comb filter.
- **Radiation is the derivative** of the outgoing wave rather than the wave
  itself, which is where the crack in the note comes from.
- A **mechanical layer** on top: valves seating (triggered when the simulated
  lift reaches zero, so the clatter follows the cam), the crack of each
  combustion event, and broadband chain whirr. Standing next to an idling
  engine, that is most of what you actually hear.
- Asymmetric saturation, because a real exhaust steepens the front of a pulse
  far more than its back, plus stereo spread between tailpipe and intake.

Only *fluctuation* radiates, so every source - each runner, and the intake - is
DC-blocked first. Feeding the manifold's standing 80 kPa of vacuum in as signal
made the engine get louder when you lifted off, which is exactly the sort of
thing that reads as synthetic.

Output level is set feed-forward from a slow estimate of how hard the pipes are
being driven, tapered by a quarter power. Pulses really are ~50x stronger at
full load than at idle: left alone that is unlistenable, and compensating by
driving a saturator harder turns it into a constant buzz. About 26 dB survives.

### Drivetrain - `src/sim/Drivetrain.cpp`

Six ratios plus neutral, a final drive, and a 1250 kg car with aerodynamic drag
and rolling resistance. The clutch is a **slipping friction coupling**, not a
rigid link: it saturates at its torque capacity, opens below a few hundred rpm
and during a shift, and comes home as engine or road speed rises. That is what
lets the engine idle in gear, pull away from rest, and be shifted without the
crank speed stepping. Engine braking on a closed throttle falls out for free.

Rendering reads a double-buffered snapshot, so the render thread never blocks
the audio thread.

## Tuning

Everything interesting is in `sim::EngineParams` (`src/sim/Engine.h`): bore,
stroke, rod length, compression ratio, cylinder count and phasing, valve timing
and lift, spark and AFR maps, burn duration, runner lengths and areas,
collector volume, inertia, friction, and the idle controller.

`phaseOffsets` sets the firing pattern — the default is an inline-four at
`{0, 540, 180, 360}`. Change it to `{0, 90, 450, 630, 270, 180, 540, 360}` for a
cross-plane V8, or to unevenly spaced values to hear a big twin.

Current defaults (2.0 L four) idle at ~870 rpm on 30 kPa of manifold pressure
with 21% residual gas, and make 197 N m at 3400 rpm (12.4 bar BMEP) and 95 kW at
6200 rpm, with 62–78 bar peak cylinder pressure and 1.4–2.0 bar FMEP. Volumetric
efficiency peaks near 0.97 and falls to 0.74 at 6800 rpm.

## Layout

```
src/sim/Engine.{h,cpp}          thermodynamics, gas exchange, crank dynamics
src/sim/Drivetrain.{h,cpp}      clutch, gearbox, vehicle
src/audio/EngineSound.{h,cpp}   pipe acoustics and mechanical noise; audio-rate driver
src/app/main.cpp                SFML window and all views
```

The views are: a section through cylinder 1 (crank, rod, piston, valves, charge
tinted by temperature), the **whole engine from above** - every cylinder in the
block with its valve heads opening, the curtain gap drawn to scale, and a halo
as each one fires, so the firing order is something you watch rather than read -
the tachometer, gear indicator and instruments, a **valve lift diagram** over
the 720 degree cycle with the overlap shaded and a cursor at the current angle,
the cylinder pressure trace, and the exhaust waveform.

## Known gaps

- Runners are lumped inertances, not a 1D wave solver, so they capture ram and
  extraction but not the higher harmonics of real pipe tuning.
- No knock model, no blowby, no crevice volumes; Woschni carries a calibration
  multiplier to stand in for what those would remove.
- No thermal model of the block: wall temperature is a constant, so there is no
  cold-start enrichment or warm-up behaviour.
- No tyre model: wheel torque becomes acceleration directly, so first gear has
  grip it would not have in reality.
- Shifts are instant selections with a fixed clutch-out time; there is no
  synchro or throttle blip modelling.
- Only cylinder 1 is drawn in section; the others appear in the top view.
