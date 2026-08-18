# Enginio2D

A four-stroke engine simulator: crank-angle thermodynamics, rigid-body crank
dynamics, and an exhaust note synthesised from the simulated port flows — with
a full editor, so the engine is something you specify rather than something
that ships fixed. SFML 3.1.0 for graphics and audio.

## Build

CLion: just open the folder — `SFML_DIR` defaults to `deps/SFML/lib/cmake/SFML`.
The SFML DLLs are copied next to the executable after each build.

Otherwise run `build.bat`, which loads the MSVC environment and uses the CMake
and Ninja that ship with Visual Studio, then launches the app. `build.bat norun`
builds only. (Its `VS` variable is a hard-coded install path; edit it if Visual
Studio lives somewhere else on your machine.)

By hand, with CMake and Ninja on `PATH`:

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
| `Tab` | open / close the engine editor |
| `W` / `Up` | throttle (ramps like a pedal, not a switch) |
| `S` | starter motor |
| `Down` / `B` | brake |
| `Q` / `E` | shift down / up |
| `0`-`8`, `N` | select gear directly (0 or N is neutral) |
| `I` | ignition on/off |
| `Esc` | quit |

Hold `S` for a moment, and it fires and settles to idle. Select first with `1`,
then throttle: the clutch feeds itself in and the car pulls away.

## The editor

`Tab` opens it over the running engine, which keeps idling behind the panel.
Drag a slider, or roll the wheel over any control; hold `Shift` for fine steps.
With **live apply** on, the engine is rebuilt about a fifth of a second after
you stop moving a control, so you hear what you changed. Turn it off and
changes wait for **Apply**.

Nine tabs, grouped the way an engine is actually specified:

- **Layout** — inline, V, flat/boxer or W; 1 to 16 cylinders; bank angle; and
  the crank: even-fire, crossplane, or odd-fire on shared pins. A chart shows
  when each cylinder fires over the 720 degree cycle, coloured by bank.
- **Bottom end** — bore, stroke, rod ratio, compression, reciprocating mass,
  flywheel inertia, with displacement, bore/stroke ratio and mean piston speed
  at the limiter reported back.
- **Head & valves** — OHV, SOHC, DOHC, desmodromic or pneumatic; one to three
  intake and exhaust valves; valve material; head diameters and lift ratio;
  cam duration and centreline for each; profile aggressiveness; port work; and
  a cam phaser. It reports LSA, overlap, IVO/IVC/EVO/EVC and the speed at which
  the valves stop following the cam.
- **Fuel & spark** — twelve fuels, spark maps, burn duration, ignition delay,
  lambda at light and full load, knock control, redline and idle speed.
- **Oil & friction** — grade, running and start temperature, friction scale,
  accessory drag.
- **Induction** — throttle bore, plenum volume, runner length and diameter,
  and a turbo, roots blower or centrifugal blower with boost, spool, lag and
  intercooler effectiveness. The ram-tuned speed of the runner is reported.
- **Exhaust** — header style, primary length and diameter, collector volume,
  and muffler. Both flow and sound follow.
- **Drivetrain** — up to eight ratios, final drive, and the car: mass, drag
  area, wheel radius, clutch capacity, brakes. Top speed per gear is listed.
- **Appearance** — theme, accent hue, block and cam-cover colour, and which
  views to show. These apply instantly and never rebuild the engine.

Twelve **presets** ship with it, from a stock 2.0 four to a crossplane V8, a
600cc superbike, a blown methanol drag motor, a turbo diesel, a 45-degree
V-twin and a hydrogen six. **Save** writes `designs/<name>.eng`, one field per
line, so a design can be read, diffed and hand-edited; **Open** loads any file
found there.

### The dynamometer

**Dyno pull** sweeps a private copy of the engine on a worker thread, holding
each speed with an ideal brake at wide-open throttle, waiting for the manifold,
the pipes and the turbo to settle, and averaging the torque the brake absorbs.
The curve draws itself as the sweep runs.

This is the honest answer to "how much power does it make": power is not a
number you set, it is what the geometry, the cam, the fuel and the boost add up
to. What the presets produce:

| Preset | Peak power | Peak torque |
| --- | --- | --- |
| Stock 2.0 inline-4 | 86 kW at 5600 | 192 N m at 3000 |
| Turbo 2.0 inline-4 | 185 kW at 5600 | 392 N m at 2400 |
| Crossplane 5.0 V8 | 222 kW at 5700 | 495 N m at 3300 |
| Flat-plane 4.5 V8 | 284 kW at 7300 | 448 N m at 3200 |
| Race 6.0 V12 | 391 kW at 7900 | 552 N m at 3500 |
| Blown methanol 7.0 V8 | 607 kW at 4800 | 1524 N m at 2100 |
| 3.0 inline-6 turbo diesel | 184 kW at 4600 | 471 N m at 1700 |
| 600cc inline-4 superbike | 70 kW at 14400 | 58 N m at 10100 |

The drivetrain tab also reports the **drag-limited top speed**, so the per-gear
speeds at the limiter can be read for what they are. A top gear taller than the
drag limit is an overdrive, not more speed.

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
  profiles are baked into a 0.1 degree lift table when the engine is built.
- **Runners** are inertances, not restrictions: each intake and exhaust port
  has a gas column with momentum plus a small volume, so the charge rams and
  the exhaust extracts. This is what produces VE near 1.0 at the torque peak
  and the fall-off either side of it.
- **Combustion** as a Wiebe function (a = 5, m = 2) burning only the *fresh*
  part of the charge, with an ignition delay and a burn duration that
  lengthens with speed, with dilution, and with distance from the mixture that
  burns fastest. Residual gas therefore slows the flame, cuts efficiency, and
  past `misfireLimit` puts it out entirely.
- **Heat loss** through the Woschni correlation including its combustion
  velocity term, referenced to the state trapped at intake valve closing.
- **Intake plenum** behind the throttle plate, and an **exhaust collector**
  that every runner feeds and which vents to atmosphere, so back pressure and
  residual gas are simulated rather than assumed.
- **Control**: spark advance and lambda follow small maps against speed and
  load, retarded by boost and by the knock sensor. Idle is held by two loops the
  way a real ECU holds it: the air valve carries the steady state, and a spark
  trim carries the fast correction, because torque follows timing within one
  firing where it takes several cycles to follow the manifold. Running both
  hard makes them fight and the engine hunts; the trim also stands down below
  half idle speed, since advancing the spark at cranking speed makes an engine
  fight itself and never catch.
- A long-overlap cam **cannot idle slowly** - so much of what it draws in goes
  straight out of the exhaust that what is left will barely burn - so the
  minimum idle speed a design will accept moves with the overlap its cam
  actually has. A 280 degree race cam idles near 1900 rpm and lopes even there.

Crank dynamics are `I dw/dt = tau_gas + tau_recip - tau_friction - tau_load`.
Reciprocating piston inertia gives the crank its within-cycle speed ripple, and
friction uses a Chen-Flynn FMEP (rising with peak pressure and with the square
of piston speed) plus accessory drag and any supercharger.

### Fuel

A fuel is five numbers that matter: energy per kilogram, the air a kilogram
needs, the latent heat it takes out of the charge as it evaporates, how fast
its flame moves, and how hard it is to detonate. Those drive everything:

- **Charge cooling** is applied in the runner, where a port injector actually
  evaporates. On petrol it is worth a few degrees. On methanol it is worth over
  a hundred, and that density gain is most of why alcohol makes power.
- **Heat release is limited by oxygen, not by fuel.** Below stoichiometric
  there is not enough air to burn everything injected, and the excess leaves as
  carbon monoxide. Releasing it anyway is a classic way to make a rich full-load
  mixture produce more heat than the air could ever supply - too much peak
  pressure, knock everywhere, and exhaust temperatures hundreds of degrees above
  anything real.
- **Knock** uses the Douaud-Eyzat induction time integrated Livengood-Wu style
  over the unburned gas, whose temperature is tracked by isentropic compression
  from the state at intake valve closing. The integral stops once the flame has
  crossed most of the chamber, because past that there is no end gas left to
  detonate. When it reaches one first, the end gas lights on its own: the
  remaining charge goes off in one step, the pressure spikes, heat loss to the
  walls jumps, and you hear it. The calibration is set so a stock 10.5:1 engine
  on 95 RON is knock-limited around 30 degrees of advance, which is where a real
  one is - so knock is a consequence of what you build, not a constant. Knock resistance is a bowl in lambda, worst just lean of
  stoichiometric — enrichment and genuine lean-burn both move away from it.
  With knock control on, the ECU pulls timing fast and gives it back slowly.
- **Compression ignition** is a different engine, not a switch: no throttle
  plate, no spark, the pedal meters fuel against a smoke limit, ignition delay
  falls as the charge gets hotter (which is why a cold one rattles), and the
  diffusion burn is longer than a premixed flame.

### Exhaust temperature

The pipes are radiators, and each has a wall temperature of its own: heated by
the gas going past, cooled to the air by convection and - once it glows - mostly
by radiation, which is the term that pins a manifold near 800 C at full load
however hard you drive it. Steel this thin carries about half a minute of
thermal lag, so a cold pipe at light load reads properly cool and comes up as
you use it. Without the wall the gas arrives at the collector still at port
temperature, which is hundreds of degrees above anything a real probe sees.

### Forced induction

Everything upstream of the throttle plate: ambient on a naturally aspirated
engine, compressor discharge on a blown one. Isentropic compression plus
compressor inefficiency sets the charge temperature, and the intercooler gives
back what it can. A **turbo** follows the mass flow the engine is already
moving through a first-order lag, so it cannot make boost before it is making
power, and it pays in exhaust backpressure. A **roots blower** makes boost off
idle and takes its drive power off the crank the whole time; a **centrifugal**
one rises with the square of impeller speed.

### Valvetrain

Above its float speed the spring can no longer hold the follower on the cam and
lift collapses, which is the wall a pushrod engine hits and a pneumatic-valve
one does not. Float speed tracks mean piston speed — which is why a
short-stroke 600 four revs to 15000 and a long-stroke pushrod twin does not get
near it — scaled by valvetrain type, valve material and cam grind. The
valvetrain also caps how far the cam profile can be pushed: a flat tappet
cannot follow what a roller or a desmodromic mechanism can.

### Oil

Viscosity is interpolated between the grade's 40 °C and 100 °C points and
extrapolated past them, against an oil temperature that warms towards a
load-dependent target. Cold oil costs power in the hydrodynamic friction term
and pegs the pump on its relief valve; hot thin oil gives the power back and
runs the pressure down. Start an engine cold and watch the friction fall.

### Sound — `src/audio/EngineSound.cpp`

The note is not sampled and not oscillator-based. The sim advances one step per
audio sample, and:

- **Every cylinder gets its own primary pipe**, driven by that cylinder's own
  simulated exhaust runner pressure. Pipe length comes from the header you
  specified, and the spread between cylinders comes from the header style: a
  log manifold smears the pulses where equal-length tubes stack them.
- **The pipes are summed per bank** before their collectors. This is what makes
  layout audible: a crossplane V8 fires evenly overall but 90-180-270-180
  within each bank, and it is the bank that shares a collector.
- Each pipe is a **digital waveguide** (delay lines out and back) with a
  low-passed open-end reflection standing in for radiation impedance *and* a
  wall-loss filter on the return traverse. Those wall losses are most of the
  difference between something that sounds like a pipe and something that
  sounds like a comb filter.
- **Radiation is the derivative** of the outgoing wave rather than the wave
  itself, which is where the crack in the note comes from.
- The **muffler** you chose sets both the damping and the loudness.
- A **mechanical layer** on top: valves seating (triggered when the simulated
  lift reaches zero, so the clatter follows the cam), the crack of each
  combustion event pitched off the bore, detonation as a much brighter ring,
  and broadband chain whirr.
- Asymmetric saturation, because a real exhaust steepens the front of a pulse
  far more than its back, plus stereo spread between tailpipe and intake.

Only *fluctuation* radiates, so every source — each runner, and the intake — is
DC-blocked first. Feeding the manifold's standing 80 kPa of vacuum in as signal
made the engine get louder when you lifted off, which is exactly the sort of
thing that reads as synthetic.

Output level is set feed-forward from a slow estimate of how hard the pipes are
being driven, tapered by a quarter power. Pulses really are ~50x stronger at
full load than at idle: left alone that is unlistenable, and compensating by
driving a saturator harder turns it into a constant buzz. About 26 dB survives.

### Drivetrain — `src/sim/Drivetrain.cpp`

Up to eight ratios plus neutral, a final drive, and a car with aerodynamic drag
and rolling resistance. The clutch is a **slipping friction coupling**, not a
rigid link: it saturates at its torque capacity, opens below a few hundred rpm
and during a shift, and comes home as engine or road speed rises. That is what
lets the engine idle in gear, pull away from rest, and be shifted without the
crank speed stepping. Engine braking on a closed throttle falls out for free.

### Threading

The physics runs on the audio thread, which owns the engine. The editor never
touches it: it hands over a finished `EngineParams` through a double-buffered
slot with a generation counter and an acknowledgement, and the audio thread
rebuilds the engine at a chunk boundary. Crank speed, oil temperature and the
driver's inputs survive the rebuild, so an edit does not feel like a restart.
Rendering reads a double-buffered snapshot, so the render thread never blocks
the audio thread. The dyno runs on a third thread against its own copy.

Sub-stepping is bounded by two constraints, the looser winning: about 8 µs of
wall time for the explicit gas-exchange integration, and 0.25 degrees of crank
angle to resolve a valve event. Insisting on the angle at every speed made a
V12 at 10000 rpm cost three times what it needed to.

## Layout

```
src/sim/Engine.{h,cpp}          thermodynamics, gas exchange, knock, boost, crank dynamics
src/sim/EngineDesign.{h,cpp}    the editable specification, and what it derives
src/sim/Dyno.{h,cpp}            steady-state power sweep on a worker thread
src/sim/Drivetrain.{h,cpp}      clutch, gearbox, vehicle
src/audio/EngineSound.{h,cpp}   pipe acoustics and mechanical noise; audio-rate driver
src/ui/Widgets.{h,cpp}          immediate-mode controls, palette, clipping and scrolling
src/ui/Editor.{h,cpp}           the editor panel, firing chart and dyno chart
src/app/main.cpp                SFML window and all views
```

`EngineDesign` is what a person edits — millimetres, litres, bar, degrees
Celsius — and `paramsFromDesign()` turns it into the SI quantities the solver
wants, filling in the consequences: firing order from the layout, valve size
from the bore, float speed from the stroke, idle bypass area from the
displacement. `clampDesign()` runs after every edit, so a design can never
describe something that cannot exist.

The views are: a section through cylinder 1 (crank, rod, piston, valves, charge
tinted by temperature, flaring white when it detonates), the **whole engine
from above** — one column per bank, every cylinder with its valve heads opening,
the curtain gap drawn to scale, and a halo as each one fires, so the firing
order is something you watch rather than read — the tachometer, gear indicator,
twenty-two instruments and a row of warning lamps, a **valve lift diagram**
over the 720 degree cycle with the overlap shaded, the phaser position applied
and a cursor at the current angle, the cylinder pressure trace, and the exhaust
waveform.

## Known gaps

- Runners are lumped inertances, not a 1D wave solver, so they capture ram and
  extraction but not the higher harmonics of real pipe tuning.
- The gas dynamics run one exhaust collector regardless of layout; the split
  into per-bank collectors exists only in the sound synthesis.
- No blowby and no crevice volumes; Woschni carries a calibration multiplier to
  stand in for what those would remove, as does the knock induction time.
- No dissociation at peak temperature, so the model slightly over-values a
  stoichiometric mixture against a rich one.
- No thermal model of the block: wall temperature is a constant, so the oil
  warms up but there is no cold-start enrichment.
- No tyre model: wheel torque becomes acceleration directly, so first gear has
  grip it would not have in reality.
- Shifts are instant selections with a fixed clutch-out time; there is no
  synchro or throttle blip modelling.
- Only cylinder 1 is drawn in section; the others appear in the top view.
- Applying an edit re-seeds the gas states at ambient, so the engine stumbles
  for a cycle or two before it settles.
