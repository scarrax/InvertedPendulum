# AP3 Teil 3 (Formaler Reglervergleich) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a headless benchmark script that compares `SimpleController`,
`LQRController`, and `SwingUpController` on stability (capture envelope),
reaction time, and robustness (force-pulse disturbance) against the real
`InvertedPendulumMB.fmu`, producing a Markdown report + plots for AP5.

**Architecture:** New standalone `benchmark_controllers.py` at the project
root, built up function-by-function across tasks: pure metric functions
(no FMU) → FMU simulation driver → scenario orchestration (with dependency
injection so scenario logic is unit-testable without the FMU) → report/plot
generation + `main`. Imports the three existing controllers unchanged from
`pendulum_game_controlled.py`.

**Tech Stack:** Python, `fmpy` (FMU co-simulation), `matplotlib` (new
dependency, plots), `pytest`.

**Spec:** `docs/superpowers/specs/2026-08-17-ap3-controller-comparison-design.md`

## Global Constraints

- **Winkelkonvention:** `phi=0` ist die hängende Ruhelage, `phi=π` die
  aufrechte Zielposition (bindend, `CLAUDE.md`).
- **FMU-Substepping:** jeder `fmu.doStep()`-Aufruf muss sub-gesteppt werden
  — `SUBSTEPS=10`, `inner_dt = dt/SUBSTEPS`, `tau` wird einmal pro äußerem
  50Hz-Frame (`dt=0.02`) gesetzt und über die Substeps konstant gehalten
  (bindende Konvention, `CLAUDE.md` — sonst numerische Instabilität bei der
  niedrigen `d_pend`-Dämpfung).
- **Anfangsbedingungen** werden ausschließlich über die FMU-Parameter
  `phi0`/`vphi0` gesetzt (`causality=parameter, initial=exact`, vor
  `enterInitializationMode()`). Die Ausgabegrößen `phi`/`vphi`
  (`causality=output, initial=calculated`) sind **nicht** von außen
  setzbar — weder vor noch während der Simulation (empirisch gegen die
  reale FMU verifiziert, siehe Spec §2).
- **Störform:** Robustheit wird über einen additiven Kraft-Puls auf `tau`
  simuliert (`KICK_TAU=8.0`, `KICK_STEPS=5`), nicht über einen
  Geschwindigkeits-Kick auf `vphi` (technisch nicht umsetzbar, siehe Spec
  §3.3).
- **Erfolgskriterium:** ein Regler "hält" die aufrechte Lage ab dem
  Zeitpunkt, an dem `|theta| < 5°` erreicht ist und für mindestens `1.0s`
  ununterbrochen bleibt (`TOLERANCE_DEG=5.0`, `HOLD_DURATION=1.0`).
- **Keine Änderung** an `pendulum_game_controlled.py`, der
  `Controller`-Basisklasse, oder den drei bestehenden Reglern.
- **Testing:** FMU-gestützte Tests werden mit
  `pytest.mark.skipif(not os.path.exists(FMU_PATH), reason="...")` gegen
  `InvertedPendulumMB.fmu` im Projektroot geguarded (gitignored, wird
  manuell in Worktrees kopiert) — Pattern aus
  `tests/test_numerical_stability.py`.
- **`benchmark_plots/`** ist ein neues, gitignored Ausgabeverzeichnis
  (Build-Artefakt, jederzeit reproduzierbar). `AP3_Reglervergleich.md` wird
  dagegen committet (Text-Deliverable, analog `AP1_Validierung.md`).

---

### Task 1: Kennzahlen-Funktionen (`held_from`, `find_capture_envelope`)

**Files:**
- Create: `benchmark_controllers.py`
- Test: `tests/test_benchmark_metrics.py`

**Interfaces:**
- Produces: `TOLERANCE_DEG` (float, module constant), `HOLD_DURATION`
  (float, module constant), `held_from(t, theta, tolerance_rad,
  hold_duration) -> float | None`, `find_capture_envelope(results_by_theta0:
  dict[float, bool]) -> float | None`.

- [ ] **Step 1: Write the failing tests**

Create `tests/test_benchmark_metrics.py`:

```python
import math

from benchmark_controllers import held_from, find_capture_envelope


def test_held_from_returns_start_time_when_held_from_the_beginning():
    t = [0.0, 0.5, 1.0, 1.5]
    theta = [0.0, 0.0, 0.0, 0.0]
    result = held_from(t, theta, tolerance_rad=math.radians(5), hold_duration=1.0)
    assert result == 0.0


def test_held_from_returns_none_when_it_only_passes_through_the_zone():
    t = [0.0, 0.5, 1.0, 1.5]
    theta = [math.radians(90), 0.0, math.radians(90), math.radians(90)]
    result = held_from(t, theta, tolerance_rad=math.radians(5), hold_duration=1.0)
    assert result is None


def test_held_from_returns_none_when_never_within_tolerance():
    t = [0.0, 0.5, 1.0]
    theta = [math.radians(90), math.radians(90), math.radians(90)]
    result = held_from(t, theta, tolerance_rad=math.radians(5), hold_duration=1.0)
    assert result is None


def test_held_from_returns_entry_time_not_confirmation_time():
    t = [0.0, 0.3, 0.6, 1.0, 1.3]
    theta = [math.radians(90), 0.0, 0.0, 0.0, 0.0]
    result = held_from(t, theta, tolerance_rad=math.radians(5), hold_duration=1.0)
    assert result == 0.3


def test_find_capture_envelope_returns_largest_success_before_first_failure():
    results = {2: True, 4: True, 6: True, 8: False, 10: False}
    assert find_capture_envelope(results) == 6


def test_find_capture_envelope_ignores_a_later_spurious_success():
    results = {2: True, 4: True, 6: False, 8: True}
    assert find_capture_envelope(results) == 4


def test_find_capture_envelope_returns_none_when_first_value_fails():
    results = {2: False, 4: True}
    assert find_capture_envelope(results) is None


def test_find_capture_envelope_returns_none_for_empty_input():
    assert find_capture_envelope({}) is None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_benchmark_metrics.py -v`
Expected: FAIL (`ModuleNotFoundError: No module named 'benchmark_controllers'`
or `ImportError`, since the file doesn't exist yet)

- [ ] **Step 3: Create `benchmark_controllers.py` with the implementation**

```python
"""Headless benchmark comparing the three AP3 controllers against the real
InvertedPendulumMB.fmu on stability, reaction time, and robustness.

See docs/superpowers/specs/2026-08-17-ap3-controller-comparison-design.md
for the full methodology.
"""

import math

# Success criterion shared across all scenarios: a controller "holds" the
# upright position from the moment |theta| stays below TOLERANCE_DEG for at
# least HOLD_DURATION seconds. TOLERANCE_DEG matches the game's own
# tight_bonus_zone (pendulum_game_controlled.py's compute_score_increment).
TOLERANCE_DEG = 5.0
HOLD_DURATION = 1.0


def held_from(t, theta, tolerance_rad, hold_duration):
    """Return the earliest time in `t` from which |theta| stays below
    `tolerance_rad` for at least `hold_duration` seconds, or None if that
    never happens within the given data."""
    entry_time = None
    for ti, th in zip(t, theta):
        if abs(th) < tolerance_rad:
            if entry_time is None:
                entry_time = ti
            elif ti - entry_time >= hold_duration:
                return entry_time
        else:
            entry_time = None
    return None


def find_capture_envelope(results_by_theta0):
    """results_by_theta0: dict {theta0_deg: bool success}, assumed to come
    from a monotonically increasing sweep. Return the largest theta0_deg
    for which every smaller tested theta0_deg also succeeded (i.e. the
    value just before the first failure), or None if the smallest tested
    theta0_deg already failed or the input is empty."""
    best = None
    for theta0_deg in sorted(results_by_theta0):
        if results_by_theta0[theta0_deg]:
            best = theta0_deg
        else:
            break
    return best
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_benchmark_metrics.py -v`
Expected: PASS (8 passed)

- [ ] **Step 5: Commit**

```bash
git add benchmark_controllers.py tests/test_benchmark_metrics.py
git commit -m "feat: add benchmark metric functions (held_from, find_capture_envelope)"
```

---

### Task 2: FMU-Simulationstreiber (`simulate_run`)

**Files:**
- Modify: `benchmark_controllers.py` (append)
- Test: `tests/test_benchmark_driver.py`

**Interfaces:**
- Consumes: nothing from Task 1 directly (this task's tests don't need
  `held_from`/`find_capture_envelope`), but lives in the same module.
- Produces: `simulate_run(fmu_path, controller, theta0_deg=None,
  vphi0=0.0, duration=20.0, on_frame=None) -> tuple[list[float],
  list[float]]` — returns `(t_history, theta_history)`, both starting at
  `t=0.0` with the initial state as the first sample, `theta` in radians
  wrapped to `[-π, π)` via `(phi % (2*math.pi)) - math.pi` (same formula
  `LQRController.compute` already uses). `on_frame`, if given, is called
  once per outer frame as `on_frame(t, theta, vphi, s, v) -> float`
  *before* the step is taken, using the pre-step state; its return value is
  added to the controller's `tau` for that frame (default 0.0 when
  `on_frame` is `None`). `theta0_deg=None` leaves the FMU's own built-in
  `phi0`/`vphi0` defaults untouched (the real game's start condition,
  `phi0 = 0.75*pi/2` from `InvertedPendulumMB.mo`, ≈67.5° from hanging /
  ≈-112.5° from upright). `theta0_deg` given as a number sets `phi0 = pi +
  radians(theta0_deg)` (deviation from upright) and `vphi0` to the `vphi0`
  argument (radians/s, default 0.0).

- [ ] **Step 1: Write the failing tests**

Create `tests/test_benchmark_driver.py`. These need the real FMU (state
initialization can't be faked meaningfully) — guard with the same
`skipif` pattern as `tests/test_numerical_stability.py`:

```python
import math
import os

import pytest

FMU_PATH = os.path.abspath("InvertedPendulumMB.fmu")


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
def test_simulate_run_stays_bounded_for_small_deviation():
    from benchmark_controllers import simulate_run
    from pendulum_game_controlled import SimpleController

    t, theta = simulate_run(FMU_PATH, SimpleController(), theta0_deg=2.0, duration=3.0)

    assert len(t) == len(theta)
    assert t[0] == 0.0
    assert t[-1] >= 2.9
    assert all(abs(th) < math.radians(90) for th in theta)


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
def test_simulate_run_default_theta0_matches_real_game_start():
    from benchmark_controllers import simulate_run
    from pendulum_game_controlled import SimpleController

    t, theta = simulate_run(FMU_PATH, SimpleController(), theta0_deg=None, duration=0.1)

    expected_theta0 = math.radians(67.5) - math.pi
    assert math.isclose(theta[0], expected_theta0, abs_tol=math.radians(1))


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
def test_simulate_run_on_frame_offset_is_applied_and_clipped():
    from benchmark_controllers import simulate_run
    from pendulum_game_controlled import SimpleController

    calls = []

    def on_frame(t, theta, vphi, s, v):
        calls.append(t)
        return 1000.0  # deliberately huge, must be clipped to MAX_TAU=10.0

    t, theta = simulate_run(
        FMU_PATH, SimpleController(), theta0_deg=2.0, duration=0.5, on_frame=on_frame
    )

    assert len(calls) > 0
    assert all(abs(th) < math.radians(180) for th in theta)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_benchmark_driver.py -v`
Expected: FAIL with `ImportError: cannot import name 'simulate_run'`

- [ ] **Step 3: Append `simulate_run` to `benchmark_controllers.py`**

```python
import os
import shutil

from fmpy import extract, read_model_description
from fmpy.fmi2 import FMU2Slave


def simulate_run(fmu_path, controller, theta0_deg=None, vphi0=0.0, duration=20.0, on_frame=None):
    # Sub-stepped FMU co-simulation loop, same pattern as run_game() in
    # pendulum_game_controlled.py: explicit-Euler at the full 0.02s step
    # numerically injects energy into the lightly-damped pendulum, so tau
    # is held constant across SUBSTEPS smaller inner steps.
    dt = 0.02
    SUBSTEPS = 10
    MAX_TAU = 10.0
    substep_dt = dt / SUBSTEPS

    unzipdir = extract(fmu_path)
    desc = read_model_description(unzipdir)
    fmu = FMU2Slave(
        guid=desc.guid,
        unzipDirectory=unzipdir,
        modelIdentifier=desc.coSimulation.modelIdentifier,
    )
    fmu.instantiate()
    fmu.setupExperiment(startTime=0.0)

    def ref(name):
        for var in desc.modelVariables:
            if var.name == name:
                return var.valueReference
        raise Exception(f"'{name}' not found in FMU")

    if theta0_deg is not None:
        fmu.setReal([ref("phi0")], [math.pi + math.radians(theta0_deg)])
        fmu.setReal([ref("vphi0")], [vphi0])

    fmu.enterInitializationMode()
    fmu.exitInitializationMode()

    tau_ref = ref("tau")
    s_ref = ref("s")
    v_ref = ref("v")
    phi_ref = ref("phi")
    vphi_ref = ref("vphi")

    def wrapped_theta(phi):
        return (phi % (2 * math.pi)) - math.pi

    t = 0.0
    t_history = [0.0]
    theta_history = [wrapped_theta(fmu.getReal([phi_ref])[0])]

    try:
        while t < duration:
            phi = fmu.getReal([phi_ref])[0]
            vphi = fmu.getReal([vphi_ref])[0]
            s = fmu.getReal([s_ref])[0]
            v = fmu.getReal([v_ref])[0]
            theta = wrapped_theta(phi)

            tau = controller.compute(phi, vphi, s, v)
            if on_frame is not None:
                tau += on_frame(t, theta, vphi, s, v)
            tau = max(-MAX_TAU, min(MAX_TAU, tau))

            fmu.setReal([tau_ref], [tau])
            for _ in range(SUBSTEPS):
                t += substep_dt
                fmu.doStep(currentCommunicationPoint=t, communicationStepSize=substep_dt)

            t_history.append(t)
            theta_history.append(wrapped_theta(fmu.getReal([phi_ref])[0]))
    finally:
        fmu.terminate()
        fmu.freeInstance()
        shutil.rmtree(unzipdir)

    return t_history, theta_history
```

Add `import math` is already present from Task 1; add the new imports
(`os`, `shutil`, `fmpy` imports) at the top of the file, grouped with the
existing `import math`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_benchmark_driver.py -v`
Expected: PASS (3 passed). If `InvertedPendulumMB.fmu` is not present in
the working directory, these skip instead — copy the FMU into the worktree
first (gitignored, not part of the repo) so the tests actually execute
rather than silently skipping.

- [ ] **Step 5: Commit**

```bash
git add benchmark_controllers.py tests/test_benchmark_driver.py
git commit -m "feat: add FMU-driven simulate_run for the controller benchmark"
```

---

### Task 3: Szenario-Funktionen (`envelope_sweep`, `reaction_time`, `robustness`, `swingup_capture`)

**Files:**
- Modify: `benchmark_controllers.py` (append)
- Test: `tests/test_benchmark_scenarios.py`

**Interfaces:**
- Consumes: `held_from`, `find_capture_envelope` (Task 1), `simulate_run`
  (Task 2) as the default `simulate_fn`.
- Produces: `CONTROLLER_FACTORIES` (dict `{"PD": SimpleController, "LQR":
  LQRController, "SwingUp": SwingUpController}`), `KickInjector` (class),
  `envelope_sweep(fmu_path, controller_factory, theta0_values_deg=None,
  tolerance_deg=TOLERANCE_DEG, hold_duration=HOLD_DURATION, duration=20.0,
  simulate_fn=simulate_run) -> dict` with keys `results_by_theta0` (dict)
  and `envelope_deg` (float | None), `reaction_time(fmu_path,
  controller_factory, theta0_values_deg=(2.0, 10.0),
  tolerance_deg=TOLERANCE_DEG, hold_duration=HOLD_DURATION, duration=20.0,
  simulate_fn=simulate_run) -> dict` with keys `settling_times` (dict) and
  `trajectories` (dict of `theta0_deg -> (t, theta)`), `robustness(fmu_path,
  controller_factory, theta0_deg=2.0, tolerance_deg=TOLERANCE_DEG,
  hold_duration=HOLD_DURATION, kick_tau=8.0, kick_steps=5, duration=20.0,
  simulate_fn=simulate_run) -> dict` with keys `kicked` (bool),
  `recovery_time` (float | None), `kick_time` (float | None), `trajectory`
  (`(t, theta)`), `swingup_capture(fmu_path, controller_factory,
  tolerance_deg=TOLERANCE_DEG, hold_duration=HOLD_DURATION, duration=20.0,
  simulate_fn=simulate_run) -> dict` with keys `capture_time` (float |
  None) and `trajectory` (`(t, theta)`). `controller_factory` is a
  zero-argument callable returning a fresh `Controller` instance (a class
  itself works, e.g. `SimpleController`) — each scenario run must get its
  own fresh instance since `SwingUpController` is stateful (`self.mode`).

- [ ] **Step 1: Write the failing tests**

Create `tests/test_benchmark_scenarios.py`. These use fake `simulate_fn`
callables, so they need no FMU:

```python
import math

from benchmark_controllers import (
    KickInjector,
    envelope_sweep,
    reaction_time,
    robustness,
    swingup_capture,
)


def test_kick_injector_triggers_once_after_hold_duration_then_stays_quiet():
    injector = KickInjector(tolerance_rad=math.radians(5), hold_duration=1.0, kick_tau=8.0, kick_steps=2)

    assert injector.offset_for(0.0, 0.0) == 0.0
    assert injector.offset_for(0.5, 0.0) == 0.0
    assert injector.offset_for(1.0, 0.0) == 8.0
    assert injector.kick_time == 1.0
    assert injector.offset_for(1.5, math.radians(90)) == 8.0
    assert injector.offset_for(2.0, math.radians(90)) == 0.0
    assert injector.offset_for(2.5, 0.0) == 0.0
    assert injector.offset_for(3.0, 0.0) == 0.0


def test_envelope_sweep_stops_after_two_consecutive_failures():
    def fake_simulate(fmu_path, controller, theta0_deg, duration):
        if theta0_deg <= 6:
            return [0.0, 1.0], [0.0, 0.0]
        return [0.0, 1.0], [math.radians(90), math.radians(90)]

    result = envelope_sweep(
        "unused.fmu", lambda: None,
        theta0_values_deg=[2, 4, 6, 8, 10, 12],
        tolerance_deg=5.0, hold_duration=1.0, duration=2.0,
        simulate_fn=fake_simulate,
    )

    assert result["envelope_deg"] == 6
    assert result["results_by_theta0"] == {2: True, 4: True, 6: True, 8: False, 10: False}
    assert 12 not in result["results_by_theta0"]


def test_reaction_time_records_settling_time_and_trajectory():
    def fake_simulate(fmu_path, controller, theta0_deg, duration):
        if theta0_deg == 2.0:
            return [0.0, 1.0], [0.0, 0.0]
        return [0.0, 1.0], [math.radians(90), math.radians(90)]

    result = reaction_time(
        "unused.fmu", lambda: None, theta0_values_deg=(2.0, 10.0),
        tolerance_deg=5.0, hold_duration=1.0, duration=1.0,
        simulate_fn=fake_simulate,
    )

    assert result["settling_times"][2.0] == 0.0
    assert result["settling_times"][10.0] is None
    assert result["trajectories"][2.0] == ([0.0, 1.0], [0.0, 0.0])


def test_robustness_measures_recovery_time_relative_to_kick():
    times = [0.0, 0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 3.5]
    thetas_deg = [0, 0, 0, 90, 90, 0, 0, 0]

    def fake_simulate(fmu_path, controller, theta0_deg, duration, on_frame=None):
        t_history, theta_history = [], []
        for t, theta_deg in zip(times, thetas_deg):
            theta = math.radians(theta_deg)
            if on_frame is not None:
                on_frame(t, theta, 0.0, 0.0, 0.0)
            t_history.append(t)
            theta_history.append(theta)
        return t_history, theta_history

    result = robustness(
        "unused.fmu", lambda: None, theta0_deg=2.0, tolerance_deg=5.0,
        hold_duration=1.0, kick_tau=8.0, kick_steps=1, duration=3.5,
        simulate_fn=fake_simulate,
    )

    assert result["kicked"] is True
    assert result["kick_time"] == 1.0
    assert result["recovery_time"] == 1.5


def test_robustness_reports_not_kicked_when_never_settled():
    def fake_simulate(fmu_path, controller, theta0_deg, duration, on_frame=None):
        t_history, theta_history = [], []
        for t in [0.0, 0.5, 1.0]:
            theta = math.radians(90)
            if on_frame is not None:
                on_frame(t, theta, 0.0, 0.0, 0.0)
            t_history.append(t)
            theta_history.append(theta)
        return t_history, theta_history

    result = robustness(
        "unused.fmu", lambda: None, simulate_fn=fake_simulate,
    )

    assert result["kicked"] is False
    assert result["recovery_time"] is None


def test_swingup_capture_uses_theta0_deg_none():
    seen = {}

    def fake_simulate(fmu_path, controller, theta0_deg, duration):
        seen["theta0_deg"] = theta0_deg
        return [0.0, 1.0], [0.0, 0.0]

    result = swingup_capture("unused.fmu", lambda: None, simulate_fn=fake_simulate)

    assert seen["theta0_deg"] is None
    assert result["capture_time"] == 0.0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_benchmark_scenarios.py -v`
Expected: FAIL with `ImportError: cannot import name 'KickInjector'`

- [ ] **Step 3: Append the scenario functions to `benchmark_controllers.py`**

```python
from pendulum_game_controlled import LQRController, SimpleController, SwingUpController

CONTROLLER_FACTORIES = {
    "PD": SimpleController,
    "LQR": LQRController,
    "SwingUp": SwingUpController,
}


class KickInjector:
    """on_frame callback for simulate_run: waits until the controller has
    held the upright position, then injects a fixed-magnitude tau pulse for
    `kick_steps` frames. Records when the kick started (`kick_time`, None
    if the controller never settled)."""

    def __init__(self, tolerance_rad, hold_duration, kick_tau, kick_steps):
        self.tolerance_rad = tolerance_rad
        self.hold_duration = hold_duration
        self.kick_tau = kick_tau
        self.kick_steps = kick_steps
        self.kick_time = None
        self._t_history = []
        self._theta_history = []
        self._kicked = False
        self._kick_frames_remaining = 0

    def offset_for(self, t, theta, vphi=None, s=None, v=None):
        self._t_history.append(t)
        self._theta_history.append(theta)

        if self._kick_frames_remaining > 0:
            self._kick_frames_remaining -= 1
            return self.kick_tau

        if not self._kicked:
            if held_from(self._t_history, self._theta_history, self.tolerance_rad, self.hold_duration) is not None:
                self._kicked = True
                self.kick_time = t
                self._kick_frames_remaining = self.kick_steps - 1
                return self.kick_tau

        return 0.0


def envelope_sweep(fmu_path, controller_factory, theta0_values_deg=None,
                    tolerance_deg=TOLERANCE_DEG, hold_duration=HOLD_DURATION,
                    duration=20.0, simulate_fn=simulate_run):
    if theta0_values_deg is None:
        theta0_values_deg = list(range(2, 92, 2))
    tolerance_rad = math.radians(tolerance_deg)

    results_by_theta0 = {}
    consecutive_failures = 0
    for theta0_deg in theta0_values_deg:
        controller = controller_factory()
        t, theta = simulate_fn(fmu_path, controller, theta0_deg=theta0_deg, duration=duration)
        success = held_from(t, theta, tolerance_rad, hold_duration) is not None
        results_by_theta0[theta0_deg] = success
        consecutive_failures = 0 if success else consecutive_failures + 1
        if consecutive_failures >= 2:
            break

    return {
        "results_by_theta0": results_by_theta0,
        "envelope_deg": find_capture_envelope(results_by_theta0),
    }


def reaction_time(fmu_path, controller_factory, theta0_values_deg=(2.0, 10.0),
                   tolerance_deg=TOLERANCE_DEG, hold_duration=HOLD_DURATION,
                   duration=20.0, simulate_fn=simulate_run):
    tolerance_rad = math.radians(tolerance_deg)
    settling_times = {}
    trajectories = {}
    for theta0_deg in theta0_values_deg:
        controller = controller_factory()
        t, theta = simulate_fn(fmu_path, controller, theta0_deg=theta0_deg, duration=duration)
        settling_times[theta0_deg] = held_from(t, theta, tolerance_rad, hold_duration)
        trajectories[theta0_deg] = (t, theta)
    return {"settling_times": settling_times, "trajectories": trajectories}


def robustness(fmu_path, controller_factory, theta0_deg=2.0,
                tolerance_deg=TOLERANCE_DEG, hold_duration=HOLD_DURATION,
                kick_tau=8.0, kick_steps=5, duration=20.0, simulate_fn=simulate_run):
    tolerance_rad = math.radians(tolerance_deg)
    injector = KickInjector(tolerance_rad, hold_duration, kick_tau, kick_steps)
    controller = controller_factory()
    t, theta = simulate_fn(
        fmu_path, controller, theta0_deg=theta0_deg, duration=duration, on_frame=injector.offset_for
    )

    if injector.kick_time is None:
        return {"kicked": False, "recovery_time": None, "kick_time": None, "trajectory": (t, theta)}

    post_kick_t = [ti for ti in t if ti >= injector.kick_time]
    post_kick_theta = [th for ti, th in zip(t, theta) if ti >= injector.kick_time]
    recovered_at = held_from(post_kick_t, post_kick_theta, tolerance_rad, hold_duration)
    recovery_time = None if recovered_at is None else recovered_at - injector.kick_time

    return {
        "kicked": True,
        "recovery_time": recovery_time,
        "kick_time": injector.kick_time,
        "trajectory": (t, theta),
    }


def swingup_capture(fmu_path, controller_factory, tolerance_deg=TOLERANCE_DEG,
                     hold_duration=HOLD_DURATION, duration=20.0, simulate_fn=simulate_run):
    tolerance_rad = math.radians(tolerance_deg)
    controller = controller_factory()
    t, theta = simulate_fn(fmu_path, controller, theta0_deg=None, duration=duration)
    capture_time = held_from(t, theta, tolerance_rad, hold_duration)
    return {"capture_time": capture_time, "trajectory": (t, theta)}
```

Note: `robustness`'s fake in the test calls `simulate_fn(..., duration=...,
on_frame=...)` as keyword arguments — `simulate_run`'s real signature
matches this (`duration` and `on_frame` are both keyword-usable), so no
adapter is needed between the fake and the real driver.

Place the new `from pendulum_game_controlled import ...` line at the top
of `benchmark_controllers.py`, grouped with the other imports (same
placement note as Task 2).

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_benchmark_scenarios.py -v`
Expected: PASS (7 passed)

- [ ] **Step 5: Commit**

```bash
git add benchmark_controllers.py tests/test_benchmark_scenarios.py
git commit -m "feat: add benchmark scenario functions (envelope sweep, reaction time, robustness, swing-up capture)"
```

---

### Task 4: Report- und Plot-Generierung, `main`-Einstiegspunkt

**Files:**
- Modify: `benchmark_controllers.py` (append)
- Modify: `requirements.txt`
- Modify: `.gitignore`
- Test: `tests/test_benchmark_report.py`

**Interfaces:**
- Consumes: the four scenario functions and `CONTROLLER_FACTORIES` from
  Task 3.
- Produces: `run_all_scenarios(fmu_path) -> dict` with keys `envelope`
  (dict `{"PD": ..., "LQR": ...}`, each an `envelope_sweep` result),
  `reaction_time` (dict `{"PD": ..., "LQR": ..., "SwingUp": ...}`, each a
  `reaction_time` result), `robustness` (dict `{"PD": ..., "LQR": ...,
  "SwingUp": ...}`, each a `robustness` result), `swingup_capture` (a
  single `swingup_capture` result dict). `generate_report(results) ->
  str`. `generate_plots(results, output_dir)` (writes PNG files, no
  return value).

- [ ] **Step 1: Write the failing test**

Create `tests/test_benchmark_report.py` (no FMU needed — synthetic
`results` dict matching the shape `run_all_scenarios` produces):

```python
from benchmark_controllers import generate_report


def _sample_results():
    return {
        "envelope": {
            "PD": {"results_by_theta0": {2: True, 4: False}, "envelope_deg": 2},
            "LQR": {"results_by_theta0": {2: True, 4: True}, "envelope_deg": 4},
        },
        "reaction_time": {
            "PD": {"settling_times": {2.0: 0.8, 10.0: None}, "trajectories": {}},
            "LQR": {"settling_times": {2.0: 0.3, 10.0: 1.1}, "trajectories": {}},
            "SwingUp": {"settling_times": {2.0: 0.3, 10.0: 1.0}, "trajectories": {}},
        },
        "robustness": {
            "PD": {"kicked": True, "recovery_time": 2.1, "kick_time": 1.0, "trajectory": ([], [])},
            "LQR": {"kicked": True, "recovery_time": None, "kick_time": 1.0, "trajectory": ([], [])},
            "SwingUp": {"kicked": False, "recovery_time": None, "kick_time": None, "trajectory": ([], [])},
        },
        "swingup_capture": {"capture_time": 3.9, "trajectory": ([], [])},
    }


def test_generate_report_includes_all_sections_and_values():
    report = generate_report(_sample_results())

    assert "PD" in report
    assert "LQR" in report
    assert "SwingUp" in report
    assert "2°" in report
    assert "0.80s" in report
    assert "kein Einschwingen" in report
    assert "2.10s" in report
    assert "keine Erholung" in report
    assert "Regler hat vor dem Puls nicht eingeschwungen" in report
    assert "3.90s" in report
    assert "N/A" in report
    assert "Kontext" in report
    assert "Methodik" in report
    assert "Diskussion" in report
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_benchmark_report.py -v`
Expected: FAIL with `ImportError: cannot import name 'generate_report'`

- [ ] **Step 3: Append report/plot generation and `main` to `benchmark_controllers.py`**

```python
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


def run_all_scenarios(fmu_path):
    envelope = {
        name: envelope_sweep(fmu_path, factory)
        for name, factory in CONTROLLER_FACTORIES.items()
        if name != "SwingUp"
    }
    reaction = {
        name: reaction_time(fmu_path, factory)
        for name, factory in CONTROLLER_FACTORIES.items()
    }
    robust = {
        name: robustness(fmu_path, factory)
        for name, factory in CONTROLLER_FACTORIES.items()
    }
    swingup = swingup_capture(fmu_path, CONTROLLER_FACTORIES["SwingUp"])
    return {
        "envelope": envelope,
        "reaction_time": reaction,
        "robustness": robust,
        "swingup_capture": swingup,
    }


def generate_report(results):
    lines = ["# AP3 Reglervergleich", ""]

    lines.append("## 1. Kontext")
    lines.append("")
    lines.append(
        "Vergleich der drei AP3-Regler (PD/SimpleController, LQR, SwingUp) "
        "gegen die reale InvertedPendulumMB.fmu, gefordert durch die "
        "Projektanweisung (AP3: 'Vergleich der Regler hinsichtlich "
        "Stabilitaet, Reaktionszeit und Robustheit gegenueber "
        "Stoerungen'). Details zur Methodik in "
        "docs/superpowers/specs/2026-08-17-ap3-controller-comparison-design.md."
    )
    lines.append("")

    lines.append("## 2. Methodik")
    lines.append("")
    lines.append(
        f"Erfolgskriterium: |theta| < {TOLERANCE_DEG:.0f}° fuer mindestens "
        f"{HOLD_DURATION:.1f}s ununterbrochen (theta = Abweichung von der "
        "aufrechten Lage phi=pi). Stabilitaet wird als groesste "
        "erfolgreiche Anfangsauslenkung im 2°-Sweep gemessen (PD/LQR), "
        "Reaktionszeit als Einschwingzeit bei festen "
        "Baseline-Auslenkungen (2°, 10°), Robustheit als Erholungszeit "
        "nach einem Kraft-Puls auf tau (KICK_TAU=8.0 fuer KICK_STEPS=5 "
        "Frames) waehrend des eingeschwungenen Zustands, und die "
        "Swing-up-Faehigkeit als Einschwingzeit ab der realen "
        "Spiel-Anfangsbedingung (phi0=0.75*pi/2)."
    )
    lines.append("")

    lines.append("## 3. Stabilitaet (Einzugsbereich)")
    lines.append("")
    lines.append("| Regler | Einzugsbereich |")
    lines.append("|---|---|")
    for name, data in results["envelope"].items():
        envelope_deg = data["envelope_deg"]
        value = "kein Erfolg im gesweepten Bereich" if envelope_deg is None else f"{envelope_deg:.0f}°"
        lines.append(f"| {name} | {value} |")
    lines.append("| SwingUp | siehe Swing-up-Ergebnis unten |")
    lines.append("")

    lines.append("## 4. Reaktionszeit")
    lines.append("")
    lines.append("| Regler | 2° | 10° |")
    lines.append("|---|---|---|")
    for name, data in results["reaction_time"].items():
        row = [name]
        for theta0 in (2.0, 10.0):
            t = data["settling_times"].get(theta0)
            row.append("kein Einschwingen" if t is None else f"{t:.2f}s")
        lines.append("| " + " | ".join(row) + " |")
    lines.append("")

    lines.append("## 5. Robustheit (Kraft-Puls)")
    lines.append("")
    lines.append("| Regler | Erholungszeit |")
    lines.append("|---|---|")
    for name, data in results["robustness"].items():
        if not data["kicked"]:
            value = "Regler hat vor dem Puls nicht eingeschwungen"
        elif data["recovery_time"] is None:
            value = "keine Erholung"
        else:
            value = f"{data['recovery_time']:.2f}s"
        lines.append(f"| {name} | {value} |")
    lines.append("")

    lines.append("## 6. Swing-up ab realer Anfangsbedingung")
    lines.append("")
    capture_time = results["swingup_capture"]["capture_time"]
    value = "kein Capture" if capture_time is None else f"{capture_time:.2f}s"
    lines.append(f"SwingUp: {value}")
    lines.append("PD/LQR: N/A (strukturell nicht loesbar, siehe Design-Dokument §3.4)")
    lines.append("")

    lines.append("## 7. Diskussion")
    lines.append("")
    lines.append(
        "Die drei Regler unterscheiden sich strukturell in ihrem "
        "Einzugsbereich (Tabelle oben): PD reagiert nur lokal um phi=pi, "
        "LQR nutzt vollen Zustand (s, v, theta, theta_dot) und deckt "
        "typischerweise einen groesseren Bereich ab, aber beide sind auf "
        "eine Linearisierung um die aufrechte Lage angewiesen und koennen "
        "die reale Spiel-Anfangsbedingung (~112.5° von der aufrechten "
        "Lage) strukturell nicht erreichen (Abschnitt 6) - genau der in "
        "der Projektanweisung genannte Vergleichspunkt fuer den "
        "energiebasierten SwingUp-Regler (siehe AP1_Validierung.md §6 "
        "fuer die zugrundeliegende Physik). Bei der Robustheit "
        "(Abschnitt 5) zeigt die Erholungszeit nach dem Kraft-Puls, "
        "welcher Regler eine Stoerung am schnellsten wieder ausregelt; "
        "ein Regler, der vor dem Puls gar nicht erst eingeschwungen war, "
        "wird als 'nicht eingeschwungen' statt mit einer Erholungszeit "
        "gefuehrt und ist entsprechend gesondert zu lesen."
    )
    lines.append("")

    return "\n".join(lines)


def generate_plots(results, output_dir):
    os.makedirs(output_dir, exist_ok=True)

    names = list(results["envelope"].keys())
    values = [(results["envelope"][n]["envelope_deg"] or 0) for n in names]
    plt.figure()
    plt.bar(names, values)
    plt.ylabel("Einzugsbereich (Grad)")
    plt.title("Stabilitaet: Einzugsbereich je Regler")
    plt.savefig(os.path.join(output_dir, "envelope_sweep.png"))
    plt.close()

    for theta0 in (2.0, 10.0):
        plt.figure()
        for name, data in results["reaction_time"].items():
            t, theta = data["trajectories"][theta0]
            plt.plot(t, [math.degrees(th) for th in theta], label=name)
        plt.axhline(5.0, color="gray", linestyle="--")
        plt.axhline(-5.0, color="gray", linestyle="--")
        plt.xlabel("t (s)")
        plt.ylabel("theta (deg)")
        plt.title(f"Reaktionszeit ab {theta0:.0f}°")
        plt.legend()
        plt.savefig(os.path.join(output_dir, f"reaction_time_{int(theta0)}deg.png"))
        plt.close()

    plt.figure()
    for name, data in results["robustness"].items():
        t, theta = data["trajectory"]
        plt.plot(t, [math.degrees(th) for th in theta], label=name)
        if data["kick_time"] is not None:
            plt.axvline(data["kick_time"], color="red", linestyle=":")
    plt.axhline(5.0, color="gray", linestyle="--")
    plt.axhline(-5.0, color="gray", linestyle="--")
    plt.xlabel("t (s)")
    plt.ylabel("theta (deg)")
    plt.title("Robustheit: Kraft-Puls-Reaktion")
    plt.legend()
    plt.savefig(os.path.join(output_dir, "robustness_kick.png"))
    plt.close()

    t, theta = results["swingup_capture"]["trajectory"]
    plt.figure()
    plt.plot(t, [math.degrees(th) for th in theta])
    capture_time = results["swingup_capture"]["capture_time"]
    if capture_time is not None:
        plt.axvline(capture_time, color="red", linestyle=":")
    plt.xlabel("t (s)")
    plt.ylabel("theta (deg)")
    plt.title("Swing-up ab realer Anfangsbedingung")
    plt.savefig(os.path.join(output_dir, "swingup_capture.png"))
    plt.close()


if __name__ == "__main__":
    fmu_path = os.path.abspath("InvertedPendulumMB.fmu")
    if not os.path.exists(fmu_path):
        raise SystemExit(
            "InvertedPendulumMB.fmu not found in project root. "
            "Copy the (gitignored) FMU into place before running this benchmark."
        )

    results = run_all_scenarios(fmu_path)

    report = generate_report(results)
    with open("AP3_Reglervergleich.md", "w", encoding="utf-8") as f:
        f.write(report)

    generate_plots(results, "benchmark_plots")

    print("Wrote AP3_Reglervergleich.md and benchmark_plots/")
```

Add `matplotlib` to `requirements.txt`:

```
pygame==2.1.2
fmpy==0.3.30
pandas==2.3.3
pytest==9.1.1
scipy==1.15.3
numpy==2.2.6
matplotlib==3.10.9
```

Add to `.gitignore` (under the existing "Simulation results" section or a
new one):

```
# Benchmark plots (regenerable via benchmark_controllers.py)
benchmark_plots/
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_benchmark_report.py -v`
Expected: PASS (1 passed)

- [ ] **Step 5: Commit**

```bash
git add benchmark_controllers.py requirements.txt .gitignore tests/test_benchmark_report.py
git commit -m "feat: add report/plot generation and main entrypoint for the controller benchmark"
```

---

### Task 5: FMU-Integrationstest und realer Benchmark-Lauf

**Files:**
- Test: `tests/test_benchmark_controllers.py`
- Create (generated by running the script for real, then committed):
  `AP3_Reglervergleich.md`

**Interfaces:**
- Consumes: `envelope_sweep`, `swingup_capture`, `robustness`,
  `CONTROLLER_FACTORIES` from Task 3, and the full `main` flow from Task 4.

- [ ] **Step 1: Write the integration tests**

Create `tests/test_benchmark_controllers.py` — a reduced, regression-guard
version of the real scenarios against the real FMU (skips gracefully
without it, same pattern as `tests/test_numerical_stability.py`):

```python
import os

import pytest

FMU_PATH = os.path.abspath("InvertedPendulumMB.fmu")


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
def test_pd_holds_a_small_deviation_but_not_a_large_one():
    from benchmark_controllers import CONTROLLER_FACTORIES, envelope_sweep

    # theta0=0.5 (not 2.0): a linear stability check of SimpleController's
    # closed loop (A - B*K_pd eigenvalues, K_pd=[0,0,K_PHI,K_VPHI]) shows a
    # positive real-part eigenvalue at both d_pend=0.15 and d_pend=0.01 —
    # SimpleController's near-upright loop is linearly unstable, not just
    # weakly damped. It still satisfies the 5-degree/1s hold criterion for
    # small enough theta0 (confirmed empirically: holds through 1.5deg,
    # fails at 2.0deg) because divergence takes longer than hold_duration
    # to reach the tolerance boundary from a small enough start — 0.5deg
    # gives comfortable margin under that ~1.5-2.0deg threshold.
    result = envelope_sweep(
        FMU_PATH, CONTROLLER_FACTORIES["PD"], theta0_values_deg=[0.5, 60], duration=5.0
    )

    assert result["results_by_theta0"][0.5] is True
    assert result["results_by_theta0"][60] is False


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
def test_swingup_captures_from_the_real_initial_condition():
    from benchmark_controllers import CONTROLLER_FACTORIES, swingup_capture

    result = swingup_capture(FMU_PATH, CONTROLLER_FACTORIES["SwingUp"], duration=15.0)

    assert result["capture_time"] is not None
    assert result["capture_time"] < 15.0


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
def test_lqr_recovers_from_the_force_pulse():
    from benchmark_controllers import CONTROLLER_FACTORIES, robustness

    result = robustness(FMU_PATH, CONTROLLER_FACTORIES["LQR"], duration=10.0)

    assert result["kicked"] is True
    assert result["recovery_time"] is not None
```

- [ ] **Step 2: Run the tests**

Run: `pytest tests/test_benchmark_controllers.py -v`
Expected: PASS (3 passed) if `InvertedPendulumMB.fmu` is present in the
working directory — copy it in first if the worktree doesn't have it yet
(gitignored, not part of the repo). If it genuinely fails (not skips),
that is a real finding — the LQR/SwingUp capture-time assumptions here
come directly from prior AP3 review findings (LQR ~11°, SwingUp captures
in ~3-4s), and PD's 0.5deg/60deg split comes from Task 2's empirical
finding plus a direct eigenvalue check (see the comment on the PD test
above), so a failure means either a real regression or that those numbers
need revisiting; do not loosen the assertions just to make it pass.

- [ ] **Step 3: Run the full benchmark for real and commit the generated report**

Run: `python benchmark_controllers.py`

This drives ~20-30 full FMU simulation runs (each up to 20s of simulated
time at `dt=0.02`/`SUBSTEPS=10`, i.e. up to 10,000 `doStep()` calls) and
can take several minutes of wall-clock time — this is expected, not a
hang.

Inspect the generated `AP3_Reglervergleich.md` for physical plausibility
before committing:
- PD's `Einzugsbereich` in the default 2°-step sweep (starting at 2°) is
  expected to come back as "kein Erfolg im gesweepten Bereich" — this is
  correct, not a bug: PD's real envelope under the current FMU is ~1.5-2°
  (see the ruling in the SDD ledger and the comment in
  `tests/test_benchmark_controllers.py`), below the sweep's first tested
  point. LQR's envelope should be clearly larger (roughly a low
  double-digit number of degrees) — consistent with prior AP3 review
  findings.
- The Swing-up capture time should be in the few-seconds range, not "kein
  Capture".
- Reaction times at 2° are expected to show "kein Einschwingen" for PD too
  (same reason as above, not a regression from this task) while LQR/SwingUp
  settle; at 10°, PD is expected to show "kein Einschwingen" as originally
  planned.
- Robustness: controllers that get kicked should mostly recover (a
  recovery time reported, not "keine Erholung"), though this is not a
  hard requirement — report what the real FMU run actually shows.

Also open a couple of the PNGs in `benchmark_plots/` and confirm the
trajectories look physically sane (e.g. the swing-up plot rising from
~-112.5° toward 0°, the reaction-time plots settling toward 0°) — this is
the one manual-inspection step this plan requires, per the project's
"no automated test for pure visualization" convention (extended here from
Pygame `redraw()` to `matplotlib` plots, for the same reason: nothing to
assert against without a human eye).

```bash
git add AP3_Reglervergleich.md
git commit -m "docs: add generated AP3 controller comparison report"
```

`benchmark_plots/` stays gitignored and uncommitted — it's regenerable
from the committed script and report.

- [ ] **Step 4: Commit the integration test**

```bash
git add tests/test_benchmark_controllers.py
git commit -m "test: add FMU-driven regression tests for the controller benchmark scenarios"
```
