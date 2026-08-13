# AP3 Teil 2 (Swing-up-Regler) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `SwingUpController` to `pendulum_game_controlled.py` that energetically swings the pendulum from the real start condition (`phi0 ≈ 67.5°`) up into the existing `LQRController`'s capture region and then automatically hands control over to it (with hysteresis for falling back out again), selectable as a third option via the existing `L` key.

**Architecture:** All changes live in the single existing `pendulum_game_controlled.py` script, following its current flat-function/flat-class style. A new pure function `pendulum_energy(m, l, g, phi, vphi)` computes pendulum energy, testable without a display or an FMU. `SwingUpController` wraps the energy-pumping law (exact inversion via the already-validated coupled equations from `AP1_Validierung.md` §6.2, no approximation) behind the existing `Controller` interface, and composes an internal `LQRController` instance for the balancing phase, switching between the two via a small hysteresis state machine (`self.mode`). Game-loop integration (third `controllers` entry, badge submode display) is verified manually, since it requires the Pygame window.

**Tech Stack:** Python 3.10, `numpy`, `scipy` (already project dependencies since AP3 Teil 1), `pygame`, `fmpy`, `pytest`.

## Global Constraints

- Winkelkonvention bleibt bestehen: `phi = 0` ist die stabile, hängende Ruhelage, `phi = math.pi` die instabile, aufrechte Zielposition.
- Die Energie- und `τ`-Formeln basieren exakt auf den bereits validierten, gekoppelten Bewegungsgleichungen aus `AP1_Validierung.md` §6.2 — keine Näherung (siehe `docs/superpowers/specs/2026-08-13-ap3-swingup-controller-design.md` Abschnitt 2 für die vollständige Herleitung). Exakte Formeln siehe Task 1.
- `Controller`, `SimpleController` und `LQRController` bleiben unverändert — `SwingUpController` ist eine neue, eigenständige Unterklasse von `Controller`, die eine `LQRController`-Instanz komponiert (nicht dupliziert).
- Die MultiBody-FMU (`InvertedPendulumMB.fmu`) bleibt unverändert (Euler-Solver, siehe `AP1_Validierung.md` Abschnitt 3.3).
- `K_ENERGY = 3.0`, `CAPTURE_THETA = 10°`, `CAPTURE_VPHI = 2.0`, `RELEASE_THETA = 25°` sind die für diesen Plan festgelegten Startwerte (siehe Task 1) — kein systematisches Tuning darüber hinaus, kein formaler Reglervergleich zwischen `SimpleController`/`LQRController`/`SwingUpController` — beides eigene, spätere AP3-Teilstücke (siehe Design-Dokument Abschnitt 6).
- Keine eigene Wagenpositionsregelung während der Swing-up-Phase — `s` wird erst nach dem Übergang zu `LQRController` mitgeregelt (dessen bestehendes Verhalten, unverändert).

---

## Task 1: `pendulum_energy()` und `SwingUpController`

**Files:**
- Create: `tests/test_swingup_controller.py`
- Modify: `pendulum_game_controlled.py` (neue Funktion `pendulum_energy`, neue Klasse `SwingUpController`, eingefügt direkt nach `LQRController` (endet `pendulum_game_controlled.py:369`), vor `K_STABILITY = 0.5`)

**Interfaces:**
- Consumes: `LQRController` (`pendulum_game_controlled.py:348-369`, unverändert) — insbesondere `LQRController().compute(phi_fmu, vphi, s, v) -> float`; `Controller`-Basisklasse (`pendulum_game_controlled.py:312-315`, unverändert).
- Produces: `pendulum_energy(m, l, g, phi, vphi) -> float` — reine Funktion. `SwingUpController` (Unterklasse von `Controller`), `SwingUpController().compute(phi_fmu, vphi, s, v) -> float` (bereits auf `[-MAX_TAU, MAX_TAU]` geclampt, gleiche Signatur wie `SimpleController.compute`/`LQRController.compute`), öffentliches Attribut `.mode` (Werte `"swingup"` oder `"lqr"`, startet bei `"swingup"`).

- [ ] **Step 1: Fehlschlagenden Test schreiben**

Create `tests/test_swingup_controller.py`:

```python
import math

from pendulum_game_controlled import LQRController, SwingUpController, pendulum_energy


def test_energy_zero_at_hanging():
    assert pendulum_energy(m=0.5, l=0.5, g=9.81, phi=0.0, vphi=0.0) == 0.0


def test_energy_at_top_equals_two_mgl():
    m, l, g = 0.5, 0.5, 9.81
    energy = pendulum_energy(m, l, g, phi=math.pi, vphi=0.0)
    assert math.isclose(energy, 2 * m * g * l)


def test_starts_in_swingup_mode():
    controller = SwingUpController()
    assert controller.mode == "swingup"


def test_tau_pumps_energy_up_when_below_target():
    # phi=0 (hanging, far from pi), vphi=1.0: cos(phi)*vphi > 0, E << E_top,
    # so a_cmd is strongly negative -> tau saturates at -MAX_TAU.
    controller = SwingUpController()
    tau = controller.compute(phi_fmu=0.0, vphi=1.0, s=0.0, v=0.0)
    assert tau == -controller.MAX_TAU
    assert controller.mode == "swingup"


def test_tau_removes_energy_when_above_target():
    # phi=0, vphi=10.0: cos(phi)*vphi > 0, E > E_top (large vphi),
    # so a_cmd is strongly positive -> tau saturates at +MAX_TAU.
    controller = SwingUpController()
    tau = controller.compute(phi_fmu=0.0, vphi=10.0, s=0.0, v=0.0)
    assert tau == controller.MAX_TAU
    assert controller.mode == "swingup"


def test_switches_to_lqr_within_capture_region():
    controller = SwingUpController()
    phi = math.pi + math.radians(5)  # theta=5 deg < CAPTURE_THETA=10 deg
    vphi = 0.5  # < CAPTURE_VPHI=2.0

    tau = controller.compute(phi_fmu=phi, vphi=vphi, s=0.0, v=0.0)

    assert controller.mode == "lqr"
    assert tau == LQRController().compute(phi_fmu=phi, vphi=vphi, s=0.0, v=0.0)


def test_stays_in_lqr_within_hysteresis_band():
    controller = SwingUpController()
    controller.mode = "lqr"
    phi = math.pi + math.radians(15)  # between CAPTURE_THETA=10 and RELEASE_THETA=25

    controller.compute(phi_fmu=phi, vphi=0.5, s=0.0, v=0.0)

    assert controller.mode == "lqr"


def test_switches_back_to_swingup_outside_release_region():
    controller = SwingUpController()
    controller.mode = "lqr"
    phi = math.pi + math.radians(30)  # theta=30 deg > RELEASE_THETA=25 deg

    tau = controller.compute(phi_fmu=phi, vphi=0.5, s=0.0, v=0.0)

    assert controller.mode == "swingup"
    assert math.isclose(tau, 2.848797352523283)
```

- [ ] **Step 2: Test laufen lassen, Fehlschlag bestätigen**

Run: `python -m pytest tests/test_swingup_controller.py -v`
Expected: FAIL mit `ImportError: cannot import name 'pendulum_energy' from 'pendulum_game_controlled'` (weder `pendulum_energy` noch `SwingUpController` existieren noch).

- [ ] **Step 3: Funktion und Klasse implementieren**

In `pendulum_game_controlled.py` den bestehenden Block

```python
    def compute(self, phi_fmu, vphi, s, v):
        theta = (phi_fmu % (2 * math.pi)) - math.pi
        x = np.array([[s], [v], [theta], [vphi]])
        tau = (-self.K @ x).item()
        return max(-self.MAX_TAU, min(self.MAX_TAU, tau))


K_STABILITY = 0.5
```

ersetzen durch:

```python
    def compute(self, phi_fmu, vphi, s, v):
        theta = (phi_fmu % (2 * math.pi)) - math.pi
        x = np.array([[s], [v], [theta], [vphi]])
        tau = (-self.K @ x).item()
        return max(-self.MAX_TAU, min(self.MAX_TAU, tau))


def pendulum_energy(m, l, g, phi, vphi):
    return 0.5 * m * l**2 * vphi**2 + m * g * l * (1 - math.cos(phi))


class SwingUpController(Controller):
    # Must match InvertedPendulumMB.mo's parameters (m_cart, m_pend, l, d_cart, d_pend) — no automatic sync.
    M = 5
    m = 0.5
    l = 0.5
    g = 9.81
    d_cart = 0.15
    d_pend = 0.15
    MAX_TAU = 10.0

    # Tuning constants for the real start condition (phi0=67.5 deg, see
    # InvertedPendulumMB.mo); adjustable if interactive testing shows they need retuning.
    K_ENERGY = 3.0
    CAPTURE_THETA = math.radians(10)
    CAPTURE_VPHI = 2.0
    RELEASE_THETA = math.radians(25)

    def __init__(self):
        self.lqr = LQRController()
        self.mode = "swingup"

    def compute(self, phi_fmu, vphi, s, v):
        theta = (phi_fmu % (2 * math.pi)) - math.pi

        if self.mode == "swingup" and abs(theta) < self.CAPTURE_THETA and abs(vphi) < self.CAPTURE_VPHI:
            self.mode = "lqr"
        elif self.mode == "lqr" and abs(theta) > self.RELEASE_THETA:
            self.mode = "swingup"

        if self.mode == "lqr":
            return self.lqr.compute(phi_fmu, vphi, s, v)

        energy = pendulum_energy(self.m, self.l, self.g, phi_fmu, vphi)
        energy_top = 2 * self.m * self.g * self.l
        sign = 1.0 if math.cos(phi_fmu) * vphi >= 0 else -1.0
        a_cmd = self.K_ENERGY * (energy - energy_top) * sign
        tau = (
            a_cmd * (self.M + self.m * math.sin(phi_fmu) ** 2)
            + self.d_cart * v
            - self.m * self.l * math.sin(phi_fmu) * vphi**2
            - self.m * self.g * math.sin(phi_fmu) * math.cos(phi_fmu)
            - (self.d_pend / self.l) * math.cos(phi_fmu) * vphi
        )
        return max(-self.MAX_TAU, min(self.MAX_TAU, tau))


K_STABILITY = 0.5
```

- [ ] **Step 4: Test laufen lassen, Erfolg bestätigen**

Run: `python -m pytest tests/test_swingup_controller.py -v`
Expected: 8 passed.

- [ ] **Step 5: Vollen Testlauf bestätigen**

Run: `python -m pytest -v`
Expected: alle Tests grün (bisherige AP1/AP2/AP3-Teil-1-Tests + `tests/test_swingup_controller.py`), keine Warnings.

- [ ] **Step 6: Commit**

```bash
git add tests/test_swingup_controller.py pendulum_game_controlled.py
git commit -m "feat: add SwingUpController with energy-based swing-up and LQR handoff"
```

---

## Task 2: Spiel-Integration (dritter `L`-Zyklus-Eintrag, Badge-Submodus)

**Files:**
- Modify: `pendulum_game_controlled.py` (`run_game`)

**Interfaces:**
- Consumes: `SwingUpController` (Task 1); `controllers`-Dict und `L`-Zyklus-Logik (`pendulum_game_controlled.py:450-451,468-470`, unverändert in ihrer Mechanik).
- Produces: keine neue öffentliche Schnittstelle — rein interne Erweiterung von `run_game()`.

- [ ] **Step 1: `controllers`-Dict um `SwingUpController` erweitern**

Die bestehende Zeile

```python
    controllers = {"PD": SimpleController(), "LQR": LQRController()}
```

ersetzen durch:

```python
    controllers = {"PD": SimpleController(), "LQR": LQRController(), "SwingUp": SwingUpController()}
```

Der bestehende `L`-Zyklus (`names = list(controllers)` / `controller_name = names[(names.index(controller_name) + 1) % len(names)]`, `pendulum_game_controlled.py:468-470`) funktioniert unverändert mit dem dritten Eintrag — keine Änderung nötig.

- [ ] **Step 2: Badge-Submodus-Helper ergänzen**

Die bestehende Funktion `classify_mode` in `pendulum_game_controlled.py`:

```python
def classify_mode(auto_time, manual_time):
    if manual_time <= 0.0 and auto_time > 0.0:
        return "Auto"
    if auto_time <= 0.0 and manual_time > 0.0:
        return "Manual"
    return "Mixed"
```

direkt danach ergänzen um:

```python


def controller_display_name(controller, name):
    if not hasattr(controller, "mode"):
        return name
    label = "swinging" if controller.mode == "swingup" else "balancing"
    return f"{name}: {label}"
```

- [ ] **Step 3: Alle drei `redraw(...)`-Aufrufstellen um den Submodus erweitern**

Die bestehende Zeile (einmaliger Aufruf vor der `while`-Schleife)

```python
    redraw(screen, time, dt, 0, 0.25, s, v, [phi], [vphi], auto_mode, paused, controller_name)
```

ersetzen durch:

```python
    redraw(screen, time, dt, 0, 0.25, s, v, [phi], [vphi], auto_mode, paused, controller_display_name(controllers[controller_name], controller_name))
```

Die bestehende Zeile (im Pause-Skip-Block)

```python
            redraw(screen, time, dt, score, 0.25, s, v, plot_taus, plot_phis, auto_mode, paused, controller_name)
```

ersetzen durch:

```python
            redraw(screen, time, dt, score, 0.25, s, v, plot_taus, plot_phis, auto_mode, paused, controller_display_name(controllers[controller_name], controller_name))
```

Die bestehende Zeile (am Ende der `while`-Schleife)

```python
        redraw(screen, time, dt, score, 0.25, s, v, taus, phis, auto_mode, paused, controller_name)
```

ersetzen durch:

```python
        redraw(screen, time, dt, score, 0.25, s, v, taus, phis, auto_mode, paused, controller_display_name(controllers[controller_name], controller_name))
```

- [ ] **Step 4: Vollen Testlauf bestätigen**

Run: `python -m pytest -v`
Expected: alle Tests weiterhin grün (diese Änderungen betreffen nur `run_game()`, keine der bestehenden Testdateien importiert oder ruft `run_game` auf).

- [ ] **Step 5: Manuell verifizieren**

Run: `python pendulum_game_controlled.py`

Erwartet:
- Spiel startet wie zuvor (Startwinkel `φ≈67,5°`, keine temporäre `phi0`-Anpassung nötig — das ist genau das Szenario, für das dieser Regler gebaut ist).
- Taste `L` zyklt durch alle drei Regler: `PD` → `LQR` → `SwingUp` → `PD` …, sichtbar am Badge-Text sobald `H` (Auto-Modus) aktiv ist.
- Mit `SwingUp` ausgewählt und `H` aktiv: Badge zeigt `AUTO [SwingUp: swinging]`. Das Pendel schwingt über mehrere Perioden zunehmend höher aus, bis der Badge-Text auf `AUTO [SwingUp: balancing]` wechselt und das Pendel danach oben stabil bleibt (Wagen zentriert sich auf `s≈0`, wie vom bestehenden `LQRController` bekannt).
- Störtest: während `AUTO [SwingUp: balancing]` per Pfeiltaste (`H` kurz aus, Stoß geben, `H` wieder an — oder falls möglich direkt während Auto-Modus durch mehrfaches Drücken) das Pendel stark auslenken; Badge sollte zurück auf `swinging` wechseln und der Regler erneut hochschwingen, sobald der Einzugsbereich wieder erreicht wird.
- Kein Absturz in keiner Kombination aus `H`/`P`/`R`/`L`.

- [ ] **Step 6: Commit**

```bash
git add pendulum_game_controlled.py
git commit -m "feat: add SwingUp as third L-cycle option with badge submode display"
```
