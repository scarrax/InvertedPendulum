# AP3 Teil 1 (LQR-Regler) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an `LQRController` (full-state LQR: `s`, `v`, `θ`, `θ̇`) to `pendulum_game_controlled.py`, selectable via a new `L` key alongside the existing `SimpleController`, without touching the `Controller` base class or `SimpleController`.

**Architecture:** All changes live in the single existing `pendulum_game_controlled.py` script, following its current flat-function style. One new pure function (`compute_lqr_gain`) computes the LQR gain matrix via `scipy.linalg.solve_continuous_are`, testable without a display or an FMU. `LQRController` wraps it behind the existing `Controller` interface. Game-loop integration (key handling, controller selection, badge) is verified manually, since it requires the Pygame window.

**Tech Stack:** Python 3.10, `numpy`, `scipy` (new dependencies — see Task 1), `pygame`, `fmpy`, `pytest`.

## Global Constraints

- Winkelkonvention bleibt bestehen: `phi = 0` ist die stabile, hängende Ruhelage, `phi = math.pi` die instabile, aufrechte Zielposition. `LQRController` berechnet `theta = (phi_fmu % (2*math.pi)) - math.pi` — dieselbe Wrapping-Formel wie `SimpleController.compute`'s `phi_err` (`pendulum_game_controlled.py:320`).
- Die Linearisierung basiert auf den **korrigierten, gekoppelten** Bewegungsgleichungen aus `AP1_Validierung.md` Abschnitt 6, nicht auf dem flachen Modell (`pendulum.mo`). Exakte Matrizen siehe Task 1.
- `Controller` (Basisklasse) und `SimpleController` bleiben unverändert — `LQRController` ist eine neue, eigenständige Unterklasse von `Controller`.
- Die MultiBody-FMU (`InvertedPendulumMB.fmu`) bleibt unverändert (Euler-Solver, siehe `AP1_Validierung.md` Abschnitt 3.3) — daran wird nichts geändert.
- Die Verstärkungsmatrix `K` wird zur Laufzeit via `scipy.linalg.solve_continuous_are` berechnet, nicht offline vorberechnet und hartkodiert.
- Kein Swing-up, kein formaler Reglervergleich, keine automatische Deaktivierung außerhalb des Einzugsbereichs — alles explizit außerhalb dieses Plans (siehe Design-Dokument Abschnitt 8).

---

## Task 1: Dependencies und `compute_lqr_gain()`

**Files:**
- Create: `requirements.txt`
- Create: `tests/test_lqr_gain.py`
- Modify: `pendulum_game_controlled.py` (neue Imports `numpy`/`scipy.linalg`; neue Funktion `compute_lqr_gain`, eingefügt zwischen der Zeile `# TODO: some other controllers` und `K_STABILITY = 0.5`)

**Interfaces:**
- Produces: `compute_lqr_gain(M, m, l, g, d_cart, d_pend, Q, R) -> np.ndarray` — reine Funktion, keine Seiteneffekte. `Q` ist eine `(4,4)`-, `R` eine `(1,1)`-`np.ndarray`. Rückgabe: `K` mit Shape `(1,4)`.

- [ ] **Step 1: `requirements.txt` anlegen**

Create `requirements.txt`:

```
pygame
fmpy
pandas
pytest
scipy
numpy
```

- [ ] **Step 2: Dependencies installieren**

Run: `python -m pip install -r requirements.txt`
Expected: Installation erfolgreich (`python -m pip show scipy numpy` zeigt beide danach an).

- [ ] **Step 3: Fehlschlagenden Test schreiben**

Create `tests/test_lqr_gain.py`:

```python
import numpy as np

from pendulum_game_controlled import compute_lqr_gain

M, m, l, g, d_cart, d_pend = 5, 0.5, 0.5, 9.81, 0.15, 0.15
Q = np.diag([1.0, 1.0, 10.0, 1.0])
R = np.array([[1.0]])


def linearized_matrices():
    A = np.array([
        [0, 1, 0, 0],
        [0, -d_cart / M, m * g / M, -d_pend / (M * l)],
        [0, 0, 0, 1],
        [0, -d_cart / (l * M), (M + m) * g / (l * M), -(M + m) * d_pend / (m * l**2 * M)],
    ])
    B = np.array([[0], [1 / M], [0], [1 / (l * M)]])
    return A, B


def test_closed_loop_is_stable():
    K = compute_lqr_gain(M, m, l, g, d_cart, d_pend, Q, R)
    A, B = linearized_matrices()

    closed_loop = A - B @ K
    eigenvalues = np.linalg.eigvals(closed_loop)

    assert (eigenvalues.real < 0).all()


def test_gain_shape():
    K = compute_lqr_gain(M, m, l, g, d_cart, d_pend, Q, R)
    assert K.shape == (1, 4)
```

- [ ] **Step 4: Test laufen lassen, Fehlschlag bestätigen**

Run: `python -m pytest tests/test_lqr_gain.py -v`
Expected: FAIL mit `ImportError: cannot import name 'compute_lqr_gain' from 'pendulum_game_controlled'` (Funktion existiert noch nicht).

- [ ] **Step 5: Imports ergänzen**

Am Anfang von `pendulum_game_controlled.py`, nach der bestehenden Zeile `import shutil`, ergänzen:

```python
import numpy as np
import scipy.linalg
```

- [ ] **Step 6: Funktion implementieren**

In `pendulum_game_controlled.py`, zwischen der Zeile `# TODO: some other controllers` und `K_STABILITY = 0.5`, einfügen:

```python
def compute_lqr_gain(M, m, l, g, d_cart, d_pend, Q, R):
    A = np.array([
        [0, 1, 0, 0],
        [0, -d_cart / M, m * g / M, -d_pend / (M * l)],
        [0, 0, 0, 1],
        [0, -d_cart / (l * M), (M + m) * g / (l * M), -(M + m) * d_pend / (m * l**2 * M)],
    ])
    B = np.array([[0], [1 / M], [0], [1 / (l * M)]])
    P = scipy.linalg.solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K


```

- [ ] **Step 7: Test laufen lassen, Erfolg bestätigen**

Run: `python -m pytest tests/test_lqr_gain.py -v`
Expected: 2 passed.

- [ ] **Step 8: Commit**

```bash
git add requirements.txt tests/test_lqr_gain.py pendulum_game_controlled.py
git commit -m "feat: add LQR gain computation via Riccati equation"
```

---

## Task 2: `LQRController`

**Files:**
- Create: `tests/test_lqr_controller.py`
- Modify: `pendulum_game_controlled.py` (neue Klasse `LQRController`, direkt nach `compute_lqr_gain`, vor `K_STABILITY = 0.5`)

**Interfaces:**
- Consumes: `compute_lqr_gain(M, m, l, g, d_cart, d_pend, Q, R) -> np.ndarray` (Task 1); `Controller`-Basisklasse (`pendulum_game_controlled.py:307-310`, unverändert).
- Produces: `LQRController` (Unterklasse von `Controller`), `LQRController().compute(phi_fmu, vphi, s, v) -> float` — Wert bereits auf `[-MAX_TAU, MAX_TAU]` geclampt, gleiche Signatur wie `SimpleController.compute`.

- [ ] **Step 1: Fehlschlagenden Test schreiben**

Create `tests/test_lqr_controller.py`:

```python
import math

from pendulum_game_controlled import LQRController


def test_zero_tau_at_equilibrium():
    controller = LQRController()
    tau = controller.compute(phi_fmu=math.pi, vphi=0.0, s=0.0, v=0.0)
    assert abs(tau) < 1e-9


def test_tau_clamped_to_max():
    controller = LQRController()
    tau = controller.compute(phi_fmu=0.0, vphi=50.0, s=100.0, v=50.0)
    assert -controller.MAX_TAU <= tau <= controller.MAX_TAU


def test_returns_plain_float():
    controller = LQRController()
    tau = controller.compute(phi_fmu=math.pi, vphi=0.1, s=0.0, v=0.0)
    assert isinstance(tau, float)
```

- [ ] **Step 2: Test laufen lassen, Fehlschlag bestätigen**

Run: `python -m pytest tests/test_lqr_controller.py -v`
Expected: FAIL mit `ImportError: cannot import name 'LQRController' from 'pendulum_game_controlled'`.

- [ ] **Step 3: Klasse implementieren**

In `pendulum_game_controlled.py`, direkt nach der in Task 1 eingefügten `compute_lqr_gain`-Funktion (vor `K_STABILITY = 0.5`), einfügen:

```python
class LQRController(Controller):
    M = 5
    m = 0.5
    l = 0.5
    g = 9.81
    d_cart = 0.15
    d_pend = 0.15
    Q = np.diag([1.0, 1.0, 10.0, 1.0])
    R = np.array([[1.0]])
    MAX_TAU = 10.0

    def __init__(self):
        self.K = compute_lqr_gain(
            self.M, self.m, self.l, self.g, self.d_cart, self.d_pend, self.Q, self.R
        )

    def compute(self, phi_fmu, vphi, s, v):
        theta = (phi_fmu % (2 * math.pi)) - math.pi
        x = np.array([[s], [v], [theta], [vphi]])
        tau = (-self.K @ x).item()
        return max(-self.MAX_TAU, min(self.MAX_TAU, tau))


```

- [ ] **Step 4: Test laufen lassen, Erfolg bestätigen**

Run: `python -m pytest tests/test_lqr_controller.py -v`
Expected: 3 passed.

- [ ] **Step 5: Vollen Testlauf bestätigen**

Run: `python -m pytest -v`
Expected: alle Tests grün (bisherige AP1/AP2-Tests + die beiden neuen Dateien aus Task 1/2), keine Warnings.

- [ ] **Step 6: Commit**

```bash
git add tests/test_lqr_controller.py pendulum_game_controlled.py
git commit -m "feat: add LQRController"
```

---

## Task 3: Spiel-Integration (Taste `L`, Regler-Umschaltung, Badge)

**Files:**
- Modify: `pendulum_game_controlled.py` (`redraw`, `run_game`)

**Interfaces:**
- Consumes: `LQRController` (Task 2), `SimpleController` (unverändert, `pendulum_game_controlled.py:313-326`).
- Produces: `redraw(..., controller_name="SimpleController")` — neuer optionaler Parameter, alle Aufrufstellen aktualisiert. Keine neue öffentliche Schnittstelle sonst.

- [ ] **Step 1: `redraw()`-Signatur und Badge-Label anpassen**

```python
def redraw(screen, time, dt, score, precision, s, v, taus, phis, auto_mode=False, paused=False):
```

ersetzen durch:

```python
def redraw(screen, time, dt, score, precision, s, v, taus, phis, auto_mode=False, paused=False, controller_name="SimpleController"):
```

Die bestehende Zeile

```python
    badge_label = "AUTO  [H to disable]" if auto_mode else "MANUAL  [H for auto]"
```

ersetzen durch:

```python
    badge_label = (
        f"AUTO [{controller_name}]  [H to disable]" if auto_mode else "MANUAL  [H for auto]"
    )
```

- [ ] **Step 2: `run_game()` um Regler-Auswahl erweitern**

Die bestehende Zeile

```python
    taus, phis = [], []
    controller = SimpleController()
    clock = pygame.time.Clock()
```

ersetzen durch:

```python
    taus, phis = [], []
    controllers = {"SimpleController": SimpleController(), "LQR": LQRController()}
    controller_name = "SimpleController"
    clock = pygame.time.Clock()
```

Im Event-Loop, nach der bestehenden `R`-Abfrage, ergänzen:

```python
            if event.type == pygame.KEYDOWN and event.key == pygame.K_l:
                controller_name = (
                    "LQR" if controller_name == "SimpleController" else "SimpleController"
                )
```

Die bestehende Zeile

```python
            tau = controller.compute(phi, vphi, s, v)
```

ersetzen durch:

```python
            tau = controllers[controller_name].compute(phi, vphi, s, v)
```

- [ ] **Step 3: Alle drei `redraw(...)`-Aufrufstellen um `controller_name` erweitern**

Die bestehende Zeile (einmaliger Aufruf vor der `while`-Schleife)

```python
    redraw(screen, time, dt, 0, 0.25, s, v, [phi], [vphi], auto_mode, paused)
```

ersetzen durch:

```python
    redraw(screen, time, dt, 0, 0.25, s, v, [phi], [vphi], auto_mode, paused, controller_name)
```

Die bestehende Zeile (im Pause-Skip-Block)

```python
            redraw(screen, time, dt, score, 0.25, s, v, plot_taus, plot_phis, auto_mode, paused)
```

ersetzen durch:

```python
            redraw(screen, time, dt, score, 0.25, s, v, plot_taus, plot_phis, auto_mode, paused, controller_name)
```

Die bestehende Zeile (am Ende der `while`-Schleife)

```python
        redraw(screen, time, dt, score, 0.25, s, v, taus, phis, auto_mode, paused)
```

ersetzen durch:

```python
        redraw(screen, time, dt, score, 0.25, s, v, taus, phis, auto_mode, paused, controller_name)
```

- [ ] **Step 4: Manuell verifizieren**

Run: `python pendulum_game_controlled.py`
Erwartet: Spiel startet wie zuvor. Taste `L` schaltet (unabhängig vom Auto/Manuell-Zustand) zwischen `SimpleController` und `LQR` um — sichtbar am AUTO-Badge (`AUTO [SimpleController]` bzw. `AUTO [LQR]`), sobald `H` aktiviert ist. Pendel von Hand nah an die aufrechte Lage (`φ≈π`) bringen und dort `H` aktivieren: mit `LQR` ausgewählt balanciert der Regler das Pendel und zentriert den Wagen auf `s≈0`; weit weg von `φ=π` (z. B. direkt beim Rundenstart) liefert `LQR` erwartungsgemäß kein sinnvolles Verhalten (Swing-up ist nicht Teil dieses Plans). Kein Absturz in keiner Kombination aus `H`/`P`/`R`/`L`.

- [ ] **Step 5: Commit**

```bash
git add pendulum_game_controlled.py
git commit -m "feat: add L key to switch between SimpleController and LQRController"
```
