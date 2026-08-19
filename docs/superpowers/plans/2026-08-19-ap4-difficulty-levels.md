# AP4: Schwierigkeitsgrade Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add three selectable difficulty levels (Leicht / Standard / Schwer) that vary the scoring tolerance zones and the FMU's mass/friction parameters in Manual mode, while Auto mode (the AP3 controllers) stays locked to Standard physics.

**Architecture:** All changes land in the existing flat-function-style `pendulum_game_controlled.py` (no new modules, matching the codebase's established pattern). Difficulty configuration and the FMU-parameter-application helpers are pure/near-pure module-level functions, independently testable; the scoring-tolerance parameterization is a signature change to an existing pure function; the game-loop wiring (key handling, HUD, reset integration) is the final integration task, same shape as prior APs' "wire it into `run_game()`" tasks.

**Tech Stack:** Python, Pygame, `fmpy` (FMI2 co-simulation), `pandas` (leaderboard), `pytest`.

**Spec:** `docs/superpowers/specs/2026-08-19-ap4-difficulty-levels-design.md`

## Global Constraints

- Winkelkonvention: `phi = 0` ist die stabile, hängende Ruhelage, `phi = math.pi` die instabile, aufrechte Zielposition. Nicht ändern.
- FMU-Parameter werden ausschließlich über `fmu.setReal()` auf `causality=parameter`-Variablen gesetzt, und zwar **vor** `fmu.enterInitializationMode()` — nie über die Output-Variablen `phi`/`vphi`/`s`/`v`.
- `SUBSTEPS = 10`-Sub-Stepping in der Game-Loop bleibt unverändert; keine Änderung an der Doppelschleifen-Struktur oder an der Aktuierungs-/Scoring-Kadenz.
- `Controller`-Basisklasse und die bestehenden Controller (`SimpleController`, `LQRController`, `SwingUpController`) werden nicht verändert.
- Die `Standard`-Stufe muss exakt den bisherigen AP3-Werten entsprechen: `m_cart=5.0`, `m_pend=0.5`, `d_cart=0.15`, `d_pend=0.01`, `bonus_zone=15°`, `tight_bonus_zone=5°`.
- FMU-geführte Tests werden mit `pytest.mark.skipif(not os.path.exists(FMU_PATH), reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)")` abgesichert, exakt wie in `tests/test_numerical_stability.py`.
- Änderungen an `redraw()` und der Event-Loop (Tasten-Handling) haben keine automatisierten Tests — das ist erwartetes, dokumentiertes Verhalten dieses Projekts, kein Lücke, die geschlossen werden muss. Menschliche interaktive Verifikation ist nach Abschluss aller Tasks erforderlich und muss im Abschlussbericht explizit als offen benannt werden.
- Zwei frühere Badges in diesem Projekt (AP3 Teil 1 und Teil 2) hatten Overflow-Regressionen bei zu langen Labels. Jedes neue Badge-/Hint-Label muss kurz genug sein, um in die bestehende Badge-Breite (`0.28 * scale`) bzw. Hint-Breite (`0.40 * scale`) zu passen — beim Review explizit gegen die längsten bestehenden Labels (`"AUTO [LQR]  [H to disable]"`, ca. 27 Zeichen) abschätzen, nicht nur "sieht kurz genug aus".

---

### Task 1: Difficulty-Konfiguration und Zyklus-Funktion

**Files:**
- Modify: `pendulum_game_controlled.py` (neue Konstanten/Funktion einfügen direkt vor `K_STABILITY = 0.5`, aktuell Zeile 422)
- Test: `tests/test_difficulty.py` (neu)

**Interfaces:**
- Produces: `DIFFICULTY_ORDER: tuple[str, str, str]` (Werte `("Leicht", "Standard", "Schwer")`), `DIFFICULTY_LEVELS: dict[str, dict]` (Schlüssel je Stufe: `bonus_zone_deg`, `tight_bonus_zone_deg`, `m_cart`, `m_pend`, `d_cart`, `d_pend`), `next_difficulty(current: str) -> str`.

- [ ] **Step 1: Failing Tests schreiben**

Erstelle `tests/test_difficulty.py`:

```python
from pendulum_game_controlled import DIFFICULTY_ORDER, DIFFICULTY_LEVELS, next_difficulty


def test_difficulty_order_has_three_levels_in_expected_sequence():
    assert DIFFICULTY_ORDER == ("Leicht", "Standard", "Schwer")


def test_next_difficulty_cycles_forward():
    assert next_difficulty("Leicht") == "Standard"
    assert next_difficulty("Standard") == "Schwer"
    assert next_difficulty("Schwer") == "Leicht"


def test_standard_matches_ap3_original_constants():
    level = DIFFICULTY_LEVELS["Standard"]
    assert level["m_cart"] == 5.0
    assert level["m_pend"] == 0.5
    assert level["d_cart"] == 0.15
    assert level["d_pend"] == 0.01
    assert level["bonus_zone_deg"] == 15.0
    assert level["tight_bonus_zone_deg"] == 5.0


def test_all_levels_present_with_required_keys():
    required_keys = {
        "bonus_zone_deg",
        "tight_bonus_zone_deg",
        "m_cart",
        "m_pend",
        "d_cart",
        "d_pend",
    }
    for name in DIFFICULTY_ORDER:
        assert set(DIFFICULTY_LEVELS[name].keys()) == required_keys
```

- [ ] **Step 2: Tests laufen lassen, Fehlschlag bestätigen**

Run: `pytest tests/test_difficulty.py -v`
Expected: FAIL mit `ImportError` / `cannot import name 'DIFFICULTY_ORDER'`

- [ ] **Step 3: Konfiguration implementieren**

In `pendulum_game_controlled.py`, direkt **vor** der Zeile `K_STABILITY = 0.5` (aktuell Zeile 422) einfügen:

```python
DIFFICULTY_ORDER = ("Leicht", "Standard", "Schwer")

DIFFICULTY_LEVELS = {
    "Leicht": {
        "bonus_zone_deg": 20.0,
        "tight_bonus_zone_deg": 8.0,
        "m_cart": 5.0,
        "m_pend": 0.3,
        "d_cart": 0.30,
        "d_pend": 0.05,
    },
    "Standard": {
        "bonus_zone_deg": 15.0,
        "tight_bonus_zone_deg": 5.0,
        "m_cart": 5.0,
        "m_pend": 0.5,
        "d_cart": 0.15,
        "d_pend": 0.01,
    },
    "Schwer": {
        "bonus_zone_deg": 8.0,
        "tight_bonus_zone_deg": 3.0,
        "m_cart": 5.0,
        "m_pend": 0.9,
        "d_cart": 0.05,
        "d_pend": 0.01,
    },
}


def next_difficulty(current):
    idx = DIFFICULTY_ORDER.index(current)
    return DIFFICULTY_ORDER[(idx + 1) % len(DIFFICULTY_ORDER)]
```

- [ ] **Step 4: Tests laufen lassen, Erfolg bestätigen**

Run: `pytest tests/test_difficulty.py -v`
Expected: PASS (4 passed)

- [ ] **Step 5: Commit**

```bash
git add pendulum_game_controlled.py tests/test_difficulty.py
git commit -m "feat: add AP4 difficulty level configuration"
```

---

### Task 2: Toleranzbereich in `compute_score_increment` parametrisieren

**Files:**
- Modify: `pendulum_game_controlled.py:425-444` (`compute_score_increment`)
- Test: `tests/test_scoring.py` (bestehende Datei erweitern)

**Interfaces:**
- Consumes: nichts aus Task 1 (dieser Task ändert nur die Funktionssignatur, verdrahtet sie noch nicht mit `DIFFICULTY_LEVELS` — das passiert erst in Task 5, wo die Laufzeit-Difficulty existiert).
- Produces: `compute_score_increment(angle, stable_streak, bonus_zone=math.radians(15), tight_bonus_zone=math.radians(5)) -> float`. Die Defaults reproduzieren exakt das bisherige Verhalten, sodass alle bestehenden Aufrufer und Tests unverändert weiterlaufen.

- [ ] **Step 1: Failing Tests schreiben**

Füge in `tests/test_scoring.py` am Ende der Datei hinzu (Datei beginnt bereits mit `import math` und dem Import aus `pendulum_game_controlled` — nicht doppelt importieren):

```python
def test_custom_bonus_zone_widens_bonus_range():
    angle = math.radians(18)  # außerhalb der Standard-15°-Zone, innerhalb einer 20°-Zone
    default_increment = compute_score_increment(angle, stable_streak=0.0)
    widened_increment = compute_score_increment(
        angle, stable_streak=0.0, bonus_zone=math.radians(20), tight_bonus_zone=math.radians(5)
    )
    assert widened_increment > default_increment


def test_custom_tight_bonus_zone_narrows_bonus_range():
    angle = math.radians(4)  # innerhalb der Standard-5°-Zone, außerhalb einer 3°-Zone
    default_increment = compute_score_increment(angle, stable_streak=0.0)
    narrowed_increment = compute_score_increment(
        angle, stable_streak=0.0, bonus_zone=math.radians(15), tight_bonus_zone=math.radians(3)
    )
    assert narrowed_increment < default_increment
```

- [ ] **Step 2: Tests laufen lassen, Fehlschlag bestätigen**

Run: `pytest tests/test_scoring.py -v`
Expected: FAIL mit `TypeError: compute_score_increment() got an unexpected keyword argument 'bonus_zone'`

- [ ] **Step 3: Funktion parametrisieren**

Ersetze in `pendulum_game_controlled.py` die bestehende Funktion (aktuell Zeilen 425-444):

```python
def compute_score_increment(angle, stable_streak):
    max_angle = math.pi / 2
    bonus_zone = math.radians(15)
    tight_bonus_zone = math.radians(5)

    increment = 0.0
    if angle <= max_angle:
        closeness = (max_angle - angle) / max_angle
        increment += 0.1 + 0.2 * closeness

        if angle <= bonus_zone:
            close2 = (bonus_zone - angle) / bonus_zone
            increment += 2 * (close2**2)

        if angle <= tight_bonus_zone:
            close3 = (tight_bonus_zone - angle) / tight_bonus_zone
            increment += 3 * (close3**2)

    increment += K_STABILITY * stable_streak
    return increment
```

durch:

```python
def compute_score_increment(angle, stable_streak, bonus_zone=math.radians(15), tight_bonus_zone=math.radians(5)):
    max_angle = math.pi / 2

    increment = 0.0
    if angle <= max_angle:
        closeness = (max_angle - angle) / max_angle
        increment += 0.1 + 0.2 * closeness

        if angle <= bonus_zone:
            close2 = (bonus_zone - angle) / bonus_zone
            increment += 2 * (close2**2)

        if angle <= tight_bonus_zone:
            close3 = (tight_bonus_zone - angle) / tight_bonus_zone
            increment += 3 * (close3**2)

    increment += K_STABILITY * stable_streak
    return increment
```

- [ ] **Step 4: Tests laufen lassen, Erfolg bestätigen**

Run: `pytest tests/test_scoring.py -v`
Expected: PASS (alle Tests, alte und neue, insgesamt 5 passed)

- [ ] **Step 5: Commit**

```bash
git add pendulum_game_controlled.py tests/test_scoring.py
git commit -m "feat: parameterize scoring tolerance zones for AP4 difficulty"
```

---

### Task 3: FMU-Physik-Helfer (`apply_difficulty_physics`, `reset_round`)

**Files:**
- Modify: `pendulum_game_controlled.py` (neue Funktionen einfügen direkt nach `controller_display_name`, aktuell Zeilen 455-459, vor `def run_game(screen):`)
- Test: `tests/test_difficulty_fmu.py` (neu, FMU-geführt)

**Interfaces:**
- Consumes: `DIFFICULTY_LEVELS` aus Task 1 (exakte Schlüssel: `m_cart`, `m_pend`, `d_cart`, `d_pend`).
- Produces: `apply_difficulty_physics(fmu, value_refs, difficulty)` (setzt die vier FMU-Parameter via `fmu.setReal()`, `value_refs` ist ein `dict` mit den Schlüsseln `"m_cart"`, `"m_pend"`, `"d_cart"`, `"d_pend"` → `valueReference`); `reset_round(fmu, value_refs, difficulty, s_ref, v_ref, phi_ref, vphi_ref) -> (s, v, phi, vphi)` (führt `fmu.reset()` + `setupExperiment()` + `apply_difficulty_physics()` + `enterInitializationMode()`/`exitInitializationMode()` aus und liefert den frischen Zustand zurück — Task 5 nutzt dies für die `R`- und `D`-Tastenbehandlung).

- [ ] **Step 1: Failing Tests schreiben**

Erstelle `tests/test_difficulty_fmu.py`:

```python
import math
import os

import pytest

from pendulum_game_controlled import apply_difficulty_physics, reset_round, DIFFICULTY_LEVELS

FMU_PATH = os.path.abspath("InvertedPendulumMB.fmu")


def _instantiate_fmu():
    from fmpy import read_model_description, extract
    from fmpy.fmi2 import FMU2Slave

    unzipdir = extract(FMU_PATH)
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

    return fmu, ref


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
@pytest.mark.parametrize("difficulty", ["Leicht", "Standard", "Schwer"])
def test_apply_difficulty_physics_sets_expected_values(difficulty):
    fmu, ref = _instantiate_fmu()
    value_refs = {
        "m_cart": ref("m_cart"),
        "m_pend": ref("m_pend"),
        "d_cart": ref("d_cart"),
        "d_pend": ref("d_pend"),
    }

    apply_difficulty_physics(fmu, value_refs, difficulty)
    fmu.enterInitializationMode()
    fmu.exitInitializationMode()

    level = DIFFICULTY_LEVELS[difficulty]
    assert math.isclose(fmu.getReal([value_refs["m_cart"]])[0], level["m_cart"])
    assert math.isclose(fmu.getReal([value_refs["m_pend"]])[0], level["m_pend"])
    assert math.isclose(fmu.getReal([value_refs["d_cart"]])[0], level["d_cart"])
    assert math.isclose(fmu.getReal([value_refs["d_pend"]])[0], level["d_pend"])

    fmu.terminate()
    fmu.freeInstance()


@pytest.mark.skipif(
    not os.path.exists(FMU_PATH),
    reason="InvertedPendulumMB.fmu not present (gitignored build artifact, copy manually into worktrees that run the game)",
)
def test_reset_round_applies_difficulty_and_returns_fresh_state():
    fmu, ref = _instantiate_fmu()
    value_refs = {
        "m_cart": ref("m_cart"),
        "m_pend": ref("m_pend"),
        "d_cart": ref("d_cart"),
        "d_pend": ref("d_pend"),
    }
    s_ref, v_ref, phi_ref, vphi_ref = ref("s"), ref("v"), ref("phi"), ref("vphi")

    # Einmal durch die Initialisierung laufen, wie beim echten Spielstart.
    fmu.enterInitializationMode()
    fmu.exitInitializationMode()

    s, v, phi, vphi = reset_round(fmu, value_refs, "Schwer", s_ref, v_ref, phi_ref, vphi_ref)

    level = DIFFICULTY_LEVELS["Schwer"]
    assert math.isclose(fmu.getReal([value_refs["m_pend"]])[0], level["m_pend"])
    assert math.isclose(fmu.getReal([value_refs["d_cart"]])[0], level["d_cart"])
    # Ein frischer Reset startet beim Modell-Default-Anfangswinkel (~67.5°
    # aus der hängenden Ruhelage), unabhängig von den Difficulty-Physik-Werten.
    assert math.isclose(math.degrees(phi), 67.5, rel_tol=1e-3)
    assert math.isclose(vphi, 0.0, abs_tol=1e-6)

    fmu.terminate()
    fmu.freeInstance()
```

- [ ] **Step 2: Tests laufen lassen, Fehlschlag bestätigen**

Run: `pytest tests/test_difficulty_fmu.py -v`
Expected: Falls die FMU lokal vorhanden ist: FAIL mit `ImportError: cannot import name 'apply_difficulty_physics'`. Falls die FMU fehlt: SKIPPED (das ist ebenfalls ein akzeptables Zwischenergebnis für diesen Schritt — dann Step 3 trotzdem durchführen und in Step 4 auf eine vorhandene FMU-Kopie prüfen).

- [ ] **Step 3: Funktionen implementieren**

In `pendulum_game_controlled.py`, direkt **nach** der Funktion `controller_display_name` (aktuell Zeilen 455-459) und **vor** `def run_game(screen):` einfügen:

```python
def apply_difficulty_physics(fmu, value_refs, difficulty):
    level = DIFFICULTY_LEVELS[difficulty]
    fmu.setReal(
        [value_refs["m_cart"], value_refs["m_pend"], value_refs["d_cart"], value_refs["d_pend"]],
        [level["m_cart"], level["m_pend"], level["d_cart"], level["d_pend"]],
    )


def reset_round(fmu, value_refs, difficulty, s_ref, v_ref, phi_ref, vphi_ref):
    fmu.reset()
    fmu.setupExperiment(startTime=0.0)
    apply_difficulty_physics(fmu, value_refs, difficulty)
    fmu.enterInitializationMode()
    fmu.exitInitializationMode()
    return (
        fmu.getReal([s_ref])[0],
        fmu.getReal([v_ref])[0],
        fmu.getReal([phi_ref])[0],
        fmu.getReal([vphi_ref])[0],
    )
```

- [ ] **Step 4: Tests laufen lassen, Erfolg bestätigen**

Run: `pytest tests/test_difficulty_fmu.py -v`
Expected: PASS (4 passed) falls `InvertedPendulumMB.fmu` im Projektverzeichnis vorhanden ist, sonst SKIPPED (4 skipped) — beides ist ein gültiger Erfolg für diesen Schritt.

- [ ] **Step 5: Commit**

```bash
git add pendulum_game_controlled.py tests/test_difficulty_fmu.py
git commit -m "feat: add FMU physics helpers for AP4 difficulty levels"
```

---

### Task 4: HUD — Difficulty-Badge und Hinweis-Anzeige in `redraw()`

**Files:**
- Modify: `pendulum_game_controlled.py:16` (Signatur von `redraw()`) und `pendulum_game_controlled.py:166-192` (Badge-Rendering)

**Interfaces:**
- Consumes: nichts Neues aus vorherigen Tasks — `redraw()` bekommt lediglich zwei neue optionale Parameter.
- Produces: `redraw(..., difficulty="Standard", hint=None)` — mit Defaults, die bestehende Aufrufer (noch) unverändert lässt, bis Task 5 die drei Aufrufstellen in `run_game()` auf die echten Laufzeitwerte umstellt.

- [ ] **Step 1: Signatur erweitern**

Ersetze in `pendulum_game_controlled.py` Zeile 16:

```python
def redraw(screen, time, dt, score, precision, s, v, taus, phis, auto_mode=False, paused=False, controller_name="PD"):
```

durch:

```python
def redraw(screen, time, dt, score, precision, s, v, taus, phis, auto_mode=False, paused=False, controller_name="PD", difficulty="Standard", hint=None):
```

- [ ] **Step 2: Difficulty-Badge und Hinweis-Anzeige einfügen**

Ersetze den bestehenden Block (aktuell Zeilen 166-192):

```python
    badge_color = gruen if auto_mode else rot
    badge_label = (
        f"AUTO [{controller_name}]  [H to disable]" if auto_mode else "MANUAL  [H for auto]"
    )
    badge_rect = pygame.Rect(
        width - math.ceil(scale * 0.30),
        math.ceil(scale * 0.02),
        math.ceil(scale * 0.28),
        math.ceil(scale * 0.06),
    )
    pygame.draw.rect(screen, badge_color, badge_rect, border_radius=8)
    pygame.draw.rect(screen, dunkel, badge_rect, 2, border_radius=8)
    display(
        badge_label,
        badge_rect.center,
        color=(255, 255, 255),
        size=math.ceil(scale / 42),
    )

    if paused:
        pause_rect = pygame.Rect(
            width - math.ceil(scale * 0.30),
            math.ceil(scale * 0.02) + badge_rect.height + math.ceil(scale * 0.01),
            math.ceil(scale * 0.28),
            math.ceil(scale * 0.06),
        )
        pygame.draw.rect(screen, dunkel, pause_rect, border_radius=8)
        display(
            "PAUSED  [P to resume]",
            pause_rect.center,
            color=(255, 255, 255),
            size=math.ceil(scale / 42),
        )
```

durch:

```python
    badge_color = gruen if auto_mode else rot
    badge_label = (
        f"AUTO [{controller_name}]  [H to disable]" if auto_mode else "MANUAL  [H for auto]"
    )
    badge_rect = pygame.Rect(
        width - math.ceil(scale * 0.30),
        math.ceil(scale * 0.02),
        math.ceil(scale * 0.28),
        math.ceil(scale * 0.06),
    )
    pygame.draw.rect(screen, badge_color, badge_rect, border_radius=8)
    pygame.draw.rect(screen, dunkel, badge_rect, 2, border_radius=8)
    display(
        badge_label,
        badge_rect.center,
        color=(255, 255, 255),
        size=math.ceil(scale / 42),
    )

    difficulty_rect = pygame.Rect(
        width - math.ceil(scale * 0.30),
        math.ceil(scale * 0.02) + badge_rect.height + math.ceil(scale * 0.01),
        math.ceil(scale * 0.28),
        math.ceil(scale * 0.06),
    )
    pygame.draw.rect(screen, blau, difficulty_rect, border_radius=8)
    pygame.draw.rect(screen, dunkel, difficulty_rect, 2, border_radius=8)
    display(
        f"Difficulty: {difficulty}  [D]",
        difficulty_rect.center,
        color=(255, 255, 255),
        size=math.ceil(scale / 42),
    )

    if paused:
        pause_rect = pygame.Rect(
            width - math.ceil(scale * 0.30),
            math.ceil(scale * 0.02) + badge_rect.height + difficulty_rect.height + 2 * math.ceil(scale * 0.01),
            math.ceil(scale * 0.28),
            math.ceil(scale * 0.06),
        )
        pygame.draw.rect(screen, dunkel, pause_rect, border_radius=8)
        display(
            "PAUSED  [P to resume]",
            pause_rect.center,
            color=(255, 255, 255),
            size=math.ceil(scale / 42),
        )

    if hint:
        hint_rect = pygame.Rect(
            width // 2 - math.ceil(scale * 0.20),
            math.ceil(scale * 0.02),
            math.ceil(scale * 0.40),
            math.ceil(scale * 0.05),
        )
        pygame.draw.rect(screen, dunkel, hint_rect, border_radius=8)
        display(
            hint,
            hint_rect.center,
            color=(255, 255, 255),
            size=math.ceil(scale / 46),
        )
```

Die längste mögliche Difficulty-Badge-Beschriftung ist `"Difficulty: Standard  [D]"` (26 Zeichen) — kürzer als das längste bestehende Auto-Badge-Label `"AUTO [SwingUp]  [H to disable]"` (31 Zeichen, wird aber laut `controller_display_name` für SwingUp ohnehin auf `"SU:s"`/`"SU:b"` abgekürzt, sodass `"AUTO [LQR]  [H to disable]"`, 27 Zeichen, das faktisch längste gerenderte Auto-Label ist). Beide Hinweis-Texte (`"Auto nur bei Standard"`, 22 Zeichen; `"Nur im Manual-Modus änderbar"`, 29 Zeichen) sind bewusst kurz gehalten. Diese Einschätzungen ersetzen keinen visuellen Test — der Reviewer muss die Zeichenzahlen gegen die Rect-Breiten (`0.28 * scale` bzw. `0.40 * scale`) explizit gegenrechnen, nicht nur "sieht kurz genug aus" urteilen (siehe Global Constraints zu den zwei früheren Badge-Overflow-Regressionen).

- [ ] **Step 3: Bestehende Tests laufen lassen (Regressionscheck)**

Run: `pytest -v`
Expected: Alle bisherigen Tests weiterhin PASS — `redraw()` selbst hat keine automatisierten Tests (Pygame-`Surface`-Rendering, siehe Global Constraints), aber kein anderer Test darf durch die Signaturänderung brechen.

- [ ] **Step 4: Commit**

```bash
git add pendulum_game_controlled.py
git commit -m "feat: add difficulty badge and hint display to redraw()"
```

---

### Task 5: Game-Loop-Integration — Tasten, Reset, Scoring, Leaderboard

**Files:**
- Modify: `pendulum_game_controlled.py` (komplette Funktionen `run_game()`, aktuell Zeilen 462-607, und `main()`, aktuell Zeilen 610-626, ersetzen)

**Interfaces:**
- Consumes: `DIFFICULTY_ORDER`, `DIFFICULTY_LEVELS`, `next_difficulty()` (Task 1); `compute_score_increment(angle, stable_streak, bonus_zone=..., tight_bonus_zone=...)` (Task 2); `apply_difficulty_physics()`, `reset_round()` (Task 3); `redraw(..., difficulty=..., hint=...)` (Task 4).
- Produces: `run_game(screen) -> (score: float, mode: str, difficulty: str)` — Rückgabetupel um `difficulty` erweitert (bisher `(score, mode)`); `update_leaderboard()` wird mit dieser Difficulty statt dem Default aufgerufen.

- [ ] **Step 1: `run_game()` ersetzen**

Ersetze die komplette Funktion `run_game(screen)` (aktuell Zeilen 462-607) durch:

```python
def run_game(screen):
    fmu_path = os.path.abspath("InvertedPendulumMB.fmu")
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

    difficulty = "Standard"
    value_refs = {
        "m_cart": ref("m_cart"),
        "m_pend": ref("m_pend"),
        "d_cart": ref("d_cart"),
        "d_pend": ref("d_pend"),
    }
    apply_difficulty_physics(fmu, value_refs, difficulty)

    fmu.enterInitializationMode()
    fmu.exitInitializationMode()

    tau_ref = ref("tau")
    s_ref = ref("s")
    v_ref = ref("v")
    phi_ref = ref("phi")
    vphi_ref = ref("vphi")

    dt = 0.02
    # Explicit-Euler FMU co-simulation at the full 0.02s step can numerically
    # inject energy into a lightly-damped oscillator (confirmed: at the low
    # d_pend used since AP3 Teil 2's swing-up fix, single 0.02s steps make
    # free-swinging pendulum motion diverge instead of decay). Sub-stepping
    # the FMU call keeps physics accurate without changing the 50Hz game
    # timing, scoring cadence, or actuation resolution (tau is still set
    # once per outer frame, before the substep loop).
    SUBSTEPS = 10
    GAME_DURATION = 40
    MAX_TAU = 10.0
    time = 0.0
    score = 0.0
    auto_mode = False
    paused = False
    stable_streak = 0.0
    hint_text = None
    hint_frames_left = 0

    s = 0.0
    v = 0.0
    phi = math.pi + 0.75 * math.pi / 2
    vphi = 0.0

    auto_time = 0.0
    manual_time = 0.0

    taus, phis = [], []
    controllers = {"PD": SimpleController(), "LQR": LQRController(), "SwingUp": SwingUpController()}
    controller_name = "PD"
    clock = pygame.time.Clock()

    redraw(
        screen, time, dt, 0, 0.25, s, v, [phi], [vphi], auto_mode, paused,
        controller_display_name(controllers[controller_name], controller_name),
        difficulty=difficulty, hint=hint_text,
    )
    pygame.display.flip()
    overlay_leaderboard(screen)

    while time < GAME_DURATION:
        keys = pygame.key.get_pressed()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                exit()
            if event.type == pygame.KEYDOWN and event.key == pygame.K_h:
                if difficulty == "Standard":
                    auto_mode = not auto_mode
                else:
                    hint_text = "Auto nur bei Standard"
                    hint_frames_left = 90
            if event.type == pygame.KEYDOWN and event.key == pygame.K_p:
                paused = not paused
            if event.type == pygame.KEYDOWN and event.key == pygame.K_l:
                names = list(controllers)
                controller_name = names[(names.index(controller_name) + 1) % len(names)]
            if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                s, v, phi, vphi = reset_round(fmu, value_refs, difficulty, s_ref, v_ref, phi_ref, vphi_ref)
                time = 0.0
                score = 0.0
                stable_streak = 0.0
                auto_time = 0.0
                manual_time = 0.0
                taus, phis = [], []
            if event.type == pygame.KEYDOWN and event.key == pygame.K_d:
                if auto_mode:
                    hint_text = "Nur im Manual-Modus änderbar"
                    hint_frames_left = 90
                else:
                    difficulty = next_difficulty(difficulty)
                    s, v, phi, vphi = reset_round(fmu, value_refs, difficulty, s_ref, v_ref, phi_ref, vphi_ref)
                    time = 0.0
                    score = 0.0
                    stable_streak = 0.0
                    auto_time = 0.0
                    manual_time = 0.0
                    taus, phis = [], []

        if hint_frames_left > 0:
            hint_frames_left -= 1
            if hint_frames_left == 0:
                hint_text = None

        if paused:
            plot_taus = taus if taus else [0.0]
            plot_phis = phis if phis else [phi - math.pi]
            redraw(
                screen, time, dt, score, 0.25, s, v, plot_taus, plot_phis, auto_mode, paused,
                controller_display_name(controllers[controller_name], controller_name),
                difficulty=difficulty, hint=hint_text,
            )
            pygame.display.flip()
            clock.tick(60)
            continue

        if auto_mode:
            tau = controllers[controller_name].compute(phi, vphi, s, v)
            auto_time += dt
        else:
            tau = 0.0
            if keys[pygame.K_LEFT]:
                tau = -MAX_TAU
            if keys[pygame.K_RIGHT]:
                tau = MAX_TAU
            manual_time += dt

        fmu.setReal([tau_ref], [tau])
        substep_dt = dt / SUBSTEPS
        for _ in range(SUBSTEPS):
            time += substep_dt
            fmu.doStep(currentCommunicationPoint=time, communicationStepSize=substep_dt)

        s = fmu.getReal([s_ref])[0]
        v = fmu.getReal([v_ref])[0]
        phi = fmu.getReal([phi_ref])[0]
        vphi = fmu.getReal([vphi_ref])[0]

        angle = (phi - math.pi) % (2 * math.pi)
        if angle > math.pi:
            angle -= 2 * math.pi
        angle = abs(angle)

        level = DIFFICULTY_LEVELS[difficulty]
        bonus_zone = math.radians(level["bonus_zone_deg"])
        tight_bonus_zone = math.radians(level["tight_bonus_zone_deg"])

        if angle <= tight_bonus_zone:
            stable_streak += dt
        else:
            stable_streak = 0.0

        score += compute_score_increment(angle, stable_streak, bonus_zone=bonus_zone, tight_bonus_zone=tight_bonus_zone)

        phis.append(phi - math.pi)
        taus.append(tau)
        if len(phis) > 750:
            phis.pop(0)
            taus.pop(0)

        redraw(
            screen, time, dt, score, 0.25, s, v, taus, phis, auto_mode, paused,
            controller_display_name(controllers[controller_name], controller_name),
            difficulty=difficulty, hint=hint_text,
        )
        pygame.display.flip()
        clock.tick(60)

    fmu.terminate()
    fmu.freeInstance()
    shutil.rmtree(unzipdir)
    return score, classify_mode(auto_time, manual_time), difficulty
```

- [ ] **Step 2: `main()` ersetzen**

Ersetze die komplette Funktion `main()` (aktuell Zeilen 610-623):

```python
def main():
    pygame.init()
    screen = pygame.display.set_mode((1920, 1080), pygame.RESIZABLE)
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    pygame.mouse.set_visible(False)
    pygame.event.set_grab(True)
    pygame.key.set_mods(0)
    pygame.mouse.set_visible(True)

    while True:
        score, mode = run_game(screen)
        player_name = get_player_name(screen)
        update_leaderboard(score, player_name, mode)
```

durch:

```python
def main():
    pygame.init()
    screen = pygame.display.set_mode((1920, 1080), pygame.RESIZABLE)
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    pygame.mouse.set_visible(False)
    pygame.event.set_grab(True)
    pygame.key.set_mods(0)
    pygame.mouse.set_visible(True)

    while True:
        score, mode, difficulty = run_game(screen)
        player_name = get_player_name(screen)
        update_leaderboard(score, player_name, mode, difficulty)
```

- [ ] **Step 3: Bestehende Tests laufen lassen (Regressionscheck)**

Run: `pytest -v`
Expected: Alle Tests aus Task 1-4 sowie alle vorherigen Test-Dateien (`test_scoring.py`, `test_leaderboard.py`, `test_mode.py`, `test_lqr_controller.py`, `test_lqr_gain.py`, `test_swingup_controller.py`, `test_numerical_stability.py`, `test_benchmark_*.py`) weiterhin PASS. `run_game()` und `main()` selbst haben keine automatisierten Tests (Pygame-Event-Loop).

- [ ] **Step 4: Manuelle interaktive Verifikation (menschlich, nicht automatisierbar)**

Dieser Schritt kann nicht von einem Subagenten durchgeführt werden — er erfordert einen echten Bildschirm. Im Abschlussbericht explizit als offen benennen, falls nicht durchgeführt. Prüfpunkte:
- `D` schaltet sichtbar durch Leicht → Standard → Schwer → Leicht, jedes Mal mit spürbarem Reset (Score/Zeit auf 0, Pendel zurück in Ausgangslage).
- Bei Leicht fühlt sich das manuelle Halten spürbar leichter an als bei Schwer (Ziel aus der Projektanweisung: "spürbar unterschiedliche Anforderungen"). Falls nicht: `DIFFICULTY_LEVELS`-Werte in Task 1 nachjustieren.
- `H` ist bei Leicht/Schwer ein No-Op mit sichtbarem Hinweis "Auto nur bei Standard"; funktioniert wieder normal bei Standard.
- `D` ist im Auto-Modus ein No-Op mit sichtbarem Hinweis "Nur im Manual-Modus änderbar".
- `R` behält die aktuell gewählte Stufe bei (kein Sprung zurück zu Standard).
- Difficulty-Badge und Hinweis-Text sind vollständig lesbar, kein Überlauf über den Rand des Badges/Hint-Rects hinaus, bei keiner der drei Stufen.
- Am Rundenende landet die tatsächlich gespielte Stufe (nicht immer "Standard") korrekt in `leaderboard.csv`.

- [ ] **Step 5: Commit**

```bash
git add pendulum_game_controlled.py
git commit -m "feat: wire AP4 difficulty selection into the game loop and leaderboard"
```
