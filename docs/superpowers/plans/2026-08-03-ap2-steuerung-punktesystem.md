# AP2 Steuerung und Punktesystem Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Extend `pendulum_game_controlled.py` per the approved AP2 design (switch to the MultiBody FMU, add pause/reset controls, add a continuous stability bonus to scoring, extend the leaderboard with Mode/Difficulty columns).

**Architecture:** All changes live in the single existing `pendulum_game_controlled.py` script, following its current flat-function style (no new modules). Two new pure functions (`compute_score_increment`, `classify_mode`) are extracted so they can be unit-tested with `pytest` without a display or an FMU. Everything else (FMU path, pause/reset key handling, leaderboard schema) is verified manually by running the game, since it requires the Pygame window and a loaded FMU.

**Tech Stack:** Python 3.10, `pygame`, `fmpy` (`FMU2Slave`), `pandas`, `pytest` (new dev dependency for this feature).

## Global Constraints

- Winkelkonvention bleibt bestehen: `phi = 0` ist die stabile, hängende Ruhelage, `phi = math.pi` die instabile, aufrechte Zielposition (siehe `protokoll.md`). Keine der neuen Funktionen darf diese Konvention ändern.
- Reset nutzt die FMI2-Standardfunktion `fmu.reset()`, kein Teardown/Neuaufbau der FMU-Instanz.
- Die MultiBody-FMU (`InvertedPendulumMB.fmu`) ist mit dem Euler-Solver exportiert (CVODE ist unter Windows instabil, siehe `AP1_Validierung.md` Abschnitt 3.3) — daran wird nichts geändert.
- `SimpleController` und die `Controller`-Basisklasse bleiben unverändert (AP3 ist ein separates Arbeitspaket).
- Keine Schwierigkeitsgrad-Logik (Pendellänge, Reibung, Störgrößen) — nur die `Difficulty`-Spalte im Leaderboard wird mit Platzhalterwert `"Standard"` vorbereitet (AP4 ist ein separates Arbeitspaket).
- Keine feinere Kraftdosierung — Steuerung bleibt bei `tau = ±MAX_TAU` über die Pfeiltasten.
- `leaderboard.csv` enthält aktuell nur die Kopfzeile (keine Datenzeilen), daher ist die Schema-Erweiterung migrationsfrei.

---

## Task 1: Stabilitäts-Bonus als testbare Funktion (`compute_score_increment`)

**Files:**
- Create: `tests/test_scoring.py`
- Modify: `pendulum_game_controlled.py` (neue Funktion `compute_score_increment` + Konstante `K_STABILITY`, eingefügt zwischen der `# TODO: some other controllers`-Zeile und `def run_game(screen):`; danach Integration in die Score-Logik innerhalb von `run_game()`)

**Interfaces:**
- Produces: `K_STABILITY: float` (Modulkonstante, Wert `0.5`), `compute_score_increment(angle: float, stable_streak: float) -> float` — reine Funktion, keine Seiteneffekte. `angle` ist der bereits auf `[0, pi]` normalisierte Betrag der Winkelabweichung von der aufrechten Lage (wie die bestehende `angle`-Variable in `run_game()`), `stable_streak` ist die Anzahl Sekunden ununterbrochener Zeit unterhalb der 5°-Zone.

- [ ] **Step 1: pytest installieren**

Run: `python -m pip install pytest`
Expected: Installation erfolgreich (`pytest` erscheint danach bei `python -m pip show pytest`).

- [ ] **Step 2: Fehlschlagenden Test schreiben**

Create `tests/test_scoring.py`:

```python
import math

from pendulum_game_controlled import K_STABILITY, compute_score_increment


def test_zero_beyond_max_angle():
    assert compute_score_increment(angle=math.pi / 2 + 0.01, stable_streak=0.0) == 0.0


def test_matches_manual_formula_in_tight_zone():
    angle = math.radians(3)
    stable_streak = 2.0

    max_angle = math.pi / 2
    bonus_zone = math.radians(15)
    tight_bonus_zone = math.radians(5)
    closeness = (max_angle - angle) / max_angle
    close2 = (bonus_zone - angle) / bonus_zone
    close3 = (tight_bonus_zone - angle) / tight_bonus_zone

    expected = (
        (0.1 + 0.2 * closeness)
        + 2 * (close2**2)
        + 3 * (close3**2)
        + K_STABILITY * stable_streak
    )

    assert math.isclose(compute_score_increment(angle, stable_streak), expected)


def test_stability_bonus_isolated():
    angle = math.radians(3)
    with_streak = compute_score_increment(angle, stable_streak=2.0)
    without_streak = compute_score_increment(angle, stable_streak=0.0)

    assert math.isclose(with_streak - without_streak, K_STABILITY * 2.0)
```

- [ ] **Step 3: Test laufen lassen, Fehlschlag bestätigen**

Run: `python -m pytest tests/test_scoring.py -v`
Expected: FAIL mit `ImportError: cannot import name 'K_STABILITY' from 'pendulum_game_controlled'` (Funktion/Konstante existieren noch nicht).

- [ ] **Step 4: Funktion implementieren**

In `pendulum_game_controlled.py`, zwischen der Zeile `# TODO: some other controllers` und `def run_game(screen):`, einfügen:

```python
K_STABILITY = 0.5


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

- [ ] **Step 5: Test laufen lassen, Erfolg bestätigen**

Run: `python -m pytest tests/test_scoring.py -v`
Expected: 3 passed.

- [ ] **Step 6: In `run_game()` integrieren**

In `run_game()`, den State-Init-Block (aktuell `time = 0.0` / `score = 0.0` / `auto_mode = False`) um eine neue Zeile ergänzen:

```python
    dt = 0.02
    GAME_DURATION = 40
    MAX_TAU = 10.0
    time = 0.0
    score = 0.0
    auto_mode = False
    stable_streak = 0.0
```

Den bestehenden Scoring-Block in der `while`-Schleife

```python
        max_angle = math.pi / 2
        bonus_zone = math.radians(15)
        tight_bonus_zone = math.radians(5)

        if angle <= max_angle:
            closeness = (max_angle - angle) / max_angle
            score += 0.1 + 0.2 * closeness

            if angle <= bonus_zone:
                close2 = (bonus_zone - angle) / bonus_zone
                score += 2 * (close2**2)

            if angle <= tight_bonus_zone:
                close3 = (tight_bonus_zone - angle) / tight_bonus_zone
                score += 3 * (close3**2)
```

ersetzen durch:

```python
        if angle <= math.radians(5):
            stable_streak += dt
        else:
            stable_streak = 0.0

        score += compute_score_increment(angle, stable_streak)
```

- [ ] **Step 7: Manuell verifizieren**

Run: `python pendulum_game_controlled.py`
Erwartet: Spiel startet und läuft wie zuvor; Score steigt weiterhin bei Annäherung an die aufrechte Lage, und steigt zusätzlich schneller, wenn der Winkel über mehrere Sekunden durchgehend unter 5° bleibt (am einfachsten mit `H` für Auto-Modus beobachtbar).

- [ ] **Step 8: Commit**

```bash
git add tests/test_scoring.py pendulum_game_controlled.py
git commit -m "feat: add continuous stability bonus to scoring"
```

---

## Task 2: Mode-Klassifikation (`classify_mode`) und Zeit-Tracking

**Files:**
- Create: `tests/test_mode.py`
- Modify: `pendulum_game_controlled.py` (neue Funktion `classify_mode`, direkt nach `compute_score_increment`; Integration von `auto_time`/`manual_time` in `run_game()`; `run_game()` gibt jetzt `(score, mode)` zurück; `main()` entpackt den Rückgabewert)

**Interfaces:**
- Consumes: nichts von Task 1 direkt (unabhängige Funktion), wird aber in derselben `run_game()`-Schleife wie `compute_score_increment` verankert.
- Produces: `classify_mode(auto_time: float, manual_time: float) -> str`, gibt `"Auto"`, `"Manual"` oder `"Mixed"` zurück. `run_game(screen) -> tuple[float, str]` (Score, Mode) statt bisher nur `float`.

- [ ] **Step 1: Fehlschlagenden Test schreiben**

Create `tests/test_mode.py`:

```python
from pendulum_game_controlled import classify_mode


def test_classify_mode_pure_auto():
    assert classify_mode(auto_time=40.0, manual_time=0.0) == "Auto"


def test_classify_mode_pure_manual():
    assert classify_mode(auto_time=0.0, manual_time=40.0) == "Manual"


def test_classify_mode_mixed():
    assert classify_mode(auto_time=10.0, manual_time=30.0) == "Mixed"
```

- [ ] **Step 2: Test laufen lassen, Fehlschlag bestätigen**

Run: `python -m pytest tests/test_mode.py -v`
Expected: FAIL mit `ImportError: cannot import name 'classify_mode'`.

- [ ] **Step 3: Funktion implementieren**

Direkt nach der `compute_score_increment`-Funktion (vor `def run_game(screen):`) einfügen:

```python
def classify_mode(auto_time, manual_time):
    if manual_time <= 0.0 and auto_time > 0.0:
        return "Auto"
    if auto_time <= 0.0 and manual_time > 0.0:
        return "Manual"
    return "Mixed"
```

- [ ] **Step 4: Test laufen lassen, Erfolg bestätigen**

Run: `python -m pytest tests/test_mode.py -v`
Expected: 3 passed.

- [ ] **Step 5: In `run_game()` integrieren**

State-Init-Block um zwei Zeilen ergänzen (nach `stable_streak = 0.0` aus Task 1):

```python
    stable_streak = 0.0
    auto_time = 0.0
    manual_time = 0.0
```

Im `if auto_mode: / else:`-Block (Tastatursteuerung) je einen Akkumulator ergänzen:

```python
        if auto_mode:
            tau = controller.compute(phi, vphi, s, v)
            auto_time += dt
        else:
            tau = 0.0
            if keys[pygame.K_LEFT]:
                tau = -MAX_TAU
            if keys[pygame.K_RIGHT]:
                tau = MAX_TAU
            manual_time += dt
```

Den bestehenden `return score` am Ende von `run_game()` ersetzen durch:

```python
    return score, classify_mode(auto_time, manual_time)
```

- [ ] **Step 6: `main()` anpassen**

In `main()`:

```python
    while True:
        score = run_game(screen)
        player_name = get_player_name(screen)
        update_leaderboard(score, player_name)
```

ersetzen durch:

```python
    while True:
        score, mode = run_game(screen)
        player_name = get_player_name(screen)
        update_leaderboard(score, player_name)
```

(`mode` wird hier bewusst noch nicht an `update_leaderboard` übergeben — das folgt in Task 3, wenn die Funktion den Parameter unterstützt.)

- [ ] **Step 7: Manuell verifizieren**

Run: `python pendulum_game_controlled.py`
Erwartet: Spiel läuft wie zuvor bis zum Ende einer Runde durch, ohne Absturz (insbesondere kein `TypeError` beim Entpacken von `run_game()`s Rückgabewert).

- [ ] **Step 8: Commit**

```bash
git add tests/test_mode.py pendulum_game_controlled.py
git commit -m "feat: track auto/manual time and classify play mode"
```

---

## Task 3: Leaderboard um `Mode` und `Difficulty` erweitern

**Files:**
- Create: `tests/test_leaderboard.py`
- Modify: `pendulum_game_controlled.py` (`update_leaderboard`, `overlay_leaderboard`, `main`)

**Interfaces:**
- Consumes: `mode` aus `run_game()`s Rückgabewert (Task 2).
- Produces: `update_leaderboard(score: float, player_name: str, mode: str, difficulty: str = "Standard", filename: str = "leaderboard.csv") -> None`.

- [ ] **Step 1: Fehlschlagenden Test schreiben**

Create `tests/test_leaderboard.py`:

```python
import pandas as pd

from pendulum_game_controlled import update_leaderboard


def test_update_leaderboard_writes_mode_and_default_difficulty(tmp_path):
    filename = tmp_path / "leaderboard.csv"

    update_leaderboard(score=12.34, player_name="Alice", mode="Auto", filename=str(filename))

    df = pd.read_csv(filename)
    assert df.loc[0, "Mode"] == "Auto"
    assert df.loc[0, "Difficulty"] == "Standard"


def test_update_leaderboard_custom_difficulty(tmp_path):
    filename = tmp_path / "leaderboard.csv"

    update_leaderboard(
        score=5.0, player_name="Bob", mode="Manual", difficulty="Hard", filename=str(filename)
    )

    df = pd.read_csv(filename)
    assert df.loc[0, "Difficulty"] == "Hard"
```

- [ ] **Step 2: Test laufen lassen, Fehlschlag bestätigen**

Run: `python -m pytest tests/test_leaderboard.py -v`
Expected: FAIL mit `TypeError: update_leaderboard() missing 1 required positional argument: 'mode'`.

- [ ] **Step 3: `update_leaderboard` anpassen**

Bestehende Funktion:

```python
def update_leaderboard(score, player_name, filename="leaderboard.csv"):
    now = datetime.now()
    entry = {
        "Date": now.strftime("%Y-%m-%d"),
        "Time": now.strftime("%H:%M:%S"),
        "Name": player_name,
        "Score": round(score, 2),
    }
    df = (
        pd.read_csv(filename)
        if os.path.exists(filename)
        else pd.DataFrame(columns=["Date", "Time", "Name", "Score"])
    )
    df = pd.concat([df, pd.DataFrame([entry])], ignore_index=True)
    df = df.sort_values(by="Score", ascending=False).reset_index(drop=True)
    df.to_csv(filename, index=False)
    print(f"{player_name} achieved score: {score:.2f} - written to {filename}\n")
```

ersetzen durch:

```python
def update_leaderboard(score, player_name, mode, difficulty="Standard", filename="leaderboard.csv"):
    now = datetime.now()
    entry = {
        "Date": now.strftime("%Y-%m-%d"),
        "Time": now.strftime("%H:%M:%S"),
        "Name": player_name,
        "Score": round(score, 2),
        "Mode": mode,
        "Difficulty": difficulty,
    }
    df = (
        pd.read_csv(filename)
        if os.path.exists(filename)
        else pd.DataFrame(columns=["Date", "Time", "Name", "Score", "Mode", "Difficulty"])
    )
    df = pd.concat([df, pd.DataFrame([entry])], ignore_index=True)
    df = df.sort_values(by="Score", ascending=False).reset_index(drop=True)
    df.to_csv(filename, index=False)
    print(f"{player_name} achieved score: {score:.2f} - written to {filename}\n")
```

- [ ] **Step 4: Test laufen lassen, Erfolg bestätigen**

Run: `python -m pytest tests/test_leaderboard.py -v`
Expected: 2 passed.

- [ ] **Step 5: `overlay_leaderboard` anpassen**

Die Fallback-Spalten sowie die angezeigte Zeile ergänzen:

```python
def overlay_leaderboard(screen, filename="leaderboard.csv", top_n=10):
    df = (
        pd.read_csv(filename).sort_values(by="Score", ascending=False).head(top_n)
        if os.path.exists(filename)
        else pd.DataFrame(columns=["Date", "Time", "Name", "Score", "Mode", "Difficulty"])
    )
```

und im Rendering-Loop:

```python
    for i, row in df.iterrows():
        rendered = entry_font.render(
            f"{i+1:>2}. {row['Name']:10} {row['Score']:.2f}  [{row['Mode']}]",
            True,
            (255, 220, 180),
        )
        panel.blit(rendered, (60, 120 + i * 35))
```

- [ ] **Step 6: `main()` anpassen**

```python
    while True:
        score, mode = run_game(screen)
        player_name = get_player_name(screen)
        update_leaderboard(score, player_name, mode)
```

- [ ] **Step 7: Manuell verifizieren**

Run: `python pendulum_game_controlled.py`, eine Runde zu Ende spielen, Namen eingeben.
Erwartet: `leaderboard.csv` enthält danach eine neue Zeile mit gefüllten Spalten `Mode` (`Auto`/`Manual`/`Mixed`, je nach gespielter Runde) und `Difficulty` (`Standard`). Die Bestenliste beim nächsten Start zeigt den `[Mode]`-Zusatz hinter dem Score an.

- [ ] **Step 8: Commit**

```bash
git add tests/test_leaderboard.py pendulum_game_controlled.py
git commit -m "feat: extend leaderboard with Mode and Difficulty columns"
```

---

## Task 4: Wechsel auf die MultiBody-FMU

**Files:**
- Modify: `pendulum_game_controlled.py` (`run_game`, Zeile mit `fmu_path`)

**Interfaces:**
- Consumes: `InvertedPendulumMB.fmu` muss im Projektroot liegen (per `export_mb.mos` erzeugt, siehe `AP1_Validierung.md`).
- Produces: keine neuen Schnittstellen — rein interner Pfadwechsel.

- [ ] **Step 1: FMU-Pfad prüfen**

Run: `ls InvertedPendulumMB.fmu` (im Projektroot)
Erwartet: Datei existiert. Falls nicht: mit OpenModelica `export_mb.mos` erneut ausführen (siehe `AP1_Validierung.md`, Abschnitt 3.3, Euler-Solver verwenden, nicht CVODE).

- [ ] **Step 2: Pfad in `run_game()` ändern**

```python
    fmu_path = os.path.abspath("InvertedPendulum.fmu")
```

ersetzen durch:

```python
    fmu_path = os.path.abspath("InvertedPendulumMB.fmu")
```

- [ ] **Step 3: Manuell verifizieren**

Run: `python pendulum_game_controlled.py`
Erwartet: Spiel lädt ohne Fehler, Pendel startet in der gleichen visuellen Ausgangslage wie zuvor, Pfeiltasten und `H` (Auto-Modus) verhalten sich wie gewohnt. Die Bewegung fühlt sich numerisch minimal anders an als mit dem alten Modell (erwartet, siehe Design-Dokument Abschnitt 3 — Euler- statt CVODE-Solver).

- [ ] **Step 4: Commit**

```bash
git add pendulum_game_controlled.py
git commit -m "feat: switch game to the MultiBody FMU"
```

---

## Task 5: Pause-Steuerung (`P`)

**Files:**
- Modify: `pendulum_game_controlled.py` (`redraw`, `run_game`)

**Interfaces:**
- Consumes: nichts Neues von vorherigen Tasks.
- Produces: `redraw(..., auto_mode=False, paused=False)` — neuer optionaler Parameter, alle Aufrufstellen aktualisiert.

- [ ] **Step 1: `redraw()`-Signatur und Paused-Badge ergänzen**

```python
def redraw(screen, time, dt, score, precision, s, v, taus, phis, auto_mode=False):
```

ersetzen durch:

```python
def redraw(screen, time, dt, score, precision, s, v, taus, phis, auto_mode=False, paused=False):
```

Nach dem bestehenden AUTO/MANUAL-Badge-Block (nach dem `display(badge_label, ...)`-Aufruf, vor dem Ende der Funktion) ergänzen:

```python
    if paused:
        pause_rect = pygame.Rect(
            width - math.ceil(scale * 0.30),
            math.ceil(scale * 0.02) + badge_rect.height + math.ceil(scale * 0.01),
            math.ceil(scale * 0.28),
            math.ceil(scale * 0.06),
        )
        pygame.draw.rect(screen, dunkel, pause_rect, border_radius=8)
        pygame.draw.rect(screen, "black", pause_rect, 2, border_radius=8)
        display(
            "PAUSED  [P to resume]",
            pause_rect.center,
            color=(255, 255, 255),
            size=math.ceil(scale / 42),
        )
```

- [ ] **Step 2: `run_game()` um Pause-State erweitern**

State-Init-Block um eine Zeile ergänzen:

```python
    auto_mode = False
    paused = False
    stable_streak = 0.0
```

Die beiden bestehenden `redraw(...)`-Aufrufe (den einmaligen vor der Schleife und den am Ende der Schleife) um `paused` erweitern:

```python
    redraw(screen, time, dt, 0, 0.25, s, v, [phi], [vphi], auto_mode, paused)
```

und

```python
        redraw(screen, time, dt, score, 0.25, s, v, taus, phis, auto_mode, paused)
```

Im Event-Loop (nach der bestehenden `H`-Abfrage) ergänzen:

```python
            if event.type == pygame.KEYDOWN and event.key == pygame.K_h:
                auto_mode = not auto_mode
            if event.type == pygame.KEYDOWN and event.key == pygame.K_p:
                paused = not paused
```

Direkt nach dem Event-Loop, vor dem bestehenden `if auto_mode:`-Block, den Pause-Skip einfügen:

```python
        if paused:
            plot_taus = taus if taus else [0.0]
            plot_phis = phis if phis else [phi - math.pi]
            redraw(screen, time, dt, score, 0.25, s, v, plot_taus, plot_phis, auto_mode, paused)
            pygame.display.flip()
            clock.tick(60)
            continue

        if auto_mode:
            tau = controller.compute(phi, vphi, s, v)
            auto_time += dt
```

- [ ] **Step 3: Manuell verifizieren**

Run: `python pendulum_game_controlled.py`
Erwartet: `P` drücken friert Zeit, Score und Pendelbewegung sichtbar ein (Anzeige „PAUSED  [P to resume]“ erscheint), erneutes `P` setzt exakt an der eingefrorenen Stelle fort. Pause sofort nach Rundenstart (vor der ersten Bewegung) führt zu keinem Absturz.

- [ ] **Step 4: Commit**

```bash
git add pendulum_game_controlled.py
git commit -m "feat: add pause control (P key)"
```

---

## Task 6: Reset-Steuerung (`R`)

**Files:**
- Modify: `pendulum_game_controlled.py` (`run_game`)

**Interfaces:**
- Consumes: `fmu.reset()` (FMI2-Standardmethode von `FMU2Slave`), `stable_streak`/`taus`/`phis`/`score`/`time` aus Task 1/2/5.
- Produces: keine neue öffentliche Schnittstelle — internes Verhalten von `run_game()`.

- [ ] **Step 1: Reset-Handler im Event-Loop ergänzen**

Nach der in Task 5 ergänzten `P`-Abfrage einfügen:

```python
            if event.type == pygame.KEYDOWN and event.key == pygame.K_r:
                fmu.reset()
                fmu.enterInitializationMode()
                fmu.exitInitializationMode()
                time = 0.0
                score = 0.0
                stable_streak = 0.0
                taus, phis = [], []
                s = fmu.getReal([s_ref])[0]
                v = fmu.getReal([v_ref])[0]
                phi = fmu.getReal([phi_ref])[0]
                vphi = fmu.getReal([vphi_ref])[0]
```

- [ ] **Step 2: Manuell verifizieren**

Run: `python pendulum_game_controlled.py`
Erwartet: Nach einigen Sekunden Spielzeit (Score > 0, Zeit > 0) `R` drücken → Zeit- und Score-Anzeige springen sofort auf `0`, Pendel springt auf die ursprüngliche Startauslenkung zurück, die Runde läuft normal weiter bis `GAME_DURATION` (nicht bis zum Leaderboard-Screen). `Auto`/`Manual`-Modus bleibt beim Reset unverändert erhalten (falls Auto-Modus aktiv war, bleibt er es). Reset funktioniert sowohl im pausierten als auch im laufenden Zustand.

- [ ] **Step 3: Commit**

```bash
git add pendulum_game_controlled.py
git commit -m "feat: add reset control (R key)"
```
