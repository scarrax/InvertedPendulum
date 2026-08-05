# CLAUDE.md

Stable project context for Claude Code sessions in this repo: what's done, what's in progress, and the conventions that must hold across all work packages (AP1–AP6). This file is the durable memory of the *project*, not of any one conversation — session resume/context handles "what are we doing right now"; this file answers "what is true about the project regardless of which session is running."

Maintenance rule: when a work package (AP) reaches a new status, update its row in the table below and append a dated entry under "AP History" — don't rewrite history, append to it. Keep entries factual and terse (what changed, not how). Do not record in-progress task-level detail here (which task of which plan is mid-review, ledger state, etc.) — that lives in `.superpowers/sdd/<plan>/progress.md` inside the relevant worktree and is expected to disappear once a branch finishes.

## Project Overview

Inverted pendulum: an OpenModelica model (cart + pendulum) exported as an FMU, driven in real time by a Pygame game (`pendulum_game_controlled.py`) via `fmpy`. Full scope and Arbeitspakete are defined in `Projektanweisung_Invertiertes_Pendel.md`.

## Arbeitspakete (AP) Status

| AP | Description | Status |
|----|---|---|
| AP1 | Modellierung mit Standardbibliotheken (MultiBody FMU vs. formelbasiertes Modell) | ✅ Done — validated, see `AP1_Validierung.md` |
| AP2 | Interaktive Steuerung und Punktesystem | ✅ Done — merged to `main`, human-verified (H/P/R, MultiBody FMU) |
| AP3 | Automatisierte Regler (LQR, Swing-up, …) | ⬜ Not started |
| AP4 | Schwierigkeitsgrade | ⬜ Not started (AP2 prepared a `Difficulty` leaderboard column with placeholder `"Standard"`) |
| AP5 | Schriftliche Ausarbeitung | ⬜ Not started |
| AP6 | Präsentation | ⬜ Not started |

### AP History

- **2026-08-03**: AP1 validated — MultiBody model matches the flat reference model (momentary acceleration comparison, not trajectory comparison — see convention below). Euler solver required on Windows; CVODE crashes with `STATUS_HEAP_CORRUPTION`.
- **2026-08-05**: AP2 done and merged to `main` (scoring stability bonus, Auto/Manual/Mixed mode tracking, leaderboard `Mode`/`Difficulty` columns + legacy-CSV backfill fix, MultiBody FMU switch, pause `P`, reset `R` with corrected FMI2 `setupExperiment()` sequence). All 6 tasks passed task-scoped review (two needed a fix round); final whole-branch review found and fixed a cross-task gap (reset didn't clear `auto_time`/`manual_time`) plus tooling/robustness issues (root `conftest.py`, leaderboard NaN handling). Human-verified interactively (H/P/R, MultiBody FMU) after merge. Known, deliberately deferred: the stability bonus makes new scores ~40x pre-AP2 leaderboard scores with no visual distinction — noted, not fixed.

## Conventions (binding across all APs)

- **Winkelkonvention**: `phi = 0` is the stable, hanging rest position; `phi = math.pi` is the unstable, upright target position. Any new controller, scoring logic, or difficulty mechanic must respect this — it is not derivable from variable names alone. See `protokoll.md` §2 for the Lagrange derivation this is based on.
- **FMU solver**: export with the **Euler** solver, not CVODE. CVODE FMUs reproducibly crash (`STATUS_HEAP_CORRUPTION`, exit `-1073740940`) under Windows + `fmpy`, even though the model itself is fine — see `AP1_Validierung.md` §3.3. This is a Windows/fmpy interaction, not a modeling error; don't waste time re-diagnosing it, just use Euler.
- **Validating a new/changed model against the reference**: don't compare time-domain trajectories — the system is chaotic (unstable, nonlinear) and trajectories diverge exponentially regardless of model correctness. Compare instantaneous accelerations at fixed states instead (`a`/`alpha` vs. `prismatic.a`/`revolute.a`), which is algebraic and insensitive to the chaos problem. See `AP1_Validierung.md` §3.1–3.2.
- **Controller interface**: new controllers subclass the existing `Controller` base class in `pendulum_game_controlled.py` (see `SimpleController` for the reference implementation, a PD controller on `phi`/`vphi`). This is AP3's contract — don't modify the base class or `SimpleController` outside AP3.
- **Reset uses `fmu.reset()` + `setupExperiment()` + `enterInitializationMode()`/`exitInitializationMode()`**, never tearing down and recreating the FMU instance. The `setupExperiment()` call after `reset()` is required — `fmpy` does not raise on a rejected FMI2 state transition, so omitting it fails silently rather than loudly.
- **`leaderboard.csv` compatibility**: the real file accumulates real rows over time and will not always match whatever schema the current code assumes. Any code that reads it must tolerate rows written by older code (missing columns entirely, not just missing values) — see `_ensure_leaderboard_columns()` in `pendulum_game_controlled.py` for the pattern.
- **Testing**: `pytest`, tests in `tests/`. Pure functions (scoring, mode classification, leaderboard I/O) are unit-tested directly. Anything requiring a live Pygame `Surface` (rendering, the interactive event loop) has no automated test — it needs a human running the game with a real display. Flag this explicitly rather than silently skipping verification when a task touches `redraw()` or the event loop.
- **Timing constants**: `dt = 0.02` (50 Hz simulation step), `GAME_DURATION = 40` seconds per round, `MAX_TAU = 10.0` (force is bang-bang, `tau = ±MAX_TAU` via arrow keys — no finer force dosing until a difficulty AP explicitly adds it).

## Key Files

- `pendulum_game_controlled.py` — the game (Pygame + fmpy + pandas), all game logic lives here (flat-function style, no submodules).
- `*.mo` / `export.mos` / `export_mb.mos` — OpenModelica models and FMU export scripts. `.fmu`/`.log`/build artifacts are gitignored; copy them manually into any worktree that needs to run the game.
- `leaderboard.csv` — persistent leaderboard, real user data, schema evolves (see convention above).
- `protokoll.md` — Lagrange derivation, linearization, controller theory. Reference for the written report (AP5), don't re-derive.
- `AP1_Validierung.md` — AP1's model validation writeup.
- `Projektanweisung_Invertiertes_Pendel.md` — full project brief, all APs, milestones, deliverables.
- `docs/superpowers/plans/` — implementation plans for individual APs (subagent-driven-development / executing-plans format).
