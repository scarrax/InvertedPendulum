# Design: AP3 (Teil 3) — Formaler Reglervergleich

Stand: 2026-08-17

## 1. Kontext und Ziel

Laut [Projektanweisung_Invertiertes_Pendel.md](../../../Projektanweisung_Invertiertes_Pendel.md)
(AP3) ist neben der Implementierung mehrerer Regler auch ein "Vergleich der
Regler hinsichtlich Stabilität, Reaktionszeit und Robustheit gegenüber
Störungen" gefordert. AP3 Teil 1 (LQR) und Teil 2 (Swing-up) sind
abgeschlossen und liefern drei Controller nach demselben Interface:
`SimpleController` ("PD"), `LQRController`, `SwingUpController`
(kombiniert Energie-Pumpen + internen LQR). Bisher existieren nur punktuelle,
informelle Vergleichswerte aus früheren Reviews (z.B. "LQR balanciert ab
~11°, PD ab ~2°", "SwingUp captured in ~3-4s") — nicht reproduzierbar,
nicht dokumentiert.

Dieses Design deckt den dritten AP3-Teilschritt ab: ein eigenständiges,
headless Benchmark-Skript, das die drei bestehenden Regler nach denselben
drei Dimensionen (Stabilität, Reaktionszeit, Robustheit) gegen die echte
FMU vermisst und die Ergebnisse als Markdown-Report + Plots festhält, direkt
verwendbar für AP5 Kapitel 5 ("Reglerentwurf und Vergleich"). Das Spiel
selbst (`pendulum_game_controlled.py`) wird nicht verändert — die drei
Controller-Klassen und die `Controller`-Basisklasse bleiben unangetastet
(AP3-Konvention, `CLAUDE.md`).

**Entscheidungen aus dem Brainstorming-Dialog:**

| Frage | Entscheidung |
|---|---|
| Konsument des Vergleichs? | Nur AP5 (schriftliche Ausarbeitung) — kein neues Spiel-Feature |
| Toleranzzone für Erfolg? | Bestehende 5°-Scoring-Zone (`tight_bonus_zone` in `compute_score_increment`) |
| Störform für Robustheitstest? | Kraft-Puls auf `tau` (siehe §3.3 — ursprünglich geplanter vphi-Kick ist gegen die reale Co-Simulation-FMU technisch nicht umsetzbar) |
| Output-Format? | Markdown-Report + PNG-Plots (matplotlib, neue Dependency) |

## 2. Architektur

Neues Skript `benchmark_controllers.py` im Projektroot (flat-file-Stil, wie
der Rest des Projekts):

- importiert `SimpleController`, `LQRController`, `SwingUpController` und
  die FMU-Pfad-/Substep-Konstanten aus `pendulum_game_controlled.py` —
  keine Duplikation der Regler-Logik.
- treibt die FMU headless (kein Pygame) über `fmpy.fmi2.FMU2Slave`, exakt
  im selben Substep-Pattern wie `run_game()`: `SUBSTEPS=10`,
  `inner_dt = dt/SUBSTEPS`, `tau` wird einmal pro äußerem 50Hz-Frame gesetzt
  und über die Substeps konstant gehalten (bindende Konvention aus
  `CLAUDE.md`).
- setzt für jedes Szenario die Anfangsauslenkung über die FMU-Parameter
  `phi0`/`vphi0` (`causality=parameter, initial=exact`, gehört zum
  `InvertedPendulumMB.mo`-Modell selbst — nicht zu verwechseln mit den
  Ausgangsgrößen `phi`/`vphi`, die `causality=output, initial=calculated`
  sind und sich **nicht** von außen überschreiben lassen, weder vor noch
  während der Simulation — empirisch gegen die reale FMU verifiziert).
  `phi0`/`vphi0` müssen wie jeder FMI2-Parameter mit `variability=fixed`
  vor `enterInitializationMode()` gesetzt werden, danach gilt der normale
  Lifecycle aus `tests/test_numerical_stability.py`
  (`instantiate()` → `setupExperiment()` → `enterInitializationMode()`/
  `exitInitializationMode()` → `doStep()`-Schleife).
- berechnet pro Szenario Kennzahlen über reine, unit-testbare Funktionen
  (siehe §4) und sammelt sie in einer Datenstruktur pro Regler.
- schreibt am Ende `AP3_Reglervergleich.md` (Report) und PNGs in
  `benchmark_plots/` (neu, gitignored).

Aufruf: `python benchmark_controllers.py` (kein CLI-Args-Parsing nötig,
YAGNI — alle Szenario-Parameter sind Konstanten im Skript, analog zu
`MAX_TAU`/`GAME_DURATION` im Spiel). Bricht mit klarer Fehlermeldung ab,
wenn `InvertedPendulumMB.fmu` im Projektroot fehlt (gitignored, wie überall
sonst im Projekt).

## 3. Szenarien

Alle Szenarien nutzen dieselbe physikalische Konfiguration wie das Spiel
(`InvertedPendulumMB.fmu`, aktuell `d_pend=0.01`). Anfangsauslenkung wird
immer als `theta = phi - π` (Abweichung von der aufrechten Lage) über den
FMU-State `phi` gesetzt; `vphi=0`, `s=0`, `v=0` als Startwerte, sofern nicht
anders angegeben.

**Erfolgskriterium (wiederverwendet über alle Szenarien):** ein Regler
"hält" die aufrechte Lage ab dem Zeitpunkt, an dem `|theta| < 5°` (bestehende
`tight_bonus_zone`) erreicht ist *und für mindestens 1s ununterbrochen*
bleibt (verhindert False-Positives durch kurzes Durchschwingen durch die
Zone).

### 3.1 Stabilität — Einzugsbereich-Sweep (PD, LQR)

Für `SimpleController` und `LQRController`: Anfangsauslenkung `theta0` wird
in 2°-Schritten erhöht (2°, 4°, 6°, … bis 90°, Abbruch beim ersten
Fehlschlag + einem Bestätigungsschritt danach, um Rauschen an der Grenze
auszuschließen). Für jede Auslenkung läuft die Simulation max. 20s; Erfolg
= Erfolgskriterium wird innerhalb der 20s erreicht. Ergebnis: größte
erfolgreiche `theta0` = "Einzugsbereich" des Reglers in Grad.

`SwingUpController` wird hier nicht separat gesweept — sein Einzugsbereich
ist per Design (Energie-Pumpen) nicht auf eine lokale Umgebung beschränkt,
das eigentliche Kriterium für ihn ist Szenario 3.4.

Nur positive `theta0` werden getestet (nicht zusätzlich das Spiegelbild bei
negativer Auslenkung) — das System ist links-rechts-symmetrisch (Cart-Start
bei `s=0, v=0`), ein separater Sweep für negative `theta0` würde nur
symmetrische Ergebnisse liefern und keine neue Information liefern.

### 3.2 Reaktionszeit — Einschwingzeit bei festen Baseline-Auslenkungen

Zwei feste Baselines, an allen drei Reglern gemessen:

- **2°** — liegt im bekannten Einzugsbereich aller drei Regler.
- **10°** — liegt außerhalb von PDs erwartetem Einzugsbereich (Kreuzcheck
  gegen 3.1), aber innerhalb von LQR/SwingUp (im internen LQR-Teilmodus).

Gemessen wird die Zeit vom Simulationsstart bis zum Erreichen des
Erfolgskriteriums. Max. Laufzeit 20s; wird das Kriterium nicht erreicht,
wird "kein Einschwingen" statt einer Zeit reportet (erwartet für PD bei
10°).

### 3.3 Robustheit — Kraft-Puls auf tau

Ein direkter Geschwindigkeits-Kick auf `vphi` ist gegen die reale
Co-Simulation-FMU nicht umsetzbar (siehe §2) — `vphi` ist eine reine,
solver-interne Ausgabegröße und lässt sich zur Laufzeit nicht von außen
setzen (empirisch verifiziert: ein `fmu.setReal` auf `vphi` zwischen zwei
`doStep()`-Aufrufen wird vom nächsten Schritt vollständig ignoriert).
Stattdessen wird die Störung über den einzigen zur Laufzeit echten
FMU-Input injiziert: `tau`.

Ablauf pro Regler: Simulation startet bei `theta0 = 2°` (im Einzugsbereich
aller drei), läuft bis das Erfolgskriterium erreicht ist (Regler ist
eingeschwungen). Ab diesem Zeitpunkt wird für `KICK_STEPS = 5` aufeinander-
folgende äußere Frames (= 0,1s bei `dt=0,02`) ein fester Zusatz-Offset
`KICK_TAU = 8.0` additiv zum normalen Regler-`tau` addiert (Summe weiterhin
auf `±MAX_TAU=10.0` begrenzt wie im Spiel), danach läuft der Regler wieder
unverändert weiter. `KICK_TAU=8.0` ist deutlich über der stationären
Regelkraft im eingeschwungenen Zustand (nahe 0), aber unter `MAX_TAU`, so
dass der Puls selbst nicht schon in Sättigung startet. Danach läuft die
Simulation weiter (max. 20s ab Ende des Pulses) und die Erholungszeit bis
zum erneuten Erfüllen des Erfolgskriteriums wird gemessen. Wird das
Kriterium nicht erneut erreicht: "keine Erholung" reportet.

### 3.4 Swing-up ab echter Anfangsbedingung (nur SwingUpController)

Zusätzliches Szenario, nur für `SwingUpController` sinnvoll (PD/LQR sind
strukturell nicht in der Lage, aus dieser Auslenkung hochzuschwingen — das
ist genau der in der Projektanweisung genannte Vergleichspunkt für den
Swing-up-Regler). Anfangsbedingung: `phi0 = 0.75·π/2` (identisch zu
`InvertedPendulumMB.mo`, die reale Spiel-Startlage, φ≈67,5° von der
hängenden Ruhelage aus gemessen). Max. Laufzeit 20s, Erfolgskriterium wie
oben. Für PD und LQR wird in der Ergebnistabelle "N/A (strukturell nicht
lösbar)" eingetragen, kein Simulationslauf nötig.

## 4. Kennzahlen-Funktionen (reine Funktionen, unit-testbar)

Diese Funktionen kennen nichts von der FMU — sie arbeiten auf bereits
simulierten `(t, theta)`-Arrays (bzw. Skalaren) und sind unabhängig
testbar:

```python
def held_from(t, theta, tolerance_rad, hold_duration):
    """Return the earliest time in `t` from which |theta| stays below
    `tolerance_rad` for at least `hold_duration` seconds, or None."""

def find_capture_envelope(results_by_theta0):
    """results_by_theta0: dict {theta0_deg: bool success}.
    Return the largest theta0_deg with success=True, given all smaller
    values also succeeded (monotonic sweep assumption), or None."""
```

`held_from` ist die gemeinsame Grundlage für Szenario 3.2 (Zeit vom
Simulationsstart), 3.3 (Zeit ab Kick-Zeitpunkt) und 3.4 (Zeit vom
Simulationsstart) — jeweils mit passendem `t`-Offset aufgerufen.
`find_capture_envelope` wertet Szenario 3.1 aus.

## 5. Output

### 5.1 Report (`AP3_Reglervergleich.md`)

Struktur analog zu `AP1_Validierung.md`/`AP2_Validierung.md`:

1. Kontext (Verweis auf Projektanweisung AP3, kurze Erinnerung an die drei
   Regler und ihre Rolle).
2. Methodik (Zusammenfassung von §3/§4 dieses Designs — Zonen, Kriterium,
   Szenarien).
3. Ergebnistabelle "Stabilität" (Einzugsbereich in Grad, PD/LQR; SwingUp:
   Verweis auf 5.4-Ergebnis statt eigener Zeile).
4. Ergebnistabelle "Reaktionszeit" (Einschwingzeit in s, je Baseline ×
   Regler).
5. Ergebnistabelle "Robustheit" (Erholungszeit in s nach Kick, je Regler).
6. Ergebnis "Swing-up ab realer Anfangsbedingung" (Zeit bis Capture,
   SwingUp; "N/A" für PD/LQR mit Begründung).
7. Kurze Diskussion/Einordnung (2-3 Absätze, keine neue Herleitung — nur
   Einordnung der Zahlen relativ zueinander, mit Verweis auf
   `AP1_Validierung.md` §6 für die zugrundeliegende Physik).

### 5.2 Plots (`benchmark_plots/`, gitignored)

- `envelope_sweep.png` — Balkendiagramm, Einzugsbereich (Grad) je Regler
  (PD, LQR; SwingUp mit separater Markierung/Anmerkung).
- `reaction_time_2deg.png`, `reaction_time_10deg.png` — überlagerte
  φ-Abweichungs-Trajektorien (theta(t)) aller drei Regler für die jeweilige
  Baseline, mit eingezeichneter 5°-Zone.
- `robustness_kick.png` — überlagerte theta(t)-Trajektorien aller drei
  Regler um den Kick-Zeitpunkt herum.
- `swingup_capture.png` — theta(t) des SwingUp-Reglers ab der realen
  Anfangsbedingung, mit markiertem Capture-Zeitpunkt.

`matplotlib` wird zu `requirements.txt` hinzugefügt (neue Dependency,
bisher nutzt das Projekt nur Pygame-eigene Plots im Spiel selbst).

## 6. Testing

- **Unit-Tests** (`tests/test_benchmark_metrics.py`, kein FMU nötig):
  `held_from` und `find_capture_envelope` gegen synthetische `(t, theta)`-
  Arrays — u.a. Fälle "hält von Anfang an", "schwingt kurz durch die Zone,
  ohne zu halten" (muss None/späteren Zeitpunkt liefern), "hält nie",
  "Sweep mit einem einzelnen Ausreißer" (Monotonie-Annahme).
- **Integrationstest** (`tests/test_benchmark_controllers.py`,
  `pytest.mark.skipif` auf FMU-Präsenz, wie `test_numerical_stability.py`):
  fährt eine *reduzierte* Version von je einem Szenario aus 3.1-3.4 gegen
  die echte FMU (kleinere Sweep-Auflösung, kürzere Laufzeiten) und prüft
  grobe Plausibilität (z.B. PD hält bei 2°, PD hält nicht bei 60°, SwingUp
  captured die reale Anfangsbedingung innerhalb 20s) — Regressionsschutz,
  kein Ersatz für den vollen Benchmark-Lauf.
- Kein Test für Plot-Erzeugung selbst (reine Visualisierung, analog zur
  bestehenden Konvention, dass Pygame-`redraw()` nicht automatisiert
  getestet wird) — wird stattdessen einmalig manuell durch Betrachten der
  erzeugten PNGs verifiziert.

## 7. Out of Scope

- Änderungen an `pendulum_game_controlled.py`, `Controller`-Basisklasse
  oder den drei bestehenden Reglern.
- Neuer Spielmodus/UI-Vergleichsanzeige im Spiel.
- Pole-Placement- oder nichtlinearer/lernbasierter Regler (laut
  Projektanweisung optional, kein Teil dieses Vergleichs).
- Schwierigkeitsgrad-Variation (AP4) — alle Szenarien laufen mit der
  Standard-Konfiguration.
- Automatisierte Kick-Magnitude-Sweeps (nur ein fester Kick-Wert je
  Robustheits-Szenario, kein zusätzlicher "maximal beherrschbarer
  Störgröße"-Sweep — YAGNI, die Projektanweisung fordert Robustheit
  gegenüber Störungen, keine Störgrößen-Grenzwertanalyse).
