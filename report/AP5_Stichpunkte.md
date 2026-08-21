# AP5 — Stichpunkte je Kapitel mit Quellen

Reine Gliederungshilfe für's Schreiben der `.tex`-Kapitel unter `chapters/` — kein
Fließtext, keine eigene Quelle. Für jedes Kapitel: was rein sollte + wo die Details
stehen (Datei + Abschnitt). Ersetzt nicht das Lesen der Quellen selbst.

---

## Kapitel 1: Einleitung und Motivation

- Warum invertiertes Pendel als Beispiel: klassisches, nichtlineares, instabiles
  Regelungsproblem
- Projektübergreifende Zielsetzung: MultiBody-Modell statt Formel-Modell, interaktive
  Steuerung, mehrere Regler, Schwierigkeitsgrade
- Ausgangslage: was existierte vorher (formelbasiertes `pendulum.mo`)
- Aufbau der Arbeit (Kapitelübersicht)

**Quellen:** `Projektanweisung_Invertiertes_Pendel.md` §1 (Projektübergreifende
Zielsetzung), §2 (Ausgangslage)

---

## Kapitel 2: Theoretische Grundlagen

- Systembeschreibung: Freiheitsgrade, Zustandsgrößen (`s`, `v`, `phi`, `vphi`)
- Winkelkonvention: `phi=0` hängend (stabil), `phi=pi` aufrecht (instabil, Ziel) —
  nicht aus Variablennamen ableitbar, explizit erklären
- Lagrange-Herleitung der Bewegungsgleichungen
- Linearisierung um die aufrechte Lage (`phi=pi`)
- Bezug zur LQR-Reglerstruktur (State-Space-Form)

**Quellen:** `protokoll.md` §2 (2.1 Systembeschreibung, 2.2 Lagrange-Herleitung,
2.3 Linearisierung, 2.4 Bezug zum implementierten Regler)

---

## Kapitel 3: Modellierung mit Standardbibliotheken

- MultiBody-Modell (Modelica Standard Library: `World`, `Joints.Prismatic`,
  `Joints.Revolute`, `Parts.Body`, Dämpfer) vs. formelbasiertes Referenzmodell
- Validierungsmethodik: **warum nicht** Trajektorienvergleich (System ist chaotisch,
  exponentielle Divergenz unabhängig von Modellkorrektheit) — stattdessen
  Momentanvergleich der Beschleunigungen (algebraisch, unempfindlich gegen Chaos)
- Ergebnisse: sehr gute Übereinstimmung bei entkoppelten Testfällen; systematische,
  aber physikalisch erklärbare Differenz bei gekoppelten Zuständen
  (Trägheitstensor der `Body`-Komponente, den das Punktmasse-Formelmodell nicht hat)
- Analytische Herleitung des Kopplungsterms (Cramer'sche Regel) als Erklärung der
  Differenz, numerisch gegen die Ergebnisse verifiziert
- Technische Randbedingung: FMU-Export mit Euler-Solver, nicht CVODE (CVODE crasht
  unter Windows+fmpy reproduzierbar mit `STATUS_HEAP_CORRUPTION`)
- **Herausforderungen-Abschnitt — CVODE/WSL-Spike als Evidenz:** CVODE-Export lief
  unter WSL2/Linux sauber durch → bestätigt, dass der Windows-Crash eine
  Plattform-Eigenheit ist, kein Modellierungsfehler. Reglerstabilität (PD/LQR/
  Robustheit) unter CVODE identisch zu Euler; einzige Abweichung: SwingUp-Fangzeit
  strukturell langsamer (4.20s → 5.84s), weil Euler numerisch Energie einspeist, die
  CVODEs genauere Integration nicht hat — nicht durch Tuning schließbar
  (Aktuator-Sättigung an `MAX_TAU`)

**Quellen:** `AP1_Validierung.md` (komplett; insb. §3.1–3.3 Methodik/Solver-Wahl,
§4 Ergebnisse, §5 Interpretation, §6 Kopplungsterm-Herleitung), `CLAUDE.md`
Backlog-Eintrag "CVODE-Solver-Vergleich via WSL" (Stand 2026-08-21)

---

## Kapitel 4: Umsetzung der Spielmechanik und des Punktesystems

- Architektur: Pygame (Rendering/Input) + `fmpy` (FMU-Co-Simulation), Game-Loop bei
  50 Hz, `SUBSTEPS=10`-Sub-Stepping (Begründung: Euler-Energieeinspeisung bei vollem
  `dt`, siehe Kapitel 3)
- Interaktive Steuerung: Tasten `H`/`P`/`R`, bang-bang `tau` per Pfeiltasten,
  `MAX_TAU=10`
- Reset-Sequenz: `fmu.reset()` + `setupExperiment()` + Init-Mode (nicht Neuanlage der
  FMU-Instanz) — `setupExperiment()` nach `reset()` zwingend, sonst stiller Fehler
- Punktesystem: `compute_score_increment()` (Basis-Term im 90°-Kegel, `bonus_zone`/
  `tight_bonus_zone`-Boni, Stabilitätsbonus `K_STABILITY * stable_streak`)
- Modi-Tracking: Auto/Manual/Mixed (`classify_mode()`), separates `auto_time`/
  `manual_time`
- Leaderboard: CSV-Persistenz, Schema-Toleranz gegenüber älteren Zeilen
  (`_ensure_leaderboard_columns()`)
- Während der Entwicklung gefundene/behobene Fehler (siehe Quelle §4.3)

**Quellen:** `AP2_Validierung.md` (komplett), `protokoll.md` §1 (Projektüberblick,
Architektur-Hinweis)

---

## Kapitel 5: Reglerentwurf und Vergleich

- PD-Regler (`SimpleController`) als einfacher Ausgangspunkt
- LQR-Regler: Linearisierung um `phi=pi`, Riccati-Gleichung
  (`scipy.linalg.solve_continuous_are`), Gain zur Laufzeit berechnet
- SwingUp-Regler: energiebasiertes Pump-Gesetz (exakt aus den gekoppelten
  Bewegungsgleichungen hergeleitet, keine Näherung), Hysterese-Umschaltung zu LQR
  (`CAPTURE_THETA`/`CAPTURE_VPHI` → `RELEASE_THETA`)
- Formaler Reglervergleich (`benchmark_controllers.py`, headless gegen reale FMU):
  Stabilität (Einzugsbereich-Sweep), Reaktionszeit (feste Baselines), Robustheit
  (Kraft-Puls auf `tau`), Swing-up-Fähigkeit ab realer Startbedingung
- **Zentraler Befund:** PD ist um die aufrechte Lage **strukturell instabil**
  (Eigenwertanalyse von `A − B·K`, positiver Realteil) — kein graduelles, sondern
  ein qualitatives Problem; LQR erreicht 10° Einzugsbereich; SwingUp fängt die reale
  Startbedingung (~112° von oben) in 4.20s
- **Ergänzung — CVODE-vs-Euler-Vergleich (Kapitel-3-Herausforderung erneut
  aufgegriffen):** Reglerstabilität/Reaktionszeit/Robustheit unter CVODE identisch;
  SwingUp-Retuning (`K_ENERGY`, Hysterese-Schwellen) unter CVODE nur marginal
  wirksam (Sättigung an `MAX_TAU`) — bestätigt, dass die Lücke strukturell und nicht
  tuning-bedingt ist

**Quellen:** `protokoll.md` §2.4, `AP3_Reglervergleich.md` (komplett),
`docs/superpowers/specs/2026-08-06-lqr-controller-design.md`,
`docs/superpowers/specs/2026-08-13-ap3-swingup-controller-design.md`,
`docs/superpowers/specs/2026-08-17-ap3-controller-comparison-design.md`,
`CLAUDE.md` AP3-History-Einträge (2026-08-06, 2026-08-13, 2026-08-17) + Backlog
"CVODE-Solver-Vergleich via WSL", Follow-up-Retuning (2026-08-21)

---

## Kapitel 6: Schwierigkeitsgrade und deren Auswirkung

- Drei Stufen (Leicht/Standard/Schwer), zwei Variationsachsen: Score-Toleranzzonen
  (`bonus_zone`/`tight_bonus_zone`) und FMU-Masse/Reibung (`m_cart`/`m_pend`/
  `d_cart`/`d_pend`)
- Aktivierung nur im Manual-Modus (`D`-Taste, zyklisch, mit Reset)
- Mutual-Exclusion-Invariante: `H` nur bei `Standard`, `D` nicht während `auto_mode`
  — garantiert, dass AP3-Regler nie gegen ungewohnte Physik laufen (kein Retuning
  nötig)
- `Standard` reproduziert die Pre-AP4-Werte bit-für-bit (Kontinuität zu AP3)
- Playtest-Befund: ursprüngliche `Leicht`-Werte waren zu hart (mehr Dämpfung dachte
  man würde helfen, verhinderte aber die Pflicht-Schwingphase → Score 0.0),
  korrigiert durch **weniger** statt mehr Dämpfung
- **Neuer Befund (2026-08-21) — Naive-Swing-Scoring-Exploit:** reines
  Resonanz-Schwingen (kein Balance-Versuch) konnte auf `Schwer` mehr Punkte bringen
  als auf `Leicht`, weil der äußere 90°-Scoring-Kegel (anders als `bonus_zone`)
  ursprünglich für alle Level identisch war. Fix: `max_angle` jetzt pro
  Schwierigkeitsgrad (`Schwer`: 40° statt 90°) — stellt die beabsichtigte Reihenfolge
  Leicht > Standard > Schwer beim naiven Spiel wieder her, legitimes präzises Spiel
  bleibt unberührt

**Quellen:** `docs/superpowers/specs/2026-08-19-ap4-difficulty-levels-design.md`,
`CLAUDE.md` AP4-History-Einträge (2026-08-19, 2026-08-21)

---

## Kapitel 7: Ergebnisse und Diskussion

- Übergreifende Synthese über AP1–AP4 (keine reine Wiederholung der Einzelkapitel!)
- Roter Faden: Modellierungsentscheidungen (Kapitel 3) → wie sie Spielmechanik
  (Kapitel 4) und Reglerverhalten (Kapitel 5) beeinflussen → wie Schwierigkeitsgrade
  (Kapitel 6) darauf aufbauen
- Zentrale technische Erkenntnisse gebündelt: PD-Instabilität, Euler-vs-CVODE-Lücke
  beim SwingUp, Scoring-Exploit-Fix — was sagen sie gemeinsam über die
  Modell-/Spieldesign-Qualität aus?
- Grenzen der Arbeit (z. B. keine automatisierten Pygame-Rendering-Tests, bekannte
  offene Punkte aus dem Backlog)

**Quellen:** keine neue Primärquelle — Synthese aus Kapiteln 3–6 und `CLAUDE.md`
(Conventions-Abschnitt für projektweite Entscheidungen)

---

## Kapitel 8: Fazit und Ausblick

- Zusammenfassung: Zielerreichung gegen Projektanweisung (AP1–AP4 abgeschlossen)
- Ausblick — offene Backlog-Punkte:
  - SwingUp-Regler fängt nach Überschwingen nicht (Hysterese-Grenzfall, noch nicht
    analysiert)
  - Weitere Variationsachsen für Schwierigkeitsgrade (z. B. Pendellänge, bräuchte
    OMEdit-Neu-Export)
  - CVODE/WSL-Solver-Vergleich: erledigt (2026-08-21) — als abgeschlossene
    Untersuchung erwähnen, nicht als offen

**Quellen:** `CLAUDE.md` Abschnitt "Ideas / Backlog"
