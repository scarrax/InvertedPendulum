# AP2: Validierung der interaktiven Steuerung und des Punktesystems

Stand: 2026-08-05

## 1. Zielsetzung

Laut Projektanweisung (AP2) sollen die bestehende Pygame-Steuerung und die
Zonen-Scoring-Logik geprüft, dokumentiert und bei Bedarf verfeinert werden;
das Leaderboard soll um Regler-Lauf-Informationen bzw. eine
Schwierigkeitsgrad-Spalte erweitert werden. Umgesetzt wurde das gemäß
Designspezifikation und Implementierungsplan
([docs/superpowers/specs/2026-08-03-ap2-steuerung-punktesystem-design.md](docs/superpowers/specs/2026-08-03-ap2-steuerung-punktesystem-design.md),
[docs/superpowers/plans/2026-08-03-ap2-steuerung-punktesystem.md](docs/superpowers/plans/2026-08-03-ap2-steuerung-punktesystem.md))
in sechs Teilaufgaben. Dieses Dokument fasst Vorgehen, Testergebnisse und
während der Entwicklung gefundene Probleme zusammen — analog zu
`AP1_Validierung.md`, aber mit anderer Methodik: AP2 validiert kein
physikalisches Modell gegen eine Referenz, sondern Programmverhalten gegen
eine Spezifikation.

## 2. Umsetzung (Kurzfassung)

| # | Task | Kern | Code |
|---|---|---|---|
| 1 | Stabilitäts-Bonus | `K_STABILITY=0.5`, `compute_score_increment(angle, stable_streak)` | `pendulum_game_controlled.py:330-345` |
| 2 | Mode-Klassifikation | `classify_mode(auto_time, manual_time)` → `Auto`/`Manual`/`Mixed`; `run_game()` liefert `(score, mode)` | `pendulum_game_controlled.py:355-361` |
| 3 | Leaderboard-Schema | `Mode`/`Difficulty`-Spalten, `_ensure_leaderboard_columns()` für Alt-CSVs | `pendulum_game_controlled.py:197-238` |
| 4 | MultiBody-FMU | `fmu_path` zeigt auf `InvertedPendulumMB.fmu` statt `InvertedPendulum.fmu` | `pendulum_game_controlled.py:340` |
| 5 | Pause (`P`) | `paused`-State, Pause-Skip im Game-Loop, „PAUSED"-Badge | `pendulum_game_controlled.py:423` |
| 6 | Reset (`R`) | `fmu.reset()` → `setupExperiment()` → `enterInitializationMode()`/`exitInitializationMode()`, Reset von `time`/`score`/`stable_streak`/`taus`/`phis`/`auto_time`/`manual_time` | `pendulum_game_controlled.py:425-441` |

Umgesetzt via `superpowers:subagent-driven-development`: pro Task ein
Implementer-Durchlauf + Task-Review (Spec-Compliance + Code-Qualität), am
Ende ein Review über den gesamten Branch. Vollständige Commit-Historie:
`af631b2..b8843b2` (9 Commits, siehe `git log --oneline af631b2..b8843b2`).

## 3. Validierungsmethodik

Anders als bei AP1 (algebraischer Momentanvergleich zweier Modelle) lässt
sich AP2 nicht vollständig automatisiert prüfen: der Großteil der Logik
(Scoring, Mode-Klassifikation, Leaderboard-I/O) ist als reine Funktion
testbar, aber alles, was eine echte Pygame-`Surface` oder Tastatureingaben
braucht (`redraw()`, der Event-Loop, `P`/`R`/`H`), lässt sich nicht headless
automatisiert prüfen (siehe Konvention in `CLAUDE.md`, Abschnitt „Testing").
Die Validierungsmethodik kombiniert deshalb drei Ebenen:

1. **Automatisierte Unit-Tests** (`pytest`) für alle reinen Funktionen.
2. **Task- und Branch-Reviews** durch unabhängige Subagenten (Spec-Abgleich
   + Codequalität), inklusive eines Fix-Loops bei gefundenen Mängeln.
3. **Interaktive Verifikation durch einen Menschen** für alles, was Display
   und Tastatur braucht — die Implementer-Subagenten selbst hatten in dieser
   Session keinen Displayzugriff und konnten `P`/`R`/den MultiBody-FMU-Wechsel
   nur statisch (Code-Trace) oder per headless Pygame-Smoke-Test
   (`SDL_VIDEODRIVER=dummy`) prüfen, nicht visuell.

## 4. Ergebnisse

### 4.1 Automatisierte Tests

```
tests/test_scoring.py::test_zero_beyond_max_angle                       PASSED
tests/test_scoring.py::test_matches_manual_formula_in_tight_zone        PASSED
tests/test_scoring.py::test_stability_bonus_isolated                    PASSED
tests/test_mode.py::test_classify_mode_pure_auto                        PASSED
tests/test_mode.py::test_classify_mode_pure_manual                      PASSED
tests/test_mode.py::test_classify_mode_mixed                            PASSED
tests/test_leaderboard.py::test_update_leaderboard_writes_mode_and_default_difficulty PASSED
tests/test_leaderboard.py::test_update_leaderboard_custom_difficulty    PASSED
tests/test_leaderboard.py::test_ensure_leaderboard_columns_adds_missing_columns PASSED
tests/test_leaderboard.py::test_ensure_leaderboard_columns_preserves_existing PASSED
tests/test_leaderboard.py::test_update_leaderboard_with_legacy_csv      PASSED

11 passed in 3.71s
```

Ausführbar sowohl mit `python -m pytest` als auch mit bare `pytest`
(root-`conftest.py` stellt sicher, dass `pendulum_game_controlled` unter
beiden Aufrufformen importierbar ist — das war während der Entwicklung
kurzzeitig nicht der Fall, siehe 4.3).

### 4.2 Review-Prozess

Jede der 6 Tasks durchlief ein eigenständiges Task-Review (Spec-Compliance
+ Codequalität); zwei Tasks brauchten dabei einen Fix-Round. Am Ende ein
Whole-Branch-Review (Modell: Opus) über den kompletten Diff
`51802b2..4cc3c20`, das drei reale, aufgabenübergreifende Probleme fand
(4.3) sowie mehrere bewusst zurückgestellte Minor-Findings dokumentierte.
Nach dem Fix-Wave (`b8843b2`) war das Review sauber.

### 4.3 Während der Entwicklung gefundene und behobene Fehler

Diese drei Punkte sind der eigentliche Mehrwert dieser Validierung — keiner
davon wäre durch ein einzelnes Task-Review allein aufgefallen:

1. **`KeyError: 'Mode'` bei Alt-Leaderboard-Einträgen.** Die Planannahme
   „`leaderboard.csv` enthält nur die Kopfzeile" war zum Planzeitpunkt
   korrekt, aber veraltet — die reale Datei hatte inzwischen 3 echte
   Datenzeilen ohne `Mode`/`Difficulty`-Spalten. `overlay_leaderboard()`
   crashte beim Rendern. Live vom menschlichen Partner reproduziert.
   Behoben durch `_ensure_leaderboard_columns()`
   (`pendulum_game_controlled.py:197-207`), das fehlende Spalten und
   einzelne `NaN`-Zellen mit Default-Werten auffüllt. Commit `ce4737b`.
2. **Reset (`R`) setzte `auto_time`/`manual_time` nicht zurück.** Task 2
   führte diese Akkumulatoren ein, bevor Task 6 (Reset) existierte; Task 6s
   Review prüfte nur gegen die eigene Aufgabenbeschreibung. Ergebnis: nach
   einem Reset mitten in der Runde beschrieb `classify_mode()` den
   verworfenen Spielabschnitt statt der tatsächlich gewerteten Runde.
   Gefunden im finalen Whole-Branch-Review, behoben in Commit `b8843b2`.
3. **FMI2-Reset-Sequenz unvollständig.** Der im Plan vorgegebene
   Reset-Ablauf (`fmu.reset()` → `enterInitializationMode()` →
   `exitInitializationMode()`) ließ den nach FMI2-Spezifikation
   erforderlichen erneuten `setupExperiment()`-Aufruf aus. `fmpy` wirft bei
   einer abgelehnten FMI2-Zustandsübergangs-Anfrage keine Exception, ein
   fehlerhafter Reset wäre also nicht laut fehlgeschlagen, sondern still.
   Als plan-mandated Finding dem menschlichen Partner vorgelegt, auf dessen
   Entscheidung hin behoben (Commit `4cc3c20`) statt die Planvorgabe
   unverändert zu übernehmen.

Daneben: bare `pytest` schlug mangels root-`conftest.py` mit 3
Importfehlern fehl (nur `python -m pytest` funktionierte) — behoben in
`b8843b2`.

### 4.4 Interaktive Verifikation durch einen Menschen

Nach dem Merge nach `main` wurde das Spiel real gespielt (`H`, `P`, `R`,
MultiBody-FMU) und als funktionsfähig bestätigt. Das schließt genau die
Lücke, die die Implementer-Subagenten mangels Display nicht prüfen
konnten (siehe 3.).

## 5. Interpretation / Einordnung

Alle 6 geplanten Teilaufgaben sind spezifikationskonform umgesetzt,
automatisiert getestet (soweit automatisierbar) und interaktiv verifiziert.
Die drei in 4.3 beschriebenen Fehler zeigen, dass Task-scoped Reviews allein
aufgabenübergreifende Lücken (Fehler 2) und veraltete Planannahmen
(Fehler 1) nicht zuverlässig fangen — dafür brauchte es das
Whole-Branch-Review bzw. echtes Durchspielen.

**Bewusst zurückgestellt:** Der Stabilitäts-Bonus (`K_STABILITY *
stable_streak`, wächst quadratisch mit der Zeit) macht neue Scores ca.
40× größer als die historischen Leaderboard-Einträge (498/462/454), ohne
optische Kennzeichnung. Im Whole-Branch-Review als Finding gemeldet, vom
menschlichen Partner explizit auf „später entscheiden" gesetzt — nicht
Teil dieser Validierung, siehe `CLAUDE.md` AP-History für 2026-08-05.

## 6. Status AP2

- [x] Stabilitäts-Bonus implementiert und getestet
- [x] Mode-Klassifikation implementiert und getestet
- [x] Leaderboard-Schema erweitert, Alt-CSV-Kompatibilität sichergestellt
- [x] MultiBody-FMU eingebunden
- [x] Pause (`P`) implementiert
- [x] Reset (`R`) implementiert, FMI2-Sequenz korrigiert
- [x] Task- und Whole-Branch-Reviews durchgeführt, alle Findings behoben
      oder mit Begründung zurückgestellt
- [x] Interaktive Verifikation durch einen Menschen (`H`/`P`/`R`,
      MultiBody-FMU)
- [ ] Bewusst offen: Score-Skala-Migration (Abschnitt 5)

**AP2 gilt damit als inhaltlich abgeschlossen.**
