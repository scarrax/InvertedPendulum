# Design: AP2 — Interaktive Steuerung und Punktesystem

Stand: 2026-08-03

## 1. Kontext und Ziel

Laut [Projektanweisung_Invertiertes_Pendel.md](../../../Projektanweisung_Invertiertes_Pendel.md)
(AP2) sollen ausgehend vom bestehenden Spiel (`pendulum_game_controlled.py`):

- die bestehende Pygame-Steuerung (Pfeiltasten, `H` für Auto-Modus) bei Bedarf
  erweitert werden,
- die bestehende Zonen-Logik des Punktesystems geprüft, dokumentiert und bei
  Bedarf verfeinert werden,
- das Leaderboard (`leaderboard.csv`) bei Bedarf um zusätzliche Spalten
  erweitert werden.

AP1 (MultiBody-Modell mit Standardbibliotheken) ist laut
[AP1_Validierung.md](../../../AP1_Validierung.md) abgeschlossen und
validiert. Dieses Dokument ist ein technisches Planungsdokument für die
Umsetzung von AP2, kein Kapitel der schriftlichen Ausarbeitung — die
theoretischen Grundlagen dafür liegen bereits in
[protokoll.md](../../../protokoll.md).

## 2. Entscheidungen (aus Brainstorming-Dialog)

| Frage | Entscheidung |
|---|---|
| Welche FMU nutzt das Spiel? | Wechsel von `InvertedPendulum.fmu` auf `InvertedPendulumMB.fmu` |
| Steuerungserweiterung? | Pause (`P`) und Reset der laufenden Runde (`R`) ergänzen |
| Punktesystem verfeinern? | Zusätzlicher Stabilitäts-Bonus für anhaltende Balance |
| Reset-Mechanismus? | FMI2 `fmu.reset()` statt komplettem Teardown/Neuaufbau |
| Stabilitäts-Bonus-Formel? | Kontinuierlicher, linear wachsender Streak-Bonus (nicht Meilensteine, nicht Multiplikator) |
| Leaderboard-Erweiterung? | Jetzt schon um `Mode` und `Difficulty` erweitern (nicht erst bei AP3/AP4) |

## 3. FMU-Wechsel

`fmu_path` in `run_game()` wechselt von `InvertedPendulum.fmu` auf
`InvertedPendulumMB.fmu`. Beide Modelle exponieren identische Signale
(`tau` → `s`, `v`, `phi`, `vphi`, siehe
[InvertedPendulumMB.mo:13-17](../../../InvertedPendulumMB.mo#L13-L17)), daher
sind keine Änderungen an `ref()`-Aufrufen oder der Regler-Schnittstelle
nötig. Die MB-FMU ist mit dem Euler-Solver exportiert (siehe
`AP1_Validierung.md`, Abschnitt 3.3 — CVODE ist unter Windows instabil);
bei `dt = 0.02 s` ist das für Gameplay-Zwecke ausreichend genau. Die
Trajektorie fühlt sich dadurch numerisch leicht anders an als vorher, das
ist erwartet.

## 4. Steuerung: Pause & Reset

Zwei neue Tasten zusätzlich zu Pfeiltasten und `H`:

- **`P` — Pause/Resume-Toggle.** Im pausierten Zustand wird `fmu.doStep()`
  nicht aufgerufen und `time` nicht erhöht — die Simulation friert exakt
  ein. Event-Polling läuft weiter (Fenster schließen, erneutes `P` zum
  Fortsetzen bleiben möglich). `redraw()` erhält ein `paused`-Flag für einen
  "PAUSED"-Badge (gleiche Stelle/Stil wie der bestehende AUTO/MANUAL-Badge).
  Im Auto-Modus wird während der Pause auch `controller.compute()` nicht
  aufgerufen.
- **`R` — Reset der laufenden Runde.** Kein Rücksprung zum
  Leaderboard-Screen. Ablauf: `fmu.reset()` (FMI2-Standardfunktion), danach
  erneut `enterInitializationMode()` / `exitInitializationMode()` wie beim
  ursprünglichen Setup. Anschließend: `time = 0`, `score = 0`,
  `stable_streak = 0`, `taus`/`phis`-Puffer geleert, `s/v/phi/vphi` frisch
  aus der FMU gelesen.

**Edge Cases:**
- `R` funktioniert unabhängig vom Pause-Zustand; der Pause-Zustand selbst
  bleibt durch einen Reset unverändert.
- Reset setzt Physik und Score zurück, **nicht** `auto_mode` oder die unten
  beschriebenen `auto_time`/`manual_time`-Akkumulatoren fürs Leaderboard —
  ein Reset ist kein neues "Spiel" im Sinne der Bestenliste, sondern ein
  Neustart innerhalb derselben Runde.

## 5. Punktesystem: Stabilitäts-Bonus

Neue Zustandsvariable `stable_streak` (Sekunden). Pro Frame, zusätzlich zur
bestehenden Zonen-Logik (grobe Toleranz, 15°-Zone, 5°-Zone):

```python
if angle <= tight_bonus_zone:          # bestehende 5°-Zone
    stable_streak += dt
else:
    stable_streak = 0.0

score += K_STABILITY * stable_streak    # neuer additiver Term
```

`K_STABILITY` ist ein neuer, dokumentierter Parameter (Startwert: `0.5`,
Feintuning during Implementierung möglich). Der Term wächst linear, solange
der Winkel ununterbrochen unter 5° Abweichung bleibt, und fällt sofort auf 0
zurück, sobald die Zone verlassen wird. Das belohnt gezielt *anhaltende*
Stabilität zusätzlich zur bestehenden, rein winkelabhängigen Bewertung. Ein
Reset (`R`) setzt `stable_streak` ebenfalls auf 0.

Formel für die Ausarbeitung (Kapitel 4):

$$score(t) = score_{zone}(t) + K_{stab} \cdot \Delta t_{stabil}(t)$$

wobei $\Delta t_{stabil}(t)$ die Dauer der aktuellen, ununterbrochenen
Serie innerhalb der 5°-Zone ist.

## 6. Leaderboard-Erweiterung

`leaderboard.csv` bekommt zwei neue Spalten: `Mode` und `Difficulty`. Da die
Datei aktuell nur den Header ohne Datenzeilen enthält, ist dies eine
unkomplizierte Schema-Erweiterung ohne Migrationsproblem.

- **`Mode`**: Während der Runde wird mitgezählt, wie viel Zeit in
  `auto_mode` vs. manuell verbracht wurde (`auto_time`/`manual_time`,
  akkumuliert um `dt` je nach aktuellem Modus). Am Rundenende:
  `"Auto"` falls ausschließlich Auto-Zeit, `"Manual"` falls ausschließlich
  manuell, sonst `"Mixed"` (mindestens ein `H`-Wechsel während der Runde).
- **`Difficulty`**: Platzhalterwert `"Standard"`, da AP4
  (Schwierigkeitsgrade) noch nicht existiert. Die Spalte wird jetzt
  angelegt, damit später keine CSV-Migration nötig ist.

`update_leaderboard()` bekommt zwei neue Parameter (`mode`,
`difficulty="Standard"`). `overlay_leaderboard()` zeigt die Top-10-Zeilen
weiterhin nach Score sortiert, optional mit `Mode`-Kürzel
(z. B. `"12. Alice  1234.56  [Auto]"`).

## 7. Testing

Die Score-Berechnung wird aus der `while`-Schleife von `run_game()` in eine
pure Funktion ausgelagert:

```python
def compute_score_increment(angle, stable_streak, dt) -> float:
    ...  # bestehende Zonen-Logik + neuer Stabilitäts-Term
```

Das ermöglicht `pytest`-Tests ohne Pygame-Fenster:

- Zonengrenzen (0°, 5°, 15°, 90°) liefern erwartete Basiswerte.
- Stabilitäts-Bonus wächst monoton mit `stable_streak`.
- Bonus fällt auf 0 zurück, sobald `angle` außerhalb der 5°-Zone liegt.

`update_leaderboard()` (Mode-Klassifikation `Auto`/`Manual`/`Mixed`) erhält
2-3 Tests mit einem temporären CSV-Pfad.

Pause/Reset-Tastensteuerung und der FMU-Wechsel selbst werden manuell im
laufenden Spiel verifiziert, da dafür der volle Pygame/FMU-Loop nötig ist.

## 8. Nicht Teil dieses Designs

- Schwierigkeitsgrade (AP4) — nur die `Difficulty`-Spalte wird
  vorbereitet, keine tatsächliche Variation von Pendellänge, Reibung o.ä.
- Weitere Regler (AP3, z. B. LQR, Swing-up) — bleiben unverändert
  `SimpleController` als einzige Implementierung.
- Feinere Kraftdosierung (analoge Steuerung statt ±`MAX_TAU`) — bewusst
  nicht Teil dieses Designs.
