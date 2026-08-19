# AP4: Schwierigkeitsgrade — Design

## Kontext

AP4 aus `Projektanweisung_Invertiertes_Pendel.md` fordert mehrere Schwierigkeitsstufen, die "spürbar unterschiedliche Anforderungen an Spieler oder Regler stellen", mit Vorschlägen für die Variationsachsen: Pendellänge/Masseverteilung, Reibung, Störgrößen, Toleranzbereich. AP2 hat bereits eine `Difficulty`-Spalte im Leaderboard mit Platzhalter `"Standard"` vorbereitet (`_ensure_leaderboard_columns()`, `update_leaderboard()` in `pendulum_game_controlled.py`).

Ein FMU-Check zu Beginn der Brainstorming-Session ergab: `m_cart`, `m_pend`, `d_cart`, `d_pend` sind in `InvertedPendulumMB.fmu` alle `causality=parameter, variability=fixed` — genau wie `phi0`/`vphi0` (AP3 Teil 3) frei per `fmu.setReal()` vor `enterInitializationMode()` setzbar, ohne Neu-Export. Die Pendellänge `l` dagegen ist `causality=calculatedParameter` (aus einer Geometrie abgeleitet) und daher zur Laufzeit nicht direkt setzbar — sie ist deshalb nicht Teil dieses AP4-Slices.

## Ziel

Drei auswählbare Schwierigkeitsstufen (Leicht / Standard / Schwer), die im manuellen Spielmodus sowohl den Punkte-Toleranzbereich als auch Masse/Reibung des Pendel-Modells verändern. Auto-Modus (die AP3-Regler) läuft garantiert immer mit den unveränderten AP3-Standardwerten — kein Regler-Retuning nötig.

## Scope

**In Scope:**
- Drei Stufen: Leicht, Standard, Schwer.
- Zwei Variationsachsen: Toleranzbereich für Punktevergabe (`bonus_zone`, `tight_bonus_zone`) und FMU-Parameter `m_cart`, `m_pend`, `d_cart`, `d_pend`.
- Neue Taste `D` zum zyklischen Durchschalten der Stufen.
- Wirkt ausschließlich im Manual-Modus; Auto-Modus bleibt bei AP3-Standardwerten (technisch erzwungen, siehe unten).
- Leaderboard-Integration (Parameter existiert bereits, muss nur durchgereicht werden).
- HUD-Anzeige der aktuellen Stufe.

**Out of Scope (bewusst zurückgestellt):**
- Pendellänge als Variationsachse (bräuchte OMEdit-Neu-Export mit mehreren fixen Längen, da `l` `calculatedParameter` ist).
- Zufällige Stöße/Störgrößen als eigene Mechanik (Nutzer hat sich für Toleranzbereich + Masse/Reibung entschieden, keine dritte Achse).
- Schwierigkeitsabhängige Variation im Auto-Modus / Regler-Retuning pro Stufe.
- **Doppelpendel** (Nutzeridee während des Brainstormings): würde ein neues Modell, eine neue Lagrange-Herleitung, einen verdoppelten Zustandsraum und ein komplett neues Controller-Interface erfordern — vom Aufwand vergleichbar mit AP1+AP3 zusammen. Als Ausblick-Idee für AP5 ("Fazit und Ausblick") bzw. optionales Stretch-Ziel nach AP4 vorgemerkt, nicht Teil dieses Slices.

## Schwierigkeitsstufen: Werte

| Stufe | bonus_zone | tight_bonus_zone | m_cart | m_pend | d_cart | d_pend |
|---|---|---|---|---|---|---|
| Leicht | 20° | 8° | 5.0 | 0.3 | 0.30 | 0.05 |
| **Standard** | 15° | 5° | 5.0 | 0.5 | 0.15 | 0.01 |
| Schwer | 8° | 3° | 5.0 | 0.9 | 0.05 | 0.01 |

`Standard` ist bit-identisch mit den bisherigen AP3-Werten (Rückwärtskompatibilität zum bestehenden Leaderboard und zu den AP3-Regler-Annahmen). `Leicht` reduziert die Pendelmasse und erhöht die Dämpfung (leichter zu halten, schneller einschwingend) bei breiteren Toleranzzonen. `Schwer` erhöht die Pendelmasse und reduziert die Wagen-Dämpfung (unruhigere Regelstrecke) bei engeren Toleranzzonen.

Diese Zahlen sind ein informierter Ausgangspunkt, kein exakt hergeleitetes Ergebnis. Die Implementierung enthält einen kurzen Playtest-/Sanity-Check pro Stufe (manuell gespielt, ggf. auch eine kurze headless-Simulation mit fester Steuerungssequenz), um zu bestätigen, dass sich die Stufen spürbar unterschiedlich anfühlen bzw. verhalten — analog zum Vorgehen bei der `d_pend`/`K_ENERGY`-Abstimmung in AP3 Teil 2. Falls der Check deutliche Fehleinschätzungen aufdeckt (z. B. "Schwer" fühlt sich nicht schwerer an), werden die Werte in derselben Task angepasst, nicht die Architektur.

## Mechanik

**Auswahl:** Neue Taste `D` schaltet zyklisch Leicht → Standard → Schwer → Leicht. Reihenfolge und Zyklus sind fix; kein Auswahlmenü. Jede neue Runde (`run_game()`-Aufruf) startet bei `Standard` — konsistent mit dem bisherigen Verhalten und dem Leaderboard-Default.

**Wirksamkeit:** `D` löst sofort einen impliziten Reset aus (analog zu `R`): `fmu.reset()` → `fmu.setupExperiment()` → `fmu.setReal([m_cart_ref, m_pend_ref, d_cart_ref, d_pend_ref], [...])` für die neue Stufe → `fmu.enterInitializationMode()` → `fmu.exitInitializationMode()`. Score, Zeit, `stable_streak`, `taus`/`phis`-Verlauf werden wie bei `R` zurückgesetzt. Das setzt exakt das AP3-Teil-3-Muster fort ("Setting an FMU's initial state from outside" in `CLAUDE.md`), nur mit vier zusätzlichen Parametern statt `phi0`/`vphi0`.

**Normaler Reset (`R`):** Behält die aktuell gewählte Stufe bei — `R` setzt nur die Rundenzustände zurück, nicht die Difficulty. Die FMU-Parameter der aktuellen Stufe müssen daher bei jedem `R` (nicht nur bei `D`) neu per `setReal()` gesetzt werden, da `fmu.reset()` alle Parameter auf ihre FMU-Default-Startwerte zurücksetzt.

**Auto/Manual-Isolation (zentrale Anforderung):**
- Solange die aktive Stufe ≠ Standard ist, ist `H` (Auto-Toggle) ein No-Op; stattdessen erscheint ein kurzer Bildschirm-Hinweis (z. B. "Auto nur bei Standard-Schwierigkeit").
- Solange `auto_mode == True`, ist `D` symmetrisch ein No-Op mit entsprechendem Hinweis.
- Dadurch kann Auto-Modus nie mit einer von Standard abweichenden Masse/Reibung laufen — keine Gefahr für die AP3-Regler-Abstimmung, kein Retuning nötig.

**Rückgabewert:** `run_game()` gibt zusätzlich die zuletzt gespielte Difficulty-Stufe zurück (z. B. als drittes Tupel-Element `(score, mode, difficulty)`); `main()` reicht sie an `update_leaderboard(score, player_name, mode, difficulty)` durch (Parameter existiert dort bereits mit Default `"Standard"`).

## Scoring-Integration

`compute_score_increment(angle, stable_streak)` bekommt zwei neue Parameter `bonus_zone` und `tight_bonus_zone` (Defaults = aktuelle Standard-Werte, damit bestehende Aufrufer/Tests ohne Änderung weiterlaufen). Die Stability-Streak-Prüfung in `run_game()` (aktuell `if angle <= math.radians(5)`, Zeile ~587) verwendet testuell denselben Wert wie `tight_bonus_zone` in `compute_score_increment` — das ist im bestehenden Code bereits eine (bisher harmlose) Duplikation der 5°-Konstante an zwei Stellen. Da AP4 diesen Wert ohnehin parametrisierbar machen muss, wird die Duplikation dabei aufgelöst: beide Stellen lesen aus derselben Difficulty-Konfiguration, keine zwei unabhängigen Konstanten mehr.

## HUD

Neues Badge "Difficulty: `<Stufe>`  [D]" im selben visuellen Stil wie das bestehende Auto/Manual-Badge (`redraw()`, Zeilen ~166–183). Platzierung unterhalb des bestehenden Badges (analog zum Pause-Badge). Sowohl AP3-Badges (`L`-Toggle, SwingUp-Submodus) hatten Overflow-Regressionen mit langen Labels — die Implementierung muss das kurze, feste Label-Set (`Leicht`/`Standard`/`Schwer`) gegen die bestehende Badge-Breite prüfen, bevor sie als erledigt gilt.

## Testing

- **Pure/unit-testbar:** die Difficulty-Konfiguration (Stufen-Werte-Tabelle als Datenstruktur), die Zyklus-Funktion (`nächste Stufe aus aktueller Stufe`), `compute_score_increment` mit den neuen Parametern, die Leaderboard-Weiterleitung der Difficulty.
- **FMU-geführt (kein Pygame, Muster wie `tests/test_numerical_stability.py`):** nach Setzen einer Stufe und Reset müssen `fmu.getReal()` auf `m_cart`/`m_pend`/`d_cart`/`d_pend` die erwarteten Werte der Stufe liefern. Guard mit `pytest.mark.skipif(not os.path.exists(fmu_path), ...)`.
- **Nicht automatisiert testbar (wie bei jeder `redraw()`/Event-Loop-Änderung):** die `D`/`H`-Tastenlogik inkl. der gegenseitigen Sperre, das neue Badge-Rendering, das tatsächliche Spielgefühl der drei Stufen. Braucht menschliche interaktive Verifikation — wird im Implementierungsplan explizit als solche markiert, nicht stillschweigend übersprungen.

## Global Constraints (gelten für die Implementierung)

- Winkelkonvention (`phi=0` hängend, `phi=math.pi` aufrecht) bleibt unverändert.
- FMU-Parameter werden ausschließlich über `setReal()` auf `causality=parameter`-Variablen vor `enterInitializationMode()` gesetzt — nie über `phi`/`vphi`/`s`/`v` (die sind `output`/`calculated`).
- `SUBSTEPS=10`-Sub-Stepping-Muster bleibt unverändert; Difficulty ändert nichts an der Doppelschleifen-Struktur in `run_game()`.
- `Controller`-Basisklasse und bestehende Controller (`SimpleController`, `LQRController`, `SwingUpController`) werden nicht verändert.
- `Standard`-Werte müssen exakt den bisherigen AP3-Konstanten entsprechen (Rückwärtskompatibilität Leaderboard + Regler-Annahmen).
