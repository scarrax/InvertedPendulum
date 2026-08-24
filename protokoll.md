# Protokoll: Inverted Pendulum Projekt

Stand: 2026-08-24 (AP1–AP4 abgeschlossen; siehe `CLAUDE.md` für den vollständigen,
datierten Verlauf — dieses Dokument hält nur den aktuellen Stand fest, keine Historie)

## 1. Projektüberblick

Das Projekt implementiert ein **Inverted-Pendulum-Spiel/Simulation** bestehend aus einem
Modelica-Physikmodell und einem Python/Pygame-Frontend.

### Dateien

| Datei | Zweck |
|---|---|
| [pendulum.mo](pendulum.mo) | **Referenzmodell** (AP1): flaches, formelbasiertes Modelica-Modell des invertierten Pendels (Wagen + Punktmasse-Stab), Bewegungsgleichungen von Hand hergeleitet/eingetragen. Eingang `tau` (Kraft auf Wagen), Ausgänge `s`, `v`, `phi`, `vphi`. Dient nur noch der Validierung (§2.2), nicht mehr dem Spiel. |
| [export.mos](export.mos) | OpenModelica-Skript, das aus `pendulum.mo` die Referenz-FMU (`InvertedPendulum.fmu`) baut. |
| [InvertedPendulumMB.mo](InvertedPendulumMB.mo) | **Produktivmodell** (seit AP1/AP2): MultiBody-Modell (Modelica Standard Library — `World`, `Joints.Prismatic`, `Joints.Revolute`, `Parts.Body`, Dämpfer), löst die vollständigen gekoppelten Gleichungen automatisch über die Kinematikkette. Parameter `m_cart`, `m_pend`, `l`, `d_cart`, `d_pend`, `s0`/`v0`/`phi0`/`vphi0` (alle `causality=parameter`, extern per `setReal()` vor `enterInitializationMode()` setzbar — nie die `phi`/`vphi`-Outputs direkt, siehe CLAUDE.md-Konvention). Das ist die FMU, die `pendulum_game_controlled.py` tatsächlich lädt. |
| [export_mb.mos](export_mb.mos) | Baut `InvertedPendulumMB.fmu` mit **Euler**-Solver (Windows/`fmpy`-Pflicht — CVODE crasht dort reproduzierbar mit `STATUS_HEAP_CORRUPTION`, siehe `AP1_Validierung.md` §3.3). Das ist die FMU des ausgelieferten/bewerteten Spiels. |
| [export_mb_cvode.mos](export_mb_cvode.mos) | Baut dieselbe MultiBody-FMU mit **CVODE**-Solver, nur unter Linux/WSL lauffähig — Solver-Vergleichs-Spike (§4), nicht Teil des ausgelieferten Spiels. |
| [pendulum_game_controlled.py](pendulum_game_controlled.py) | Das Spiel: Pygame (Rendering/Input) + `fmpy` (FMU-Co-Simulation) + `numpy`/`scipy` (LQR) + `pandas` (Leaderboard). Alle Spiellogik in einer Datei (flacher Funktionsstil). |
| [benchmark_controllers.py](benchmark_controllers.py) | Headless (kein Pygame) Reglervergleich der drei Controller gegen die reale FMU, erzeugt `AP3_Reglervergleich.md` + Plots in `benchmark_plots/`. |
| [requirements.txt](requirements.txt) | Gepinnte Abhängigkeiten (seit AP3). |
| [leaderboard.csv](leaderboard.csv) | Persistente Bestenliste (Datum, Zeit, Name, Score, Mode, Difficulty). Schema toleriert ältere Zeilen ohne diese Spalten. |

### Steuerung / Spielablauf

- Pfeiltasten: manuelle Steuerung der Wagenkraft (bang-bang, `tau = ±MAX_TAU = 10.0`).
- Taste `H`: Umschalten zwischen manuellem und Auto-Modus (nur bei `Standard`-Schwierigkeit
  möglich, siehe unten).
- Taste `L`: Umschalten des Auto-Modus-Reglers zwischen `SimpleController` (PD) und `LQR`.
- Im Auto-Modus übernimmt der gewählte Regler (`SimpleController`, `LQR` oder — abhängig
  vom Startzustand des Pendels — `SwingUpController`) die Steuerung; siehe §2.4.
- Taste `D`: zyklisches Umschalten der Schwierigkeit `Leicht → Standard → Schwer → Leicht`
  (nur im Manual-Modus möglich, löst einen Reset aus).
- Taste `P`: Pause. Taste `R`: Reset der aktuellen Runde.
- Scoring (`compute_score_increment`): Basis-Term innerhalb eines äußeren Winkelkegels
  (`max_angle`, seit AP4 pro Schwierigkeit unterschiedlich groß), plus zwei gestaffelte
  Bonus-Zonen (`bonus_zone`, `tight_bonus_zone`, ebenfalls pro Schwierigkeit skaliert) und
  ein Stabilitätsbonus (`K_STABILITY * stable_streak`) für anhaltendes Halten.
- Nach Spielende: Namenseingabe und Eintrag in `leaderboard.csv` (inkl. Mode- und
  Difficulty-Spalte), Anzeige der Top 10 vor dem nächsten Spiel.
- Simulation läuft mit `dt = 0.02` (50 Hz), pro Frame in `SUBSTEPS = 10` FMU-`doStep()`-
  Aufrufe unterteilt (`tau` über die Substeps konstant) — notwendig, weil explizite
  Euler-Integration bei vollem `dt` numerisch Energie in das leicht gedämpfte Pendel
  einspeist (siehe CLAUDE.md-Konvention "FMU co-simulation must be sub-stepped").

### Architektur-Hinweis

`Controller` ist eine abstrakte Basisklasse ([pendulum_game_controlled.py:342](pendulum_game_controlled.py#L342))
mit drei Implementierungen: `SimpleController` (PD, Zeile 348), `LQRController` (Zeile 378)
und `SwingUpController` (Zeile 406, komponiert intern einen `LQRController` und schaltet
per Hysterese zwischen Energie-Pump-Gesetz und LQR um). Details und Herleitung in §2.4.

---

## 2. Theorie des invertierten Pendels

### 2.1 Systembeschreibung

Ein Wagen (Masse $M$) bewegt sich reibungsbehaftet horizontal auf einer Schiene
(Position $s$, Geschwindigkeit $v$). Am Wagen hängt drehbar ein Pendel (Punktmasse $m$
am Ende einer masselosen Stange der Länge $l$), das sich frei um den Aufhängepunkt drehen
kann (Winkel $\varphi$, Winkelgeschwindigkeit $\dot\varphi$). Einzige Stellgröße ist die
horizontale Kraft $\tau$ auf den Wagen.

Das macht es zum klassischen Regelungs-Benchmark, weil es:

- **unteraktuiert** ist: 2 Freiheitsgrade ($s$, $\varphi$), aber nur 1 Stellgröße ($\tau$) —
  der Pendelwinkel lässt sich nicht direkt steuern, nur indirekt über die Kopplung zum Wagen.
- **nichtlinear** ist: die Kopplung geht über $\sin\varphi$, $\cos\varphi$.
- eine **instabile Gleichgewichtslage** besitzt: die aufrechte Position ist das Regelziel,
  energetisch aber ein Maximum — jede Störung wächst ohne Regelung exponentiell an.

### 2.2 Herleitung der Bewegungsgleichungen (Lagrange)

Mit $\varphi = 0$ als hängender Ruhelage (Pendel zeigt nach unten) liegt die Pendelmasse bei

$$x_p = s + l\sin\varphi,\qquad y_p = -l\cos\varphi$$

Kinetische und potentielle Energie:

$$T = \tfrac12(M+m)v^2 + m\,v\,l\cos\varphi\,\dot\varphi + \tfrac12 m l^2\dot\varphi^2$$

$$V = -mgl\cos\varphi$$

Dämpfung wird über die Rayleigh-Dissipationsfunktion eingeführt (Reibung $d_{cart}v$
am Wagen, Dämpfung $d_{pend}\dot\varphi$ am Gelenk):

$$\mathcal{F} = \tfrac12 d_{cart}v^2 + \tfrac12 d_{pend}\dot\varphi^2$$

Die Euler-Lagrange-Gleichungen $\frac{d}{dt}\frac{\partial L}{\partial \dot q} -
\frac{\partial L}{\partial q} + \frac{\partial \mathcal{F}}{\partial \dot q} = Q$
liefern für $s$ bzw. $\varphi$ (die Coriolis-Kreuzterme $\pm m l\sin\varphi\,v\dot\varphi$
heben sich in der $\varphi$-Gleichung exakt weg):

$$(M+m)\,a + ml\cos\varphi\,\alpha = \tau - d_{cart}v + ml\sin\varphi\,\dot\varphi^2 \qquad (I)$$

$$ml\cos\varphi\,a + ml^2\,\alpha = -mgl\sin\varphi - d_{pend}\dot\varphi \qquad (II)$$

Das ist ein 2×2-lineares Gleichungssystem in $(a,\alpha)$ mit
Massenmatrix-Determinante $D = ml^2(M+m\sin^2\varphi)$. Auflösen (Cramer) liefert:

$$a = \frac{\tau - d_{cart}v + m\sin\varphi\,(l\dot\varphi^2 + g\cos\varphi)
\;+\; \dfrac{d_{pend}\cos\varphi}{l}\dot\varphi}{M + m\sin^2\varphi}$$

$$\alpha = \frac{-\tau\cos\varphi - m l\dot\varphi^2\sin\varphi\cos\varphi - (M+m)g\sin\varphi
\;-\;\dfrac{M+m}{ml}\,d_{pend}\dot\varphi \;+\; \cos\varphi\,d_{cart}v}{l\,(M + m\sin^2\varphi)}$$

**Vergleich mit dem Code:** [pendulum.mo:31-33](pendulum.mo#L31-L33) implementiert nicht
diese vollständige Lösung, sondern eine vereinfachte Variante ohne die beiden hervorgehobenen
Terme — insbesondere fehlt in der $\alpha$-Gleichung die Verstärkung des
Dämpfungskoeffizienten von $d_{pend}$ auf $\frac{M+m}{ml}\,d_{pend}$
($=22\times$ bei $M=5,\,m=0{,}5,\,l=0{,}5$). Das ist **keine kleine** Vereinfachung: sie
entsteht dadurch, dass die Wagen- und Pendelgleichung über den $ml\cos\varphi$-Term in der
Massenmatrix gekoppelt sind — die volle Dämpfungsreaktion wirkt sich erst über die
Wagenbeschleunigung zurück auf $\alpha$ aus, bevor man beide Gleichungen zusammen auflöst.
Die vereinfachte Formel in `pendulum.mo` entspricht stattdessen der Näherung eines fest
eingespannten Wagens ($M\to\infty$), bei der dieser Term gegen $d_{pend}/(ml^2)$
konvergiert — dort wäre die Vereinfachung exakt.

Für $M=5 \gg m=0{,}5$ ist diese Näherung grob, aber nicht beliebig schlecht: numerisch
gegen das MultiBody-Modell (`InvertedPendulumMB.mo`, löst die vollständigen, gekoppelten
Gleichungen automatisch über die Kinematikkette) validiert in
[AP1_Validierung.md, Abschnitt 6](AP1_Validierung.md#6-analytische-herleitung-des-kopplungsterms)
— die obige Herleitung reproduziert die dort gemessene Restdifferenz zwischen flachem und
MultiBody-Modell bis auf ~1% genau.

**Konvention:** Mit dieser Herleitung ist $\varphi = 0$ die *stabile* hängende Lage
(Minimum von $V$) und $\varphi = \pi$ die *instabile* aufrechte Lage (Maximum von $V$).
Das erklärt, warum im Spiel die Zielregion bei $\varphi = \pi$ liegt
(siehe `angle = (phi - math.pi) ...` in [pendulum_game_controlled.py:361](pendulum_game_controlled.py#L361)).

### 2.3 Linearisierung um die aufrechte Lage

Für den Reglerentwurf setzt man $\varphi = \pi + \theta$ mit kleinem $\theta$, also
$\sin\varphi \approx -\theta$, $\cos\varphi \approx -1$. Das System wird dadurch linear
und lässt sich als Zustandsraummodell schreiben:

$$\dot x = Ax + Bu,\qquad x = (s,\, v,\, \theta,\, \dot\theta)^T$$

Dieses linearisierte System ist **steuerbar** (Controllability-Matrix hat vollen Rang) —
das ist die theoretische Rechtfertigung dafür, dass ein einfacher Regler die instabile
Lage überhaupt stabilisieren kann, obwohl nur eine Stellgröße existiert.

### 2.4 Bezug zum implementierten Regler

Drei Regler sind implementiert (AP3), alle gegen die reale MultiBody-FMU verifiziert und
in `AP3_Reglervergleich.md` formal verglichen:

**`SimpleController`** (PD, [pendulum_game_controlled.py:348](pendulum_game_controlled.py#L348))
ist im Kern ein **PD-Regler auf Winkel und Winkelgeschwindigkeit**:

$$\tau = -(K_\varphi\,\theta + K_{\dot\varphi}\,\dot\varphi)$$

Das ist eine reduzierte Version einer vollständigen Zustandsrückführung: er gewichtet
weder $s$ noch $v$, balanciert also das Pendel, lässt den Wagen aber an den Rand der
Schiene driften. Wichtiger: eine Eigenwertanalyse der geschlossenen Schleife $A-B K$ zeigt,
dass PD um die aufrechte Lage **strukturell instabil** ist (positiver Realteil eines
Eigenwerts) — kein graduelles, sondern ein qualitatives Problem. Er erfüllt das
Halte-Kriterium des Benchmarks nur für sehr kleine Anfangsauslenkungen (empirisch ~1,5°),
weil die Divergenz dort langsamer wächst als das Beobachtungsfenster lang ist.

**`LQRController`** ([pendulum_game_controlled.py:378](pendulum_game_controlled.py#L378))
implementiert die in §2.3 skizzierte vollständige Zustandsrückführung
$\tau = -K x$, $x=(s,v,\theta,\dot\theta)^T$, mit $K$ zur Laufzeit über die
Riccati-Gleichung (`scipy.linalg.solve_continuous_are`) aus den *gekoppelten* Matrizen
$A$, $B$ berechnet (nicht aus den vereinfachten Koeffizienten des flachen `pendulum.mo`,
siehe §2.2). Erreicht einen gemessenen Einzugsbereich von ~10° um die aufrechte Lage.

**`SwingUpController`** ([pendulum_game_controlled.py:406](pendulum_game_controlled.py#L406))
adressiert den Fall, dass das Pendel weit von der aufrechten Lage startet — relevant, da
die reale Modell-Anfangsbedingung $\varphi_0 = 0{,}75\cdot\pi/2 \approx 67{,}5^\circ$
([InvertedPendulumMB.mo:10](InvertedPendulumMB.mo#L10), gemessen von der hängenden Lage
also ~112° von der aufrechten Lage entfernt) weit außerhalb des LQR-Einzugsbereichs liegt.
Er komponiert intern einen `LQRController` und schaltet per Hysterese
(`CAPTURE_THETA=10°`/`CAPTURE_VPHI=2.0` → `RELEASE_THETA=25°`) zwischen einem
energiebasierten Pump-Gesetz und der LQR-Übernahme um. Das Pump-Gesetz ist exakt aus den
gekoppelten Bewegungsgleichungen (§2.2) hergeleitet, keine Näherung. Fängt die reale
Startbedingung in ~4,2s (Euler-FMU).

**Solver-Sensitivität (Spike, siehe §4):** Reglerstabilität, Reaktionszeit und Robustheit
sind zwischen Euler- und CVODE-FMU identisch; nur die SwingUp-Fangzeit steigt unter CVODE
auf ~5,84s — plausibel erklärt durch numerische Energieeinspeisung, die spezifisch aus der
expliziten Euler-Integration stammt (siehe SUBSTEPS-Konvention oben) und unter CVODEs
genauerer Integration entfällt. Ein gezieltes Retuning der numerischen Konstanten
(`K_ENERGY`, Hysterese-Schwellen) für CVODE bringt nur ~6% Verbesserung (5,48s), begrenzt
durch Aktuator-Sättigung an `MAX_TAU` — die Regelgesetze selbst (LQR-Gain, Energie-Pump-
Formel) sind lösungsunabhängig, nur ihre numerischen Tuning-Konstanten waren historisch
gegen die Euler-FMU verifiziert.

---

## 3. Schwierigkeitsgrade (AP4)

Drei Stufen (`Leicht`/`Standard`/`Schwer`, Taste `D`, nur im Manual-Modus) variieren zwei
Achsen: die Scoring-Toleranzzonen (`bonus_zone_deg`, `tight_bonus_zone_deg`, `max_angle_deg`)
und die FMU-Massen-/Reibungsparameter (`m_cart`, `m_pend`, `d_cart`, `d_pend`, gesetzt vor
`enterInitializationMode()`). `Standard` reproduziert bit-genau die Pre-AP4-Werte. Eine
Mutual-Exclusion-Invariante garantiert, dass Auto-Modus (AP3-Regler) nie mit
schwierigkeitsveränderter Physik läuft: `H` ist nur bei `Standard` wirksam, `D` nur wenn
`auto_mode == False`.

`max_angle_deg` ist erst nachträglich (2026-08-21) pro Schwierigkeit unterschiedlich
gesetzt worden (`Leicht`/`Standard` 90°, `Schwer` 40°): ein anfangs geteilter,
schwierigkeitsunabhängiger äußerer Scoring-Kegel erlaubte naives, ungezieltes
Resonanz-Schwingen (bang-bang ohne Balance-Versuch) auf `Schwer`, annähernd gleich hohe
Punktzahlen wie auf `Leicht` zu erzielen — das kehrte die beabsichtigte
Schwierigkeits-/Skill-Beziehung faktisch um. Details und Messwerte in `CLAUDE.md`
(AP4-Verlauf, Eintrag 2026-08-21).

---

## 4. Solver-Vergleich (CVODE vs. Euler, WSL-Spike)

Zusätzlich zur produktiven Euler-FMU (Windows-Pflicht, siehe Dateien-Tabelle) wurde eine
CVODE-FMU desselben `InvertedPendulumMB.mo`-Modells unter WSL2/Linux exportiert
(`export_mb_cvode.mos`) und mit `benchmark_controllers.py` unverändert gegen dieselben
Szenarien getestet. Ergebnis: kein Absturz unter Linux (bestätigt, dass der
Windows+`fmpy`-Crash eine reine Plattform-Eigenheit ist, keine Modellierungsschwäche);
Reglerkennzahlen identisch bis auf die SwingUp-Fangzeit (§2.4). Details, Sweep-Ergebnisse
und Nutzungshinweise (`PENDULUM_FMU`/`PENDULUM_SWINGUP_*`-Umgebungsvariablen) in
`CLAUDE.md`, Backlog-Eintrag "CVODE-Solver-Vergleich via WSL".

---

## 5. Offene Punkte / Backlog

- **SwingUp fängt nach Überschwingen nicht**: schießt das Pendel beim Hochschwingen über
  die aufrechte Lage hinaus, kann `SwingUpController` es danach nicht mehr stabilisieren
  (vermuteter Hysterese-Grenzfall, noch nicht analysiert). Betrifft nur Auto-Modus.
- Weitere Variationsachsen für Schwierigkeitsgrade (z. B. Pendellänge) — bräuchte einen
  erneuten `.mo`-Export.
- Keine automatisierte Regressionsabsicherung der AP4-Mutual-Exclusion-Invariante (lebt im
  ungetesteten Pygame-Event-Loop).

Details, Daten und Rulings zu allen oben genannten Punkten stehen in `CLAUDE.md`
("AP History" und "Ideas / Backlog") — dieses Dokument fasst nur den Stand zusammen.
