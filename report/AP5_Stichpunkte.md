# AP5 — Stichpunkte je Kapitel mit Quellen, Formeln, Abbildungs-/Tabellenvorschlägen

Gliederungshilfe für's Schreiben der `.tex`-Kapitel unter `chapters/` — **kein Fließtext,
keine eigene Quelle**. Gliedert sich wie die eigentlichen Kapitel in Unterkapitel, damit sich
das direkt auf `\section{}`/`\subsection{}` in den `.tex`-Dateien abbilden lässt. Für jedes
Unterkapitel: was inhaltlich rein sollte (kleinschrittig), Formeln zum direkten Übernehmen
(gerendert + als LaTeX-Quellcode), Vorschläge wo Abbildungen/Tabellen den Text sinnvoll
ersetzen/ergänzen können. Ersetzt nicht das Lesen der Quellen selbst; die Formeln sind gegen
`protokoll.md` und `Pendel_Herleitung_Ausfuehrlich.md` geprüft.

**Wortbudget-Hinweis** (grobe Richtwerte, ~9.000 Wörter Text gesamt für 20 Seiten):
Kap.1 ~450, Kap.2 ~1.350, Kap.3 ~1.350, Kap.4 ~1.100, Kap.5 ~1.800, Kap.6 ~1.100,
Kap.7 ~1.100, Kap.8 ~700. Abbildungen/Tabellen/Verzeichnisse/Anhang zählen laut
`MCE_Projektablauf (2).pdf` nicht mit — großzügig einsetzen, das kostet kein Seitenbudget.

---

# Kapitel 1: Einleitung und Motivation

## 1.1 Problemstellung und Benchmark-Charakter

- Einstieg: was ist ein invertiertes Pendel (Wagen + drehbares Pendel, eine Stellgröße)
- Drei Eigenschaften kurz benennen, die es zum Standard-Benchmark machen (Details/Herleitung
  folgen in Kapitel 2, hier nur die Einordnung): unteraktuiert, nichtlinear, instabile
  Ruhelage

**Abbildungsvorschlag:**
- *Abbildung 1.1*: Foto/Screenshot des laufenden Spiels (Pygame-Fenster) als Aufmacher —
  vermittelt sofort, worum es geht, bevor die Theorie kommt *(machst du selbst)*

## 1.2 Stand der Technik

- Kurzer Abriss, seit wann das invertierte Pendel als Lehr-/Forschungsbeispiel dient (seit
  den 1960ern)
- Welche Varianten es gibt: Cart-Pole (diese Arbeit), Furuta-Pendel (rotatorisch),
  Doppelpendel
- Welche Reglerklassen historisch/aktuell eingesetzt werden: klassische Zustandsrückführung
  (Pole Placement, LQR), energiebasiertes Swing-up, moderne Ansätze (modellprädiktive
  Regelung, Reinforcement Learning) — Letztere nur kurz erwähnen, nicht Thema dieser Arbeit
- Einordnung des eigenen Projekts in diesen Kontext: Cart-Pole-Variante, klassische
  Regelungsansätze (PD/LQR/energiebasiertes Swing-up), bewusste Scope-Grenze gegenüber
  MPC/RL

**Quellen (extern):**
- Lundberg, K. H.; Barton, T. S. (2010): *History of Inverted-Pendulum Systems*. IFAC
  Proceedings Volumes, Vol. 42, Nr. 24, S. 131–135. DOI: 10.3182/20091021-3-JP-2009.00025.
  → historischer Überblick, gut für den einleitenden Satz "seit den 1960ern Standard-
  Benchmark"
- Boubaker, O. (2013): *The Inverted Pendulum Benchmark in Nonlinear Control Theory: A
  Survey*. International Journal of Advanced Robotic Systems, 10(5), 233.
  → umfassende Übersicht (>100 zitierte Arbeiten) über Pendel-Varianten und Reglerklassen,
  gut geeignet um die eigene Cart-Pole-Wahl und Regler-Auswahl einzuordnen

## 1.3 Zielsetzung und Ausgangslage

- Projektübergreifende Zielsetzung: MultiBody-Modell statt reinem Formel-Modell, interaktive
  Steuerung (Pygame), mehrere Regler im Vergleich, Schwierigkeitsgrade als didaktisches
  Spielelement
- Ausgangslage: was existierte vor diesem Projekt (formelbasiertes `pendulum.mo`, keine
  Interaktivität, kein Regler)

## 1.4 Aufbau der Arbeit

- Kurzüberblick über Kapitel 2–8 (2–3 Sätze reichen, keine Wiederholung der Inhalte)

**Quellen (intern):** `Projektanweisung_Invertiertes_Pendel.md` §1 (Projektübergreifende
Zielsetzung), §2 (Ausgangslage)

---

# Kapitel 2: Theoretische Grundlagen

## 2.1 Systembeschreibung und Freiheitsgrade

- Wagen (Masse $M$) + Pendelmasse ($m$, masselose Stange Länge $l$), Zustandsgrößen
  $s,v,\varphi,\dot\varphi$, einzige Stellgröße $\tau$
- Klassifikation als Regelungs-Benchmark, hier ausführlich (im Gegensatz zur kurzen
  Erwähnung in 1.1): unteraktuiert (2 Freiheitsgrade, 1 Stellgröße), nichtlinear (Kopplung
  über $\sin\varphi/\cos\varphi$), instabile Ruhelage bei $\varphi=\pi$
- Winkelkonvention explizit machen und begründen: $\varphi=0$ hängend (stabil, Minimum von
  $V$), $\varphi=\pi$ aufrecht (instabil, Maximum von $V$, das Regelziel) — nicht aus
  Variablennamen im Code ablesbar, muss explizit erklärt werden

**Formeln:**

Position der Pendelmasse relativ zum Wagen:
$$x_p = s+l\sin\varphi,\quad y_p=-l\cos\varphi$$
```latex
x_p = s + l\sin\varphi, \qquad y_p = -l\cos\varphi
```

**Abbildungsvorschlag:**
- *Abbildung 2.1*: Skizze des Systems (Wagen, Schiene, Pendel, Winkel $\varphi$, Kräfte
  $\tau$) mit eingezeichneter Winkelkonvention ($\varphi=0$ unten, $\varphi=\pi$ oben) —
  **TikZ-Entwurf liegt bereit unter `report/figures/abbildung_2_1_systemskizze.tex`**,
  bitte auf Genauigkeit prüfen (Geometrie ist gegen die $x_p/y_p$-Formeln oben verifiziert,
  siehe Erklärung im Datei-Header)
- *Abbildung 2.2* (optional): Energie-Landschaft $V(\varphi)=-mgl\cos\varphi$ als Kurve über
  $\varphi\in[0,2\pi]$, Minimum bei $0$/$2\pi$ markiert, Maximum bei $\pi$ markiert —
  illustriert die instabile Ruhelage anschaulicher als reiner Text

## 2.2 Herleitung der Bewegungsgleichungen (Lagrange)

- Warum Lagrange-Mechanik statt Newton: Zwangskräfte am Gelenk müssten sonst explizit
  eliminiert werden; Lagrange umgeht das über Energieausdrücke in generalisierten
  Koordinaten $q=(s,\varphi)$
- Herleitungsschritte (Reihenfolge fürs Kapitel empfohlen): Position/Geschwindigkeit der
  Pendelmasse → kinetische Energie $T$ → potentielle Energie $V$ → Rayleigh-
  Dissipationsfunktion $\mathcal F$ für die Dämpfung → allgemeine Euler-Lagrange-Gleichung
  mit Dissipation und externer Kraft
- Euler-Lagrange-Gleichung für $s$ und für $\varphi$ getrennt anwenden
- Wichtiger Zwischenschritt, explizit erwähnen: in der $\varphi$-Gleichung heben sich zwei
  Coriolis-Kreuzterme exakt weg — das ist eine strukturelle Eigenschaft der Herleitung, kein
  Zufall/keine Näherung
- Ergebnis: gekoppeltes $2\times2$-System in $(a,\alpha)$, gelöst per Cramer'scher Regel
  (Determinante $D=ml^2(M+m\sin^2\varphi)$, immer $>0$ — keine Singularität)

**Formeln:**

Kinetische/potentielle Energie:
$$T=\tfrac12(M+m)v^2+mvl\cos\varphi\,\dot\varphi+\tfrac12 ml^2\dot\varphi^2,\qquad V=-mgl\cos\varphi$$
```latex
T = \frac{1}{2}(M+m)v^2 + mvl\cos\varphi\,\dot\varphi + \frac{1}{2}ml^2\dot\varphi^2,
\qquad V = -mgl\cos\varphi
```

Rayleigh-Dissipationsfunktion:
$$\mathcal F=\tfrac12 d_{cart}v^2+\tfrac12 d_{pend}\dot\varphi^2$$
```latex
\mathcal{F} = \frac{1}{2}d_{cart}v^2 + \frac{1}{2}d_{pend}\dot\varphi^2
```

Gekoppelte Bewegungsgleichungen (Ergebnis der Euler-Lagrange-Gleichungen):
$$(M+m)a+ml\cos\varphi\,\alpha=\tau-d_{cart}v+ml\sin\varphi\,\dot\varphi^2 \qquad(I)$$
$$ml\cos\varphi\,a+ml^2\alpha=-mgl\sin\varphi-d_{pend}\dot\varphi \qquad(II)$$
```latex
(M+m)\,a + ml\cos\varphi\,\alpha = \tau - d_{cart}v + ml\sin\varphi\,\dot\varphi^2 \tag{I}
```
```latex
ml\cos\varphi\,a + ml^2\,\alpha = -mgl\sin\varphi - d_{pend}\dot\varphi \tag{II}
```

Aufgelöst (Cramer):
$$a=\frac{\tau-d_{cart}v+m\sin\varphi(l\dot\varphi^2+g\cos\varphi)+\frac{d_{pend}\cos\varphi}{l}\dot\varphi}{M+m\sin^2\varphi}$$
```latex
a = \frac{\tau - d_{cart}v + m\sin\varphi\,(l\dot\varphi^2 + g\cos\varphi)
      + \dfrac{d_{pend}\cos\varphi}{l}\dot\varphi}{M + m\sin^2\varphi}
```
$$\alpha=\frac{-\tau\cos\varphi-ml\dot\varphi^2\sin\varphi\cos\varphi-(M+m)g\sin\varphi-\frac{M+m}{ml}d_{pend}\dot\varphi+\cos\varphi\,d_{cart}v}{l(M+m\sin^2\varphi)}$$
```latex
\alpha = \frac{-\tau\cos\varphi - ml\dot\varphi^2\sin\varphi\cos\varphi
      - (M+m)g\sin\varphi - \dfrac{M+m}{ml}d_{pend}\dot\varphi
      + \cos\varphi\,d_{cart}v}{l\,(M + m\sin^2\varphi)}
```

## 2.3 Linearisierung und Zustandsraumdarstellung

- Linearisierung um $\varphi=\pi+\theta$ mit kleinem $\theta$
  (Kleinwinkelnäherung $\sin\theta\approx\theta$, $\cos\theta\approx1$), Terme höherer
  Ordnung (z. B. $\theta\cdot\dot\theta^2$) werden gestrichen
- Ergebnis: lineares Zustandsraummodell $\dot x=Ax+B\tau$ mit $x=(s,v,\theta,\dot\theta)^T$

**Formeln:**

Linearisierte Zustandsraumform:
```latex
\dot x = \begin{pmatrix}
0 & 1 & 0 & 0 \\
0 & -\dfrac{d_{cart}}{M} & \dfrac{mg}{M} & -\dfrac{d_{pend}}{Ml} \\
0 & 0 & 0 & 1 \\
0 & -\dfrac{d_{cart}}{lM} & \dfrac{(M+m)g}{lM} & -\dfrac{(M+m)d_{pend}}{ml^2M}
\end{pmatrix} x
+ \begin{pmatrix}0\\ 1/M\\ 0\\ 1/(lM)\end{pmatrix}\tau
```

## 2.4 Steuerbarkeit

- Definition: Steuerbarkeitsmatrix $\mathcal C=[B\ AB\ A^2B\ A^3B]$ muss vollen Rang haben
- Bedeutung hier: theoretische Rechtfertigung dafür, dass ein Regler mit nur einer
  Stellgröße die instabile Lage überhaupt stabilisieren kann — Voraussetzung, die vor dem
  Reglerentwurf (Kapitel 5) einmal explizit festgehalten werden sollte

**Quellen (intern):** `protokoll.md` §2 (2.1–2.4), `Pendel_Herleitung_Ausfuehrlich.md`
(vollständige Zwischenschritte, falls beim Schreiben Rückfragen zur Algebra auftauchen)

**Quellen (extern):**
- Ogata, K. (2010): *Modern Control Engineering*, 5. Auflage, Pearson. → Standardwerk für
  Zustandsraumdarstellung, Linearisierung, Steuerbarkeit; guter Beleg für die in diesem
  Kapitel verwendete Terminologie und Definitionen
- Anderson, B. D. O.; Moore, J. B. (1990): *Optimal Control: Linear Quadratic Methods*.
  Prentice Hall. → falls die Steuerbarkeits-/LQR-Theorie hier schon vertieft werden soll
  (sonst besser erst in Kapitel 5 zitieren)

---

# Kapitel 3: Modellierung mit Standardbibliotheken

## 3.1 Referenzmodell vs. MultiBody-Modell

- Zwei Modelle im Projekt: formelbasiertes Referenzmodell (`pendulum.mo`, Punktmasse,
  Bewegungsgleichungen von Hand eingetragen) vs. MultiBody-Produktivmodell
  (`InvertedPendulumMB.mo`, Modelica Standard Library)
- MultiBody-Modell im Detail: verwendete Komponenten (`World`, `Joints.Prismatic`,
  `Joints.Revolute`, `Parts.Body` für Wagen und Pendelmasse, `Translational.Components.Damper`
  für $d_{cart}$, `Rotational.Components.Damper` für $d_{pend}$), Parameter
  ($m_{cart},m_{pend},l,d_{cart},d_{pend},\varphi_0,\dot\varphi_0$)
- Warum MultiBody: löst die vollständigen, gekoppelten Bewegungsgleichungen automatisch über
  die Kinematikkette, keine Handherleitung nötig — Gegenprobe zur eigenen Herleitung
  (Kapitel 2)

**Tabellenvorschlag:**
- *Tabelle 3.1*: Modellparameter im Vergleich (`pendulum.mo` vs. `InvertedPendulumMB.mo`) —
  Spalten: Parameter, Referenzmodell, MultiBody-Modell, Einheit

**Abbildungsvorschlag:**
- *Abbildung 3.1*: Screenshot des MultiBody-Modells aus OMEdit (Diagrammansicht mit den
  verbundenen Komponenten) — zeigt anschaulich, was "Standardbibliothek" bedeutet, ohne dass
  man das im Text beschreiben muss *(machst du selbst)*

## 3.2 Validierungsmethodik

- Warum **kein** Trajektorienvergleich: System ist chaotisch (instabil, nichtlinear),
  Trajektorien divergieren exponentiell unabhängig von Modellkorrektheit
- Stattdessen: Momentanvergleich der Beschleunigungen $a,\alpha$ bei festen Zuständen
  (algebraisch, unempfindlich gegen Chaos)

## 3.3 Validierungsergebnisse

- Sehr gute Übereinstimmung bei entkoppelten Testfällen (reine Wagenbewegung/reine
  Pendelbewegung)
- Systematische, aber physikalisch erklärbare Differenz bei gekoppelten Zuständen
- Ursache: Referenzmodell verwendet eine vereinfachte Dämpfungskopplung (fehlender
  Verstärkungsfaktor $\frac{M+m}{ml}$ auf $d_{pend}$, entspricht der Näherung $M\to\infty$,
  "unendlich schwerer Wagen") — analytisch hergeleitet (Kapitel 2) und numerisch gegen die
  gemessene Differenz verifiziert

**Formeln:**

Vereinfachter Term im Referenzmodell vs. korrekter Term:
$$\text{Referenzmodell: } \frac{d_{pend}}{ml^2}\dot\varphi \qquad\text{korrekt: } \frac{M+m}{m^2l^3}d_{pend}\dot\varphi \;\;(\text{Faktor} \approx 22\times \text{ bei } M{=}5,m{=}0{,}5,l{=}0{,}5)$$
*(exakte Werte/Herleitung siehe `AP1_Validierung.md` §6 — hier nur die Größenordnung als
Beleg für "keine kleine Vereinfachung" zitieren)*

**Tabellenvorschlag:**
- *Tabelle 3.2*: Validierungsergebnisse (Testfall, gemessene Beschleunigung Referenzmodell,
  MultiBody, absolute/relative Differenz) — direkt aus `AP1_Validierung.md` §4 übernehmbar

**Abbildungsvorschlag:**
- *Abbildung 3.2*: Balkendiagramm der Validierungsergebnisse (Testfälle auf x-Achse,
  Differenz in % auf y-Achse) — macht "sehr gute Übereinstimmung außer bei gekoppelten
  Zuständen" auf einen Blick sichtbar

## 3.4 Technische Randbedingungen: Solver-Wahl

- FMU-Export mit Euler-Solver zwingend unter Windows/`fmpy` (CVODE crasht reproduzierbar,
  `STATUS_HEAP_CORRUPTION`) — als reine Plattform-Eigenheit einordnen, nicht als
  Modellierungsfehler

## 3.5 Herausforderungen: CVODE/WSL-Spike

- CVODE-Export unter WSL2/Linux lief sauber durch → bestätigt die Plattform-Hypothese aus
  3.4
- Reglerkennzahlen (Stabilität, Reaktionszeit, Robustheit) identisch zu Euler; einzige
  Abweichung: SwingUp-Fangzeit strukturell langsamer (4,20 s → 5,84 s), erklärt durch
  numerische Energieeinspeisung der expliziten Euler-Integration bei vollem Zeitschritt —
  nicht durch Tuning schließbar (Aktuator-Sättigung, siehe Kapitel 5.6)

**Tabellenvorschlag:**
- *Tabelle 3.3*: CVODE- vs. Euler-Kennzahlen im Überblick (Stabilität, Reaktionszeit,
  Robustheit, SwingUp-Fangzeit) — direkt aus dem CVODE-Backlog-Eintrag in `CLAUDE.md`
  übernehmbar

**Quellen (intern):** `AP1_Validierung.md` (komplett; insb. §3.1–3.3 Methodik/Solver-Wahl,
§4 Ergebnisse, §5 Interpretation, §6 Kopplungsterm-Herleitung), `CLAUDE.md`
Backlog-Eintrag "CVODE-Solver-Vergleich via WSL" (Stand 2026-08-21),
`Pendel_Herleitung_Ausfuehrlich.md` §10 (Vergleich mit dem vereinfachten Referenzmodell)

**Quellen (extern):**
- Fritzson, P. (2014): *Principles of Object-Oriented Modeling and Simulation with Modelica
  3.3: A Cyber-Physical Approach*. Wiley. → Standardreferenz für Modelica/MultiBody-
  Bibliothek allgemein, guter Beleg für Begriffe wie "objektorientierte physikalische
  Modellierung", "Kinematikkette", FMI/FMU-Konzept

---

# Kapitel 4: Umsetzung der Spielmechanik und des Punktesystems

## 4.1 Softwarearchitektur

- Pygame (Rendering/Input) + `fmpy` (FMU-Co-Simulation) + `numpy`/`scipy` (LQR-Berechnung) +
  `pandas` (Leaderboard) — alles in einer Datei, flacher Funktionsstil, kein Framework
- Game-Loop: 50 Hz (`dt=0.02`), pro Frame `SUBSTEPS=10` FMU-`doStep()`-Aufrufe (`tau`
  konstant über die Substeps) — Begründung kurz wiederholen (Euler-Energieeinspeisung bei
  vollem `dt`, Verweis auf Kapitel 3.5)

**Abbildungsvorschlag:**
- *Abbildung 4.1*: Architekturdiagramm (Kasten-Pfeil-Diagramm: Pygame-Loop ↔ `fmpy`/FMU ↔
  Regler-Auswahl ↔ Leaderboard-CSV) — ein Blick genügt, um den Datenfluss zu verstehen,
  spart mehrere Sätze Beschreibung

## 4.2 Interaktive Steuerung

- Pfeiltasten (bang-bang `tau=±MAX_TAU=10.0`), Tastenbelegung `H` (Auto/Manual), `L`
  (Reglerwahl im Auto-Modus), `P` (Pause), `R` (Reset), `D` (Schwierigkeit, Details Kapitel 6)
- Reset-Sequenz technisch wichtig: `fmu.reset()` + `setupExperiment()` + Init-Mode — **nicht**
  Neuanlage der FMU-Instanz; `setupExperiment()` nach `reset()` ist Pflicht, sonst schlägt
  die FMI2-Zustandsmaschine still (ohne Exception) fehl

**Tabellenvorschlag:**
- *Tabelle 4.1*: Tastenbelegung im Überblick (Taste, Wirkung, Bedingung falls vorhanden wie
  bei `D`/`H`)

**Abbildungsvorschlag:**
- *Abbildung 4.2*: Screenshot des Spiel-UI mit Beschriftung der Elemente (Score, Auto/Manual-
  Badge, Schwierigkeits-Badge, Pendel-Visualisierung) *(machst du selbst)*

## 4.3 Punktesystem

- Basis-Term innerhalb eines äußeren Winkelkegels, zwei gestaffelte Bonus-Zonen (enger =
  mehr Bonus, quadratisch gewichtet), zusätzlicher Stabilitätsbonus für anhaltendes Halten
  in der engsten Zone

**Formeln:**

Punktezuwachs pro Frame (`compute_score_increment`):
$$\text{increment}=\begin{cases}0 & \text{wenn } |\theta|>\theta_{max}\\[4pt]\underbrace{0{,}1+0{,}2\cdot\frac{\theta_{max}-|\theta|}{\theta_{max}}}_{\text{Basis-Term}}+\underbrace{2\left(\frac{\theta_{bonus}-|\theta|}{\theta_{bonus}}\right)^2}_{\text{falls }|\theta|\le\theta_{bonus}}+\underbrace{3\left(\frac{\theta_{tight}-|\theta|}{\theta_{tight}}\right)^2}_{\text{falls }|\theta|\le\theta_{tight}}+K_{stab}\cdot t_{stable} & \text{sonst}\end{cases}$$
```latex
\text{increment} =
\begin{cases}
0 & \text{falls } |\theta| > \theta_{\max} \\[4pt]
\left(0{,}1 + 0{,}2\,\dfrac{\theta_{\max}-|\theta|}{\theta_{\max}}\right)
+ 2\left(\dfrac{\theta_{bonus}-|\theta|}{\theta_{bonus}}\right)^{2}_{+}
+ 3\left(\dfrac{\theta_{tight}-|\theta|}{\theta_{tight}}\right)^{2}_{+}
+ K_{stab}\cdot t_{stable} & \text{sonst}
\end{cases}
```
*(die tiefgestellten $+$ bedeuten: Term nur addieren, wenn die jeweilige Zone erreicht ist —
im Fließtext lieber verbal beschreiben statt der Notation, das ist nur eine Gedankenstütze;
$\theta_{max},\theta_{bonus},\theta_{tight}$ sind seit AP4 pro Schwierigkeitsgrad
unterschiedlich, siehe Kapitel 6)*

**Abbildungsvorschlag:**
- *Abbildung 4.3*: Plot der Score-Zuwachsfunktion über dem Winkel $\theta$ (x-Achse: Winkel
  in Grad, y-Achse: increment) für einen festen `stable_streak=0` — zeigt anschaulich die
  gestaffelten Zonen, lässt sich direkt aus `compute_score_increment()` erzeugen (z. B. mit
  matplotlib, ein kleines Wegwerf-Skript reicht)

## 4.4 Modi-Tracking und Leaderboard

- Modi-Tracking: Auto/Manual/Mixed-Klassifikation einer Runde, separate Zeiterfassung
  `auto_time`/`manual_time` für die Leaderboard-Anzeige
- Leaderboard: CSV-Persistenz, Schema-Toleranz gegenüber älteren Zeilen (fehlende Spalten
  werden nachträglich befüllt, nicht als Fehler behandelt) — kurz als Software-Engineering-
  Aspekt erwähnen (Daten-Kompatibilität über Entwicklungszeit hinweg)

**Tabellenvorschlag:**
- *Tabelle 4.2*: Beispielausschnitt aus `leaderboard.csv` (anonymisiert/gekürzt) zur
  Illustration des Schemas (Spalten Score, Mode, Difficulty, …)

## 4.5 Entwicklungsbedingte Fehler (kurzer Rückblick)

- 2–3 prägnanteste während der Entwicklung gefundene/behobene Fehler (nicht die volle Liste
  aus `AP2_Validierung.md` wiederholen, nur die lehrreichsten Beispiele — z. B. der
  `setupExperiment()`-nach-`reset()`-Fehler aus 4.2, da er "stumm" fehlschlägt)

**Quellen (intern):** `AP2_Validierung.md` (komplett), `protokoll.md` §1 (Projektüberblick,
Architektur-Hinweis)

---

# Kapitel 5: Reglerentwurf und Vergleich

## 5.1 PD-Regler

- Einfachster Ausgangspunkt (`SimpleController`): nur Winkel/Winkelgeschwindigkeit,
  ignoriert Wagenposition/-geschwindigkeit komplett

**Formeln:**

PD-Regelgesetz:
$$\tau=-(K_\varphi\theta+K_{\dot\varphi}\dot\varphi)$$
```latex
\tau = -\big(K_\varphi\,\theta + K_{\dot\varphi}\,\dot\varphi\big)
```

## 5.2 LQR-Regler

- Vollständige Zustandsrückführung, Linearisierung aus Kapitel 2.3, Gain über
  Riccati-Gleichung zur Laufzeit berechnet (`scipy.linalg.solve_continuous_are`)
- Kurz erklären, was $Q$ und $R$ bedeuten (Gewichtung Zustandsabweichung vs. Stellaufwand)
  und welche konkrete Gewichtung gewählt wurde und warum ($\theta$ am stärksten gewichtet)

**Formeln:**

LQR-Kostenfunktional und Riccati-Gleichung:
$$J=\int_0^\infty\big(x^TQx+u^TRu\big)\,dt,\qquad A^TP+PA-PBR^{-1}B^TP+Q=0,\qquad K=R^{-1}B^TP,\qquad \tau=-Kx$$
```latex
J = \int_0^\infty \big(x^{T}Qx + u^{T}Ru\big)\,dt
\qquad\text{s.t.}\qquad
A^{T}P + PA - PBR^{-1}B^{T}P + Q = 0,
\qquad K = R^{-1}B^{T}P,
\qquad \tau = -Kx
```

## 5.3 Swing-up-Regler

- Motivation: reale Startbedingung ~112° von der aufrechten Lage entfernt, weit außerhalb
  des LQR-Einzugsbereichs
- Energiebasiertes Pump-Gesetz, exakt aus den gekoppelten Bewegungsgleichungen hergeleitet
  (keine Näherung) — Herleitung Schritt für Schritt:
  1. Pendelenergie definieren ($\varphi=0$ als Referenz)
  2. Zeitableitung $\dot E$ bilden, Gleichung $(II)$ aus Kapitel 2.2 einsetzen
  3. Coriolis-Kürzung (wie schon in 2.2) — übrig bleibt eine einfache Formel mit genau zwei
     Termen (Dämpfungsverlust + steuerbarer Term)
  4. Daraus die Vorzeichen-Logik des Pump-Gesetzes ableiten
  5. Gewünschte Beschleunigung über die $\tau$-Rückumrechnung (inverse Dynamik aus Gleichung
     $(I)$) in eine tatsächliche Stellgröße übersetzen
- Hysterese-Umschaltung zu LQR (`CAPTURE_THETA`/`CAPTURE_VPHI` → `RELEASE_THETA`), mit
  Begründung warum Hysterese nötig ist: verhindert Oszillieren zwischen beiden Modi

**Formeln:**

Pendelenergie:
$$E=\tfrac12 ml^2\dot\varphi^2+mgl(1-\cos\varphi),\qquad E_{\text{top}}=2mgl$$
```latex
E = \frac{1}{2}ml^{2}\dot\varphi^{2} + mgl\,(1-\cos\varphi), \qquad E_{\text{top}} = 2mgl
```

Energiebilanz-Herleitung (zentrales Ergebnis, zeigt warum das Vorzeichen-Gesetz korrekt ist):
$$\dot E=-d_{pend}\dot\varphi^2-ml\cos\varphi\,\dot\varphi\,a$$
```latex
\dot E = -\,d_{pend}\dot\varphi^{2} \;-\; ml\cos\varphi\,\dot\varphi\,a
```

Energie-Pump-Regelgesetz (gewünschte Wagenbeschleunigung):
$$a_{\text{cmd}}=k_{\text{energy}}\,(E-E_{\text{top}})\cdot\operatorname{sign}(\cos\varphi\,\dot\varphi)$$
```latex
a_{\text{cmd}} = k_{\text{energy}}\,\big(E - E_{\text{top}}\big)\cdot
\operatorname{sign}\!\big(\cos\varphi\,\dot\varphi\big)
```

Rückumrechnung auf die Stellgröße $\tau$ (inverse Dynamik, $a=a_{\text{cmd}}$ vorgegeben):
$$\tau=a_{\text{cmd}}(M+m\sin^2\varphi)+d_{cart}v-m\sin\varphi(l\dot\varphi^2+g\cos\varphi)-\frac{d_{pend}\cos\varphi}{l}\dot\varphi$$
```latex
\tau = a_{\text{cmd}}\,(M+m\sin^{2}\varphi) + d_{cart}v
     - m\sin\varphi\,(l\dot\varphi^{2}+g\cos\varphi)
     - \frac{d_{pend}\cos\varphi}{l}\dot\varphi
```

**Abbildungsvorschlag:**
- *Abbildung 5.4*: Phasenporträt/Energie-über-Zeit-Plot des SwingUp-Vorgangs (Energie
  nähert sich $E_{\text{top}}$, dann Umschaltpunkt zu LQR markiert) — illustriert die
  Energie-Pump-Herleitung sehr anschaulich, evtl. selbst aus einer Simulation erzeugen

## 5.4 Methodik des formalen Reglervergleichs

- `benchmark_controllers.py` (headless gegen reale FMU), drei Kriterien: Stabilität
  (Einzugsbereich-Sweep, ab welcher Anfangsauslenkung noch gehalten wird), Reaktionszeit
  (feste Baselines, Zeit bis Halte-Kriterium erfüllt), Robustheit (Kraft-Puls auf `tau`,
  gemessene Peak-Auslenkung + Erholungszeit), plus SwingUp-spezifisches Szenario (Fangzeit
  ab realer Startbedingung)

## 5.5 Ergebnisse und zentrale Befunde

- **Zentraler Befund, ausführlich darstellen:** PD ist um die aufrechte Lage **strukturell
  instabil** (Eigenwertanalyse von $A-BK_{PD}$ zeigt positiven Realteil) — kein graduelles,
  sondern ein qualitatives Problem
- Wichtig für die Diskussion: PD erfüllt das Halte-Kriterium des Benchmarks trotzdem für
  sehr kleine Anfangsauslenkungen (empirisch ~1,5°), weil die Divergenz dort langsamer
  wächst als das Beobachtungsfenster lang ist — das ist ein Artefakt der Messmethode, keine
  tatsächliche Stabilität; unbedingt so einordnen, nicht als "PD funktioniert für kleine
  Winkel" missverständlich formulieren
- LQR erreicht 10° Einzugsbereich; SwingUp fängt die reale Startbedingung in 4,20 s (Euler)

**Tabellenvorschlag:**
- *Tabelle 5.1*: Reglervergleich im Überblick (Regler | Einzugsbereich | Reaktionszeit |
  Robustheit/Peak-Auslenkung | SwingUp-Fangzeit) — direkt aus `AP3_Reglervergleich.md`
  übernehmbar, ist vermutlich die wichtigste Tabelle der ganzen Arbeit

**Abbildungsvorschlag:**
- *Abbildung 5.1*: Eigenwert-Plot der PD-geschlossenen Schleife in der komplexen Ebene
  (zeigt den positiven Realteil visuell — stärker als jede verbale Beschreibung)
- *Abbildung 5.2*: Zeitverlauf $\theta(t)$ für alle drei Regler ab derselben
  Anfangsauslenkung (aus `benchmark_plots/`, direkt wiederverwendbar) — zentrale
  Vergleichsabbildung des Kapitels
- *Abbildung 5.3*: Einzugsbereich-Sweep als Balken-/Liniendiagramm (aus `benchmark_plots/`)

## 5.6 Solver-Sensitivität: CVODE vs. Euler

- Ergänzung zu Kapitel 3.5: Reglerstabilität/Reaktionszeit/Robustheit unter CVODE identisch
  zu Euler; gezieltes Retuning der SwingUp-Konstanten unter CVODE nur marginal wirksam
  (~6%, von 5,84 s auf 5,48 s), begrenzt durch Aktuator-Sättigung (`tau` klemmt an `MAX_TAU`
  unabhängig vom Tuning) — bestätigt, dass die Euler-CVODE-Lücke strukturell (numerische
  Energieeinspeisung) und nicht tuning-bedingt ist
- Wichtige methodische Erkenntnis dabei: die *Regelgesetze* (LQR-Gain, Energie-Pump-Formel)
  sind lösungsunabhängig hergeleitet, nur die *numerischen Tuning-Konstanten* waren
  historisch ausschließlich gegen Euler verifiziert — das beim Schreiben sauber als
  methodische Einschränkung benennen, nicht verschweigen

**Tabellenvorschlag:**
- *Tabelle 5.2*: CVODE- vs. Euler-Retuning-Sweep-Ergebnisse (K_ENERGY-Werte,
  Fangzeit) — zeigt den Sättigungseffekt quantitativ

**Quellen (intern):** `protokoll.md` §2.4, `Pendel_Herleitung_Ausfuehrlich.md` §12
(vollständige Regler-Herleitung inkl. der $\dot E$-Herleitung), `AP3_Reglervergleich.md`
(komplett), `docs/superpowers/specs/2026-08-06-lqr-controller-design.md`,
`docs/superpowers/specs/2026-08-13-ap3-swingup-controller-design.md`,
`docs/superpowers/specs/2026-08-17-ap3-controller-comparison-design.md`,
`CLAUDE.md` AP3-History-Einträge (2026-08-06, 2026-08-13, 2026-08-17) + Backlog
"CVODE-Solver-Vergleich via WSL", Follow-up-Retuning (2026-08-21)

**Quellen (extern):**
- Åström, K. J.; Furuta, K. (2000): *Swinging up a Pendulum by Energy Control*. Automatica,
  36(2), S. 278–285. DOI: 10.1016/S0005-1098(99)00140-5. → **die** klassische Referenz für
  energiebasierte Swing-up-Regelung; der eigene `SwingUpController` folgt demselben
  Grundprinzip (Energie über die Kopplung zur Wagenbeschleunigung pumpen) — unbedingt zitieren,
  wenn das Pump-Gesetz eingeführt wird (5.3), und die eigene Herleitung (exakt aus den
  gekoppelten Gleichungen, nicht Åströms vereinfachtes Modell) davon abgrenzen
- Ogata, K. (2010): *Modern Control Engineering*, 5. Auflage, Pearson. → Referenz für
  Riccati-Gleichung/LQR-Herleitung (5.2), Hysterese-Konzept bei Schaltregelungen (5.3)
- Anderson, B. D. O.; Moore, J. B. (1990): *Optimal Control: Linear Quadratic Methods*.
  Prentice Hall. → tiefere LQR-Theorie (5.2), falls Beweis-/Optimalitätsaspekte diskutiert
  werden sollen (z. B. warum die Riccati-Lösung optimal ist)

---

# Kapitel 6: Schwierigkeitsgrade und deren Auswirkung

## 6.1 Konzept und Variationsachsen

- Drei Stufen (`Leicht`/`Standard`/`Schwer`), zwei Variationsachsen: Score-Toleranzzonen
  ($\theta_{bonus},\theta_{tight}$, seit dem Exploit-Fix auch $\theta_{max}$) und
  FMU-Masse-/Reibungsparameter ($m_{cart},m_{pend},d_{cart},d_{pend}$)
- Aktivierung nur im Manual-Modus (`D`-Taste, zyklisch `Leicht→Standard→Schwer→Leicht`,
  löst impliziten Reset aus)

**Tabellenvorschlag:**
- *Tabelle 6.1*: Alle drei Schwierigkeitsgrade mit vollständigen Parametern
  ($\theta_{bonus},\theta_{tight},\theta_{max},m_{cart},m_{pend},d_{cart},d_{pend}$) —
  zentrale Referenztabelle des Kapitels

## 6.2 Mutual-Exclusion-Invariante

- `H` (Auto-Modus) ist nur bei `Standard`-Physik wirksam, `D` (Schwierigkeitswechsel) ist
  während `auto_mode` wirkungslos — dadurch laufen die AP3-Regler nie gegen untypische
  Physik, kein Retuning pro Schwierigkeitsgrad nötig; das ist der Grund, warum Kapitel 5 und
  Kapitel 6 unabhängig voneinander funktionieren
- `Standard` reproduziert die Pre-AP4-Werte bit-genau (Kontinuität zu AP3, wichtig für die
  Vergleichbarkeit der Kapitel-5-Ergebnisse)

**Abbildungsvorschlag:**
- *Abbildung 6.2* (optional): Zustandsdiagramm der Mutual-Exclusion-Invariante (welche
  Tasten in welchem Modus wirken/nicht wirken) — nur falls Text allein unübersichtlich wird

## 6.3 Playtest-Korrektur der Leicht-Werte

- Befund (gut als kleine "Lessons Learned"-Box geeignet): ursprüngliche `Leicht`-Werte waren
  kontraintuitiv zu hart — mehr Dämpfung sollte das Halten erleichtern, verhinderte aber die
  Pflicht-Schwingphase am Rundenanfang (Score 0,0 in echten Tests)
- Fix: **weniger** statt mehr Dämpfung; guter Beleg dafür, dass Spieltests/reale Messung
  nötig waren, reine Intuition beim Parameterdesign nicht ausreichte

## 6.4 Naive-Swing-Scoring-Exploit

- Befund (neuester, ausführlich darstellen): reines Resonanz-Schwingen (fixe Periode, kein
  Balance-Versuch) konnte auf `Schwer` annähernd gleich viele Punkte bringen wie auf
  `Leicht`, weil der äußere Scoring-Kegel $\theta_{max}$ ursprünglich für alle Level
  identisch (90°) war, während $\theta_{bonus}/\theta_{tight}$ schon skaliert waren — das
  kehrte die beabsichtigte Schwierigkeits-/Skill-Beziehung faktisch um
- Wichtige Methodik-Lektion, explizit erwähnen: **erste Messung mit einer einzelnen
  willkürlichen Schwingperiode überschätzte den Effekt stark** (Faktor 6,5) — ein sauberer
  Sweep über die jeweils eigene Resonanzperiode jeder Schwierigkeit zeigte ein deutlich
  moderateres, aber real vorhandenes Problem (228 vs. 233 Punkte, praktisch gleichauf).
  Messmethode beeinflusst die wahrgenommene Größe eines Effekts erheblich — sauber
  herausarbeiten
- Fix: $\theta_{max}$ jetzt pro Schwierigkeitsgrad (`Schwer`: 40° statt 90°) — stellt die
  beabsichtigte Reihenfolge Leicht > Standard > Schwer beim naiven Spiel wieder her
  (233 > 125 > 111), legitimes präzises Spiel bleibt unberührt, da `Schwer`s
  $\theta_{bonus}/\theta_{tight}$ (8°/3°) weit innerhalb der neuen 40°-Grenze liegen

**Tabellenvorschlag:**
- *Tabelle 6.2*: Naive-Swing-Scores vor/nach dem Fix, je Schwierigkeit, an der eigenen
  Resonanzperiode gemessen (233/125/111 nach Fix) — belegt die Wirksamkeit des Fixes
  quantitativ

**Abbildungsvorschlag:**
- *Abbildung 6.1*: Balkendiagramm Naive-Swing-Score je Schwierigkeit, vorher/nachher
  gruppiert — macht den Exploit und die Korrektur auf einen Blick sichtbar, vermutlich die
  überzeugendste Abbildung der Arbeit für diesen Befund

**Quellen (intern):** `docs/superpowers/specs/2026-08-19-ap4-difficulty-levels-design.md`,
`CLAUDE.md` AP4-History-Einträge (2026-08-19, 2026-08-21)

---

# Kapitel 7: Ergebnisse und Diskussion

## 7.1 Synthese über AP1–AP4

- Übergreifende Synthese (**keine** reine Wiederholung der Einzelkapitel — das ist der
  häufigste Fehler in Diskussionskapiteln, bewusst vermeiden)
- Roter Faden explizit ausformulieren (als Gliederung, nicht als Fließtext hier): wie
  Modellierungsentscheidungen (Kapitel 3, MultiBody vs. Referenzmodell) die Genauigkeit der
  Reglerauslegung beeinflussen (Kapitel 5, LQR-Gain basiert auf der *korrekten* gekoppelten
  Herleitung) → wie das wiederum die Spielmechanik/Schwierigkeitsgrade beeinflusst (Kapitel
  4/6, Mutual-Exclusion-Invariante entkoppelt beides bewusst)

## 7.2 Zentrale technische Erkenntnisse im Vergleich

- Zentrale technische Erkenntnisse gebündelt gegenüberstellen (nicht nur auflisten, sondern
  in Bezug setzen): PD-Instabilität (strukturelles Regelungsproblem) vs.
  Euler-vs-CVODE-Fangzeit-Lücke (numerisches/Solver-Problem) vs. Scoring-Exploit (Spiel-
  Design-Problem) — alle drei sind Beispiele dafür, dass naive/erste Implementierungen in
  unterschiedlichen Schichten (Regelungstheorie, Numerik, Spieldesign) versteckte Annahmen
  hatten, die erst durch Messung/Verifikation gegen die reale FMU sichtbar wurden — das als
  gemeinsames methodisches Muster herausarbeiten
- Kurzer Bezug zum Stand der Technik (Kapitel 1.2): eigene Ergebnisse einordnen — z. B.
  PD-Instabilitätsbefund als konkretes Beispiel für die in der Literatur bekannte
  Notwendigkeit vollständiger Zustandsrückführung bei unteraktuierten Systemen

**Abbildungsvorschlag:**
- *Abbildung 7.1* (optional, zusammenfassend): eine kombinierte Übersichtsgrafik/Tabelle,
  die die wichtigsten Kennzahlen aus Kapitel 3/5/6 auf einer Seite zusammenfasst — als
  visueller Abschluss vor dem Fazit

## 7.3 Grenzen der Arbeit

- Ehrlich benennen: keine automatisierten Tests für Pygame-Rendering/Event-Loop (nur
  manuelles Playtesting), Mutual-Exclusion-Invariante hat keine automatisierte
  Regressionsabsicherung, SwingUp-Überschwing-Fangproblem nicht analysiert (siehe Kapitel 8)

**Quellen:** keine neue Primärquelle — Synthese aus Kapiteln 3–6 und `CLAUDE.md`
(Conventions-Abschnitt für projektweite Entscheidungen); ggf. Rückgriff auf die
Stand-der-Technik-Quellen aus Kapitel 1.2 für die Einordnung

---

# Kapitel 8: Fazit und Ausblick

## 8.1 Zusammenfassung

- Zielerreichung gegen Projektanweisung, AP-für-AP kurz (AP1–AP4 abgeschlossen, AP5 diese
  Arbeit, AP6 Vortrag steht noch aus) — 3–4 Sätze als Stichpunkte reichen, keine erneute
  Detailwiederholung

## 8.2 Ausblick

- Offene Backlog-Punkte, jeweils kurz mit Ursache/Status:
  - SwingUp-Regler fängt nach Überschwingen nicht (vermuteter Hysterese-Grenzfall bei hoher
    Winkelgeschwindigkeit im Überschwing-Moment, noch nicht analysiert — konkreter nächster
    Schritt: Hysterese-Übergangsbedingung um eine Geschwindigkeits-/Richtungsprüfung
    erweitern)
  - Weitere Variationsachsen für Schwierigkeitsgrade denkbar (z. B. Pendellänge $l$) —
    bräuchte einen erneuten `.mo`-Export, da $l$ aktuell kein FMU-Parameter im gleichen
    Sinne wie Masse/Reibung ist
  - Keine automatisierte Regressionsabsicherung der Mutual-Exclusion-Invariante (lebt im
    ungetesteten Pygame-Event-Loop) — als konkreter Vorschlag: reine, testbare Prädikat-
    Funktionen extrahieren
  - CVODE/WSL-Solver-Vergleich: bereits abgeschlossen (2026-08-21) — als erledigte
    Untersuchung erwähnen, nicht als offenen Punkt formulieren
- Kurzer, persönlicher Abschlusssatz zum methodischen Vorgehen (z. B. Wert von Messung
  gegen die reale FMU statt reiner Theorie/Intuition — durchgängiges Muster der ganzen
  Arbeit) — das ist der einzige Abschnitt der gesamten Arbeit, der bewusst etwas freier/
  reflektierender formuliert sein darf

**Quellen (intern):** `CLAUDE.md` Abschnitt "Ideas / Backlog"

---

# Literaturverzeichnis — Sammelübersicht aller externen Quellen

Zum Kopieren in die `.bib`-Datei bzw. das Literaturverzeichnis (Details/DOI siehe jeweiliges
Unterkapitel oben):

1. Åström, K. J.; Furuta, K. (2000): *Swinging up a Pendulum by Energy Control*. Automatica,
   36(2), 278–285. DOI: 10.1016/S0005-1098(99)00140-5. — Kapitel 5.3, 5.6
2. Anderson, B. D. O.; Moore, J. B. (1990): *Optimal Control: Linear Quadratic Methods*.
   Prentice Hall. — Kapitel 2.4, 5.2
3. Boubaker, O. (2013): *The Inverted Pendulum Benchmark in Nonlinear Control Theory: A
   Survey*. International Journal of Advanced Robotic Systems, 10(5), 233. — Kapitel 1.2
4. Fritzson, P. (2014): *Principles of Object-Oriented Modeling and Simulation with Modelica
   3.3: A Cyber-Physical Approach*. Wiley. — Kapitel 3.1, 3.5
5. Lundberg, K. H.; Barton, T. S. (2010): *History of Inverted-Pendulum Systems*. IFAC
   Proceedings Volumes, 42(24), 131–135. DOI: 10.3182/20091021-3-JP-2009.00025. — Kapitel 1.2
6. Ogata, K. (2010): *Modern Control Engineering*, 5. Auflage. Pearson. — Kapitel 2.4, 5.2,
   5.3

**Hinweis:** Alle sechs Quellen wurden für dieses Dokument per Websuche auf Existenz und
Zitierdetails (Autoren, Jahr, Journal/Verlag, DOI wo verfügbar) geprüft, aber nicht im
Volltext gelesen — vor dem Zitieren im eigentlichen Ausarbeitungstext unbedingt selbst
zumindest überfliegen, damit die Aussage, die du daran aufhängst, wirklich durch die Quelle
gedeckt ist.
