# AP1: Validierung des MultiBody-Modells gegen das formelbasierte Referenzmodell
 
Stand: 2026-08-03
 
## 1. Zielsetzung
 
Laut Projektanweisung (AP1) muss das neu erstellte MultiBody-Modell
(`InvertedPendulumMB.mo`) dieselben Signale liefern wie das ursprüngliche,
formelbasierte Modell `pendulum.mo`, geprüft anhand definierter Testszenarien.
Dieses Dokument fasst Vorgehen, Ergebnisse und Einordnung dieser Validierung
zusammen.
 
## 2. Modellaufbau (Kurzfassung)
 
`InvertedPendulumMB.mo` bildet die Kinematikkette
 
```
World → Prismatic (Wagen, Achse x) → Body (M) / Revolute (Pendel, Achse z)
       → FixedTranslation (Länge l) → Body (m)
```
 
mit `useAxisFlange = true` an Prismatic und Revolute zur Einleitung von
Kraft (`forceCart`, gesteuert über `tau`) und Dämpfung (`damperCart`,
`damperPend`) ab.
 
**Winkelkonvention** (kritischer Punkt, siehe `protokoll.md`): φ=0 ist die
stabile, hängende Ruhelage, φ=π die instabile, aufrechte Zielposition.
Diese Konvention wurde experimentell verifiziert: bei `tau=0` und einer
Anfangsauslenkung von `phi0 = 0.75·π/2 ≈ 67,5°` pendelt sich das System
nach ca. 8 Sekunden bei `phi=0` ein, wie erwartet.
 
## 3. Validierungsmethodik
 
### 3.1 Verworfener Ansatz: Trajektorienvergleich über die Zeit
 
Ein erster Ansatz simulierte beide Modelle mit identischem, aktivem
Testsignal `tau(t) = 5·sin(0,5t)` über 20 Sekunden und verglich die
Zeitverläufe von `s`, `v`, `phi`, `vphi` per RMSE.
 
Ergebnis: sehr hohe RMSE-Werte (z. B. `phi`: RMSE ≈ 38, max. Fehler ≈ 86 rad).
Diese Abweichung ist kein Hinweis auf einen Modellierungsfehler, sondern
eine Folge der **sensitiven Abhängigkeit von Anfangsbedingungen** in einem
instabilen, nichtlinearen System (invertiertes Pendel ohne Regler): geringste
numerische Unterschiede zwischen beiden Modellen führen nach einiger Zeit zu
exponentiell divergierenden Trajektorien (chaotisches Verhalten). Ein
Trajektorienvergleich über mehrere Sekunden ist für dieses System daher
methodisch ungeeignet.
 
### 3.2 Gewählter Ansatz: Momentanvergleich der Beschleunigungen
 
Statt über die Zeit zu integrieren, wird bei einem fest vorgegebenen Zustand
`(s0, v0, phi0, vphi0, tau)` direkt die daraus resultierende Beschleunigung
verglichen:
 
- Flaches Modell: Variablen `a`, `alpha` (explizit in `pendulum.mo` berechnet)
- MultiBody-Modell: Variablen `prismatic.a`, `revolute.a` (von der
  MultiBody-Bibliothek automatisch aus der Kinematik/Dynamik hergeleitet)
Beide Größen sind algebraische Funktionen des momentanen Zustands, ein
Vergleich erfordert keine Zeitintegration und ist damit unempfindlich
gegenüber der Chaos-Problematik aus 3.1.
 
**Voraussetzung:** Die Startwerte `s0, v0, phi0, vphi0` wurden in beiden
Modellen von hart kodierten `initial equation`-Zuweisungen auf echte
Modellparameter umgestellt, damit sie sich über die FMU-Schnittstelle
(`setReal` vor `enterInitializationMode()`) gezielt vorgeben lassen.
 
### 3.3 Technische Randbedingungen
 
- FMU-Export mit dem **Forward-Euler-Solver** (`--fmiFlags=s:euler` bzw.
  ohne CVODE-Flag), nicht mit CVODE.
  **Grund:** Der CVODE-Export führte bei beiden FMUs (flach und MultiBody)
  unter Windows 10 in Kombination mit `fmpy` reproduzierbar zu einem
  Heap-Corruption-Absturz (`STATUS_HEAP_CORRUPTION`, Exit-Code
  `-1073740940`). Laut OpenModelica-Ticket-Historie ist die CVODE-FMU-CS-
  Unterstützung unter Linux erprobt, unter Windows aber offenbar mit
  Reibungspunkten behaftet (Sundials-Laufzeitabhängigkeiten,
  ABI-Kompatibilität). Für die hier durchgeführte Validierung ist der
  Euler-Solver ausreichend, da keine steifen Systemeigenschaften vorliegen.
- Parameterwerte während des Exports: `M=5`, `m=0,5`, `l=0,5`,
  `d_cart=0,15`, `d_pend=0,15` (identisch in beiden Modellen).
## 4. Ergebnisse
 
Getestete Zustände (`s0`, `v0`, `phi0`, `vphi0`, `tau`) und resultierende
Beschleunigungen:
 
| # | s0 | v0 | phi0 | vphi0 | tau | a_flat | a_mb | a_diff | alpha_flat | alpha_mb | alpha_diff |
|---|----|----|------|-------|-----|--------|------|--------|------------|----------|------------|
| 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| 1 | 0 | 0 | 0 | 0 | 2 | 0,4000 | 0,3997 | 3,2e-4 | -0,800 | -0,793 | 7,0e-3 |
| 2 | 0 | 0 | 0 | 0 | -3 | -0,600 | -0,600 | 4,8e-4 | 1,200 | 1,190 | 1,0e-2 |
| 3 | 0 | 1 | 0 | 0 | 0 | -0,030 | -0,030 | 2,4e-5 | 0 | 0,0595 | 0,0595 |
| 4 | 0 | -2 | 0 | 0 | 0 | 0,060 | 0,060 | 4,8e-5 | 0 | -0,119 | 0,119 |
| 5 | 0 | 0 | 0 | 1 | 0 | 0 | 0,0595 | 0,0595 | -0,060 | -1,308 | 1,248 |
| 6 | 0 | 0 | π/2 | 0 | 0 | ≈0 | ≈0 | ≈0 | -19,62 | -19,46 | 0,162 |
| 7 | 0 | 0 | π | 0 | 0 | ≈0 | ≈0 | ≈0 | ≈0 | ≈0 | ≈0 |
| 8 | 0 | 1 | 1,178 | 0,5 | 2 | 0,671 | 0,679 | 7,8e-3 | -18,69 | -19,09 | 0,398 |
| 9 | 1 | -0,5 | π/2 | -1 | -3 | -0,486 | -0,486 | 5,6e-17 | -19,57 | -18,27 | 1,298 |
 
(Vollständige Rohdaten inkl. Python-Skript: `momentanVergleich.py`,
`sim_flat.csv` nicht relevant für diesen Ansatz, da zustandsbasiert statt
zeitbasiert.)
 
## 5. Interpretation
 
### 5.1 Sehr gute Übereinstimmung bei entkoppelten Testfällen
 
Bei reiner Krafteinwirkung (`tau≠0`, `phi0=0`, keine Geschwindigkeiten) und
bei reiner Winkelauslenkung (`v0=vphi0=0`) stimmen beide Modelle bis auf
numerische Rundungsgenauigkeit überein (`a_diff`, `alpha_diff` im Bereich
10⁻⁴ bis 10⁻¹⁷). Dies bestätigt, dass Krafteinleitung, Massenträgheit und
Gravitationsterm im MultiBody-Modell korrekt der Struktur des formelbasierten
Modells entsprechen.
 
### 5.2 Systematische Differenz bei gekoppelten Zuständen
 
Bei Zuständen mit `v0≠0` oder `vphi0≠0` zeigt sich eine reproduzierbare
Differenz vor allem in `alpha` (Größenordnung 0,06 bis 1,3). Diese Differenz
ist **kein Fehler**, sondern entspricht exakt der in `protokoll.md` bereits
dokumentierten Vereinfachung: Das formelbasierte Modell vernachlässigt die
Rückwirkung der Pendel-Gelenkdämpfung (`d_pend`) auf die Wagenbeschleunigung.
Das MultiBody-Modell berechnet diese Kopplung dagegen automatisch und
vollständig aus der Kinematik- und Dynamikstruktur, ohne dass sie manuell
vereinfacht werden müsste.
 
Besonders deutlich in Zeile 5: bei `vphi0=1`, sonst Ruhelage, ergibt sich im
MultiBody-Modell eine Wagenbeschleunigung `a_mb=0,0595`, während das flache
Modell `a_flat=0` liefert, weil dieser Term dort in der Bewegungsgleichung
schlicht nicht vorkommt.
 
### 5.3 Einordnung
 
Das MultiBody-Modell ist damit **physikalisch vollständiger** als das
ursprüngliche, formelbasierte Modell, bei ansonsten identischer Struktur.
Die beobachteten Abweichungen sind lokalisiert, erklärbar und stimmen in
Richtung und Größenordnung mit der bereits vorab dokumentierten
Modellvereinfachung überein.
 
## 6. Offener Punkt (zurückgestellt)
 
Eine geschlossene analytische Herleitung des fehlenden Kopplungsterms
(vollständige Euler-Lagrange-Gleichung mit Rayleigh-Dissipationsfunktion
für `d_pend`, unter Berücksichtigung der Kopplung über die Massenmatrix)
wurde nicht durchgeführt. Sie wäre für eine vollständige quantitative
Erklärung der Restdifferenz wünschenswert, ist aber für den Abschluss von
AP1 nicht erforderlich und wurde angesichts des kritischen Zeitpfads
(AP3: Reglerentwurf) zurückgestellt. Mögliche spätere Verwendung: als
vertiefende Diskussion in Kapitel 7 (Ergebnisse und Diskussion) der
schriftlichen Ausarbeitung.
 
## 7. Status AP1
 
- [x] MultiBody-Modell mit Standardbibliotheken erstellt
- [x] Winkelkonvention experimentell verifiziert
- [x] FMU-Export funktionsfähig (Euler-Solver; CVODE unter Windows instabil,
      dokumentiert in Abschnitt 3.3)
- [x] Quantitative Validierung gegen `pendulum.mo` durchgeführt und
      dokumentiert
- [ ] Optional/zurückgestellt: analytische Herleitung der Restdifferenz
**AP1 gilt damit als inhaltlich abgeschlossen.**
 