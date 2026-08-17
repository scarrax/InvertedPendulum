# AP3 Reglervergleich

## 1. Kontext

Vergleich der drei AP3-Regler (PD/SimpleController, LQR, SwingUp) gegen die reale InvertedPendulumMB.fmu, gefordert durch die Projektanweisung (AP3: 'Vergleich der Regler hinsichtlich Stabilitaet, Reaktionszeit und Robustheit gegenueber Stoerungen'). Details zur Methodik in docs/superpowers/specs/2026-08-17-ap3-controller-comparison-design.md.

## 2. Methodik

Erfolgskriterium: |theta| < 5° fuer mindestens 1.0s ununterbrochen (theta = Abweichung von der aufrechten Lage phi=pi). Stabilitaet wird als groesste erfolgreiche Anfangsauslenkung in einem Sweep von 2° bis 90° in 2°-Schritten gemessen (PD/LQR; 2° ist damit zugleich die kleinste getestete Auslenkung, nicht nur die Schrittweite), Reaktionszeit als Einschwingzeit bei festen Baseline-Auslenkungen (2°, 10°), Robustheit als Erholungszeit und maximale Auslenkung nach einem Kraft-Puls auf tau (KICK_TAU=8.0 fuer KICK_STEPS=5 Frames) waehrend des eingeschwungenen Zustands, und die Swing-up-Faehigkeit als Einschwingzeit ab der realen Spiel-Anfangsbedingung (phi0=0.75*pi/2).

## 3. Stabilitaet (Einzugsbereich)

| Regler | Einzugsbereich |
|---|---|
| PD | kein Erfolg im gesweepten Bereich |
| LQR | 10° |
| SwingUp | siehe Swing-up-Ergebnis unten |

## 4. Reaktionszeit

| Regler | 2° | 10° |
|---|---|---|
| PD | kein Einschwingen | kein Einschwingen |
| LQR | 0.00s | 0.70s |
| SwingUp | 0.00s | 0.70s |

Hinweis zu '0.00s': Gemessen wird der fruehste Zeitpunkt, ab dem |theta| ununterbrochen 1.0s lang unter 5° bleibt. Die Baseline 2° liegt bereits innerhalb dieser Toleranzzone - ein Regler, der sie nie verlaesst, erhaelt damit per Definition des Erfolgskriteriums 0.00s, was *nicht* 'sofortige Reaktion' bedeutet. Dasselbe gilt fuer die Erholungszeiten in Abschnitt 5: bleibt die Auslenkung nach dem Kraft-Puls durchgehend unter 5°, ist die Erholungszeit ebenfalls 0.00s - wie stark der Puls tatsaechlich gestoert hat, zeigt dort erst die Spalte 'Max. Auslenkung nach Puls'.

## 5. Robustheit (Kraft-Puls)

| Regler | Erholungszeit | Max. Auslenkung nach Puls |
|---|---|---|
| PD | Regler hat vor dem Puls nicht eingeschwungen | kein Puls ausgeloest |
| LQR | 0.00s | 1.02° |
| SwingUp | 0.00s | 1.02° |

## 6. Swing-up ab realer Anfangsbedingung

SwingUp: 4.20s
PD/LQR: N/A (strukturell nicht loesbar, siehe Design-Dokument §3.4)

## 7. Diskussion

**PD (SimpleController) ist um die aufrechte Lage nicht asymptotisch stabilisierend.** Das ist kein gradueller Unterschied zu LQR, sondern ein struktureller: eine Eigenwertanalyse der geschlossenen Regelschleife (A - B*K mit K_pd = [0, 0, K_PHI, K_VPHI] entsprechend SimpleController.compute) um phi=pi liefert einen Eigenwert mit positivem Realteil (~1.15 beim frueheren d_pend=0.15, ~1.28 beim aktuellen d_pend=0.01) - die aufrechte Lage bleibt unter PD instabil, der Regler ist dort also nicht 'nur schwach gedaempft' und hat auch keinen kleinen, aber positiven Einzugsbereich im Sinne asymptotischer Stabilitaet. Dass PD das Erfolgskriterium bei sehr kleinen Anfangsauslenkungen (empirisch bis ~1.5°) trotzdem erfuellt, liegt allein daran, dass die Divergenz von einem so kleinen Start aus laenger als das 1.0s-Haltefenster braucht, um die 5°-Toleranzgrenze zu ueberschreiten. Der Sweep in Abschnitt 3 *beginnt* bei 2° (2° ist Startwert und Schrittweite zugleich); 'kein Erfolg im gesweepten Bereich' fuer PD heisst deshalb, dass PDs reale Schwelle (~1.5-2°) unterhalb des ersten getesteten Punktes liegt - nicht, dass PD bei beliebig kleiner Auslenkung sofort versagt.

**LQR erreicht demgegenueber 10° (Abschnitt 3).** Der LQR nutzt den vollen Zustand (s, v, theta, theta_dot) statt nur phi/vphi und ist um phi=pi tatsaechlich asymptotisch stabilisierend, deshalb ein messbarer Einzugsbereich von 10° gegenueber PDs Schwelle unterhalb von 2°. Beide sind aber auf eine Linearisierung um die aufrechte Lage angewiesen und koennen die reale Spiel-Anfangsbedingung (~112.5° von der aufrechten Lage) strukturell nicht erreichen (Abschnitt 6) - genau der in der Projektanweisung genannte Vergleichspunkt fuer den energiebasierten SwingUp-Regler (siehe AP1_Validierung.md §6 fuer die zugrundeliegende Physik).

**Die SwingUp-Zeilen in den Abschnitten 4 und 5 sind kein eigenstaendiges Ergebnis.** SwingUpController schaltet per Hysterese (CAPTURE_THETA=10°, CAPTURE_VPHI=2.0) auf seinen internen LQRController um; beide Baselines (2°, 10°) liegen innerhalb dieses Fangbereichs, SwingUp delegiert dort also von Anfang an direkt an LQR. Dass seine Zahlen mit LQRs identisch sind, ist deshalb zu erwarten und weder Zufall noch Fehler - es sagt aber auch nichts ueber SwingUp selbst aus. Das unterscheidende Verhalten des SwingUp-Reglers zeigt sich erst in Abschnitt 6, im eigentlichen Swing-up ab der realen Anfangsbedingung.

**Zur Robustheit (Abschnitt 5): die Erholungszeiten sind nur zusammen mit der maximalen Auslenkung zu lesen.** Der Kraft-Puls (KICK_TAU=8.0 fuer 5 Frames) lenkt das Pendel gegen die eingeschwungenen Regler nur minimal aus (Maximalauslenkung: LQR 1.02°, SwingUp 1.02°) und bleibt damit deutlich innerhalb der 5°-Toleranzzone. Eine Erholungszeit von 0.00s bedeutet hier also 'die Stoerung hat die Toleranzzone nie verlassen', nicht 'sofort ausgeregelt'; der Puls ist fuer diese Regler zu schwach, um als Stoerung im Sinne des Kriteriums zu zaehlen. Ein Regler, der vor dem Puls gar nicht erst eingeschwungen war (PD, siehe oben), wird als 'nicht eingeschwungen' statt mit einer Erholungszeit gefuehrt und ist entsprechend gesondert zu lesen. Ein aussagekraeftigerer Robustheitsvergleich braeuchte einen staerkeren Puls oder eine Magnitudenvariation - bewusst ausserhalb des Scopes dieses Benchmarks (Design-Dokument §7).
