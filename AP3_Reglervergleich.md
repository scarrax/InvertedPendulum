# AP3 Reglervergleich

## 1. Kontext

Vergleich der drei AP3-Regler (PD/SimpleController, LQR, SwingUp) gegen die reale InvertedPendulumMB.fmu, gefordert durch die Projektanweisung (AP3: 'Vergleich der Regler hinsichtlich Stabilitaet, Reaktionszeit und Robustheit gegenueber Stoerungen'). Details zur Methodik in docs/superpowers/specs/2026-08-17-ap3-controller-comparison-design.md.

## 2. Methodik

Erfolgskriterium: |theta| < 5° fuer mindestens 1.0s ununterbrochen (theta = Abweichung von der aufrechten Lage phi=pi). Stabilitaet wird als groesste erfolgreiche Anfangsauslenkung im 2°-Sweep gemessen (PD/LQR), Reaktionszeit als Einschwingzeit bei festen Baseline-Auslenkungen (2°, 10°), Robustheit als Erholungszeit nach einem Kraft-Puls auf tau (KICK_TAU=8.0 fuer KICK_STEPS=5 Frames) waehrend des eingeschwungenen Zustands, und die Swing-up-Faehigkeit als Einschwingzeit ab der realen Spiel-Anfangsbedingung (phi0=0.75*pi/2).

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

## 5. Robustheit (Kraft-Puls)

| Regler | Erholungszeit |
|---|---|
| PD | Regler hat vor dem Puls nicht eingeschwungen |
| LQR | 0.00s |
| SwingUp | 0.00s |

## 6. Swing-up ab realer Anfangsbedingung

SwingUp: 4.20s
PD/LQR: N/A (strukturell nicht loesbar, siehe Design-Dokument §3.4)

## 7. Diskussion

Die drei Regler unterscheiden sich strukturell in ihrem Einzugsbereich (Tabelle oben): PD reagiert nur lokal um phi=pi, LQR nutzt vollen Zustand (s, v, theta, theta_dot) und deckt typischerweise einen groesseren Bereich ab, aber beide sind auf eine Linearisierung um die aufrechte Lage angewiesen und koennen die reale Spiel-Anfangsbedingung (~112.5° von der aufrechten Lage) strukturell nicht erreichen (Abschnitt 6) - genau der in der Projektanweisung genannte Vergleichspunkt fuer den energiebasierten SwingUp-Regler (siehe AP1_Validierung.md §6 fuer die zugrundeliegende Physik). Bei der Robustheit (Abschnitt 5) zeigt die Erholungszeit nach dem Kraft-Puls, welcher Regler eine Stoerung am schnellsten wieder ausregelt; ein Regler, der vor dem Puls gar nicht erst eingeschwungen war, wird als 'nicht eingeschwungen' statt mit einer Erholungszeit gefuehrt und ist entsprechend gesondert zu lesen.
