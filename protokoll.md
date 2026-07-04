# Protokoll: Inverted Pendulum Projekt

Stand: 2026-07-04

## 1. Projektüberblick

Das Projekt implementiert ein **Inverted-Pendulum-Spiel/Simulation** bestehend aus einem
Modelica-Physikmodell und einem Python/Pygame-Frontend.

### Dateien

| Datei | Zweck |
|---|---|
| [pendulum.mo](pendulum.mo) | Modelica-Modell des invertierten Pendels (Wagen + Stab), flache Bewegungsgleichungen. Eingang `tau` (Kraft auf Wagen), Ausgänge `s`, `v`, `phi`, `vphi`. |
| [export.mos](export.mos) | OpenModelica-Skript, das aus `pendulum.mo` eine FMU (Co-Simulation, CVODE-Solver) baut (`InvertedPendulum.fmu`). Muss vor dem Spiel im Projektroot liegen. |
| [pendulum_game_controlled.py](pendulum_game_controlled.py) | Das eigentliche Spiel: lädt die FMU via `fmpy`, simuliert in Echtzeit (dt=0.02s, 40s Spieldauer), visualisiert mit Pygame (Wagen, Pendel, Tachometer, Plots für φ(t) und τ(t)). |
| [leaderboard.csv](leaderboard.csv) | Persistente Bestenliste (Datum, Zeit, Name, Score). |

### Steuerung / Spielablauf

- Pfeiltasten: manuelle Steuerung der Wagenkraft (`tau = ±MAX_TAU`).
- Taste `H`: Umschalten zwischen manuellem und Auto-Modus.
- Im Auto-Modus übernimmt `SimpleController` (PD-Regler) die Steuerung.
- Scoring: Bonus für geringe Winkelabweichung von der aufrechten Position, gestaffelt in
  drei Zonen (grobe Toleranz, 15°-Zone, 5°-Zone).
- Nach Spielende: Namenseingabe und Eintrag in `leaderboard.csv`, Anzeige der Top 10 vor
  dem nächsten Spiel.

### Architektur-Hinweis

`Controller` ist eine abstrakte Basisklasse ([pendulum_game_controlled.py:264](pendulum_game_controlled.py#L264)),
`SimpleController` die einzige aktuelle Implementierung. Ein TODO-Kommentar
([pendulum_game_controlled.py:285](pendulum_game_controlled.py#L285)) deutet an, dass weitere Regler
(z. B. LQR oder ein energiebasierter Swing-up-Controller) folgen sollen.

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

Über die Euler-Lagrange-Gleichungen (mit Reibung $d_{cart}v$ am Wagen und Dämpfung
$d_{pend}\dot\varphi$ am Gelenk als generalisierte Kräfte) ergibt sich nach Auflösen des
2×2-Gleichungssystems in $a = \dot v$ und $\alpha = \ddot\varphi$:

$$a = \frac{\tau - d_{cart}v + m\sin\varphi\,(l\dot\varphi^2 + g\cos\varphi)}{M + m\sin^2\varphi}$$

$$\alpha = \frac{-\tau\cos\varphi - m l\dot\varphi^2\sin\varphi\cos\varphi - (M+m)g\sin\varphi - d_{pend}\dot\varphi}{l\,(M + m\sin^2\varphi)}$$

Das entspricht (bis auf einen kleinen, im Modell vernachlässigten Kopplungsterm — die
Rückwirkung der Gelenkdämpfung $d_{pend}$ auf die Wagenbeschleunigung) exakt der Struktur
in [pendulum.mo:31-33](pendulum.mo#L31-L33). Nachgerechnet und verifiziert.

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

`SimpleController` ([pendulum_game_controlled.py:270-283](pendulum_game_controlled.py#L270-L283))
ist im Kern ein **PD-Regler auf Winkel und Winkelgeschwindigkeit**:

$$\tau = -(K_\varphi\,\theta + K_{\dot\varphi}\,\dot\varphi)$$

Das ist eine reduzierte Version einer vollständigen Zustandsrückführung (wie sie z. B.
LQR liefern würde): eine echte LQR-Lösung würde zusätzlich $s$ und $v$ gewichten, um den
Wagen z. B. mittig auf der Schiene zu halten. Der aktuelle Regler ignoriert
Wagenposition/-geschwindigkeit komplett — er balanciert das Pendel, lässt den Wagen aber
ggf. an den Rand der Schiene driften.

Mögliche Erweiterungen (Ansatzpunkt für das TODO im Code):

- **LQR / Pole Placement** mit vollem Zustand $(s, v, \theta, \dot\theta)$, um auch die
  Wagenposition zu regeln.
- **Energiebasierter Swing-up-Controller** für den Fall, dass das Pendel weit von der
  aufrechten Lage startet (relevant, da die Modell-Anfangsbedingung
  $\varphi = 0.75\cdot\pi/2 \approx 67{,}5^\circ$ in [pendulum.mo:24](pendulum.mo#L24)
  noch nicht in der Nähe der aufrechten Lage liegt).

---

## 3. Offene Punkte / nächste Schritte

- Weitere Regler-Implementierungen (LQR, Swing-up) gemäß TODO in
  [pendulum_game_controlled.py:285](pendulum_game_controlled.py#L285).
- Vergleich der flachen Modelica-Gleichungen mit einer Standardbibliotheks-Variante
  (`Modelica.Mechanics.MultiBody.Examples.Elementary.Pendulum`), siehe Kommentar in
  [pendulum.mo:2](pendulum.mo#L2).
