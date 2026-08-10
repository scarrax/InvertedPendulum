# Design: AP3 (Teil 1) — LQR-Regler

Stand: 2026-08-06

## 1. Kontext und Ziel

Laut [Projektanweisung_Invertiertes_Pendel.md](../../../Projektanweisung_Invertiertes_Pendel.md)
(AP3) sollen weitere Reglerklassen nach dem bestehenden `Controller`-Interface
implementiert werden, unter anderem ein LQR mit vollem Zustand
`(s, v, θ, θ̇)`. AP3 ist als Ganzes zu groß für einen Plan (LQR,
energiebasierter Swing-up, optional weitere Regler, plus ein Vergleich der
Regler) — dieses Dokument deckt nur den ersten Teilschritt ab: den
LQR-Regler selbst, isoliert getestet und interaktiv verifizierbar. Swing-up
und der formale Reglervergleich sind eigene, spätere Designs.

`SimpleController` bleibt gemäß `CLAUDE.md`-Konvention unverändert als
Referenzimplementierung erhalten; die `Controller`-Basisklasse wird nicht
verändert.

**Wichtiger Kontext:** Das Spiel läuft seit AP2 gegen `InvertedPendulumMB.fmu`.
Die für den LQR nötige Linearisierung basiert daher nicht auf den
vereinfachten Gleichungen in `pendulum.mo`, sondern auf den in
[AP1_Validierung.md, Abschnitt 6](../../../AP1_Validierung.md#6-analytische-herleitung-des-kopplungsterms)
hergeleiteten, vollständig gekoppelten Bewegungsgleichungen — diese stimmen
mit dem MultiBody-Modell bis auf ~1% überein, während `pendulum.mo`s
Gleichungen den Dämpfungskoeffizienten in der `α`-Gleichung um den Faktor
`(M+m)/(ml) ≈ 22` unterschätzen.

## 2. Entscheidungen (aus Brainstorming-Dialog)

| Frage | Entscheidung |
|---|---|
| AP3-Startpunkt? | LQR zuerst, Swing-up als eigenes späteres Teilstück |
| Linearisierungsbasis? | Korrigierte, gekoppelte Gleichungen (AP1_Validierung.md §6), nicht das flache Modell |
| Wagenziel? | `s = 0` (Schienenmitte) wird mitgeregelt |
| Vergleichsumfang? | Nur LQR selbst diese Runde, formaler Vergleich zu SimpleController später |
| Regler-Auswahl im Spiel? | Neue Taste `L` schaltet SimpleController/LQRController um |
| Verhalten außerhalb Einzugsbereich? | `τ` wird geclampt wie bei SimpleController, keine Sonderlogik |
| Gain-Berechnung? | Zur Laufzeit via `scipy.linalg.solve_continuous_are`, nicht offline vorberechnet |

## 3. Zustandsraummodell (Linearisierung um φ=π)

Ausgangspunkt sind die nichtlinearen, gekoppelten Gleichungen aus
`AP1_Validierung.md` §6.2. Mit `φ = π + θ` (kleines `θ`), `sinφ ≈ −θ`,
`cosφ ≈ −1`, Zustand `x = [s, v, θ, θ̇]`, Eingang `u = τ`:

```
ṡ  = v
v̇  = (m·g/M)·θ − (d_cart/M)·v − (d_pend/(M·l))·θ̇ + (1/M)·τ
θ̇  = θ̇
θ̈  = ((M+m)·g/(l·M))·θ − (d_cart/(l·M))·v − ((M+m)·d_pend/(m·l²·M))·θ̇ + (1/(l·M))·τ
```

Mit `M=5, m=0.5, l=0.5, g=9.81, d_cart=0.15, d_pend=0.15` (identisch zu
`InvertedPendulumMB.mo`):

```
A = [[0,     1,       0,      0    ],
     [0,    -0.03,    0.981, -0.06 ],
     [0,     0,       0,      1    ],
     [0,    -0.06,   21.582, -1.32 ]]

B = [0, 0.2, 0, 0.4]^T
```

Die `-1.32`-Komponente entspricht exakt dem in AP1_Validierung.md §6.4
numerisch validierten Dämpfungskoeffizienten (dort gemessen: `-1.308`).

## 4. Gain-Berechnung

```python
def compute_lqr_gain(M, m, l, g, d_cart, d_pend, Q, R):
    A = np.array([...])  # wie oben, parametrisch in M/m/l/g/d_cart/d_pend
    B = np.array([...])
    P = scipy.linalg.solve_continuous_are(A, B, Q, R)
    K = np.linalg.inv(R) @ B.T @ P
    return K  # shape (1, 4)
```

Reine Funktion, kein Pygame-/FMU-Bezug. `Q`, `R` sind Klassenkonstanten von
`LQRController` (Startwerte während der Implementierung festlegen, z. B.
`Q = diag(1, 1, 10, 1)`, `R = [[1]]` — Gewichtung auf `θ` deutlich höher als
auf `s`/`v`, da Balance Priorität vor Zentrierung hat; Feintuning Teil der
Implementierung).

## 5. Reglerklasse & Spiel-Integration

```python
class LQRController(Controller):
    Q = ...
    R = ...

    def __init__(self):
        self.K = compute_lqr_gain(M=5, m=0.5, l=0.5, g=9.81,
                                   d_cart=0.15, d_pend=0.15,
                                   Q=self.Q, R=self.R)

    def compute(self, phi_fmu, vphi, s, v):
        theta = (phi_fmu % (2 * math.pi)) - math.pi
        x = np.array([s, v, theta, vphi])
        tau = float(-self.K @ x)
        return max(-MAX_TAU, min(MAX_TAU, tau))
```

`K` wird einmalig in `__init__` berechnet, nicht pro Frame.

**Integration in `run_game()`:** beide Controller-Instanzen (`SimpleController()`,
`LQRController()`) werden zu Rundenbeginn erzeugt; eine Referenz auf den
aktuell aktiven Auto-Regler wird bei `K_l`-Tastendruck (analog zu `K_h`/`K_p`/
`K_r`) umgeschaltet. `L` funktioniert unabhängig vom Auto/Manuell-Zustand
(auch im manuellen Modus drückbar) und bestimmt nur, welcher Regler beim
nächsten bzw. aktuellen Aufenthalt im Auto-Modus verwendet wird — `H` bleibt
ausschließlich für den Auto/Manuell-Wechsel selbst zuständig. `redraw()`
bekommt einen neuen Parameter (Reglername), zeigt ein kleines Badge analog
zum bestehenden AUTO/MANUAL-Badge (z. B. "LQR" / "SimpleController").

## 6. Testing

`tests/test_lqr_controller.py`:

- `compute_lqr_gain(...)` liefert ein `K`, für das alle Eigenwerte von
  `A − B·K` negativen Realteil haben (`np.linalg.eigvals`,
  `assert (eigenvalues.real < 0).all()`) — echter Stabilitätsnachweis.
- `LQRController().compute(phi_fmu=math.pi, vphi=0, s=0, v=0)` liegt nahe 0
  (Gleichgewicht braucht keine Stellgröße).
- Clamping-Test: bei stark ausgelenktem Zustand bleibt `tau` innerhalb
  `[-MAX_TAU, MAX_TAU]`.

Interaktive Verifikation (Taste `L`, Pendel von Hand nah an `φ=π` bringen,
Balance und Rezentrierung auf `s=0` beobachten) bleibt — wie bei allen
Pygame-/FMU-abhängigen Teilen seit AP2 — Aufgabe eines Menschen mit echtem
Display.

## 7. Dependencies

Neues `requirements.txt` im Projektroot: `pygame`, `fmpy`, `pandas`,
`pytest`, `scipy`, `numpy`. Das Projekt hatte bisher keine getrackten
Abhängigkeiten; `scipy`/`numpy` sind die ersten, die das erfordern.

## 8. Nicht Teil dieses Designs

- Energiebasierter Swing-up-Controller — eigenes, späteres AP3-Teilstück.
  Der LQR liefert außerhalb eines kleinen Einzugsbereichs um `φ=π` kein
  sinnvolles Verhalten; das ist erwartet und wird hier nicht kompensiert.
- Formaler Reglervergleich (Stabilität, Reaktionszeit, Robustheit) zwischen
  `SimpleController` und `LQRController` — eigenes, späteres AP3-Teilstück,
  sobald mehr als zwei Regler existieren.
- Pole Placement, nichtlinearer oder lernbasierter Regler (laut
  Projektanweisung optional) — nicht Teil dieses Designs.
- Automatisches Deaktivieren/Umschalten des LQR außerhalb seines
  Einzugsbereichs — bewusst nicht Teil dieses Designs (siehe
  Brainstorming-Entscheidung).
