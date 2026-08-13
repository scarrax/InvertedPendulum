# Design: AP3 (Teil 2) — Swing-up-Regler mit automatischem LQR-Übergang

Stand: 2026-08-13

## 1. Kontext und Ziel

Laut [Projektanweisung_Invertiertes_Pendel.md](../../../Projektanweisung_Invertiertes_Pendel.md)
(AP3) ist ein energiebasierter Swing-up-Controller explizit gefordert,
"relevant weil die Modell-Anfangsbedingung (φ ≈ 67,5°) nicht in der Nähe der
aufrechten Lage liegt." AP3 Teil 1 (`docs/superpowers/specs/2026-08-06-lqr-controller-design.md`)
hat bereits `LQRController` geliefert, der aber nur innerhalb eines kleinen
Einzugsbereichs um `φ=π` (~11°, siehe AP3-Teil-1-Review) funktioniert — aus
der realen Startlage `phi0 = 0.75·π/2 ≈ 67,5°` (`InvertedPendulumMB.mo:10`)
kann er das Pendel nicht selbst hochbringen.

Dieses Design deckt den zweiten AP3-Teilschritt ab: einen energiebasierten
Swing-up-Regler, der das Pendel von einer beliebigen Startlage in den
LQR-Einzugsbereich bringt, und dort automatisch an den bestehenden LQR
übergibt (Hybrid-Regler). `SimpleController` und `LQRController` bleiben
unverändert (AP3-Konvention aus `CLAUDE.md`); die `Controller`-Basisklasse
wird nicht verändert.

**Entscheidungen aus dem Brainstorming-Dialog:**

| Frage | Entscheidung |
|---|---|
| Verhalten am oberen Punkt? | Automatischer Übergang zu LQR (kombinierter Hybrid-Regler) |
| UI-Integration? | Dritte Option im bestehenden `L`-Zyklus (PD → LQR → SwingUp → …) |
| Regelgesetz-Ansatz? | Klassisches Energie-Pumpen (Åström/Furuta), an φ=0(unten)/π(oben)-Konvention angepasst |
| τ-Umrechnung? | Exakte Inversion über die bereits validierten gekoppelten Gleichungen (AP1_Validierung.md §6.2), keine Näherung |

## 2. Physikalische Herleitung

Ausgangspunkt sind dieselben gekoppelten Bewegungsgleichungen wie beim LQR
(`AP1_Validierung.md` §6.1/§6.2):

```
(I)   (M+m)*a + m*l*cos(phi)*alpha = tau - d_cart*v + m*l*sin(phi)*vphi^2
(II)  m*l*cos(phi)*a + m*l^2*alpha = -m*g*l*sin(phi) - d_pend*vphi
```

### 2.1 Pendel-Energie

Mit `φ=0` als hängende Ruhelage (Referenzhöhe) und `φ=π` als aufrechte Lage:

```
E = ½·m·l²·φ̇² + m·g·l·(1 − cos φ)
```

`E(0,0) = 0`, `E(π,0) = E_top = 2·m·g·l` (Zielenergie).

### 2.2 Energie-Dynamik

Zeitableitung von `E`, mit (II) nach `α` aufgelöst eingesetzt:

```
dE/dt = m·l²·φ̇·α + m·g·l·sin(φ)·φ̇
      = φ̇ · [m·l²·α + m·g·l·sin(φ)]
      = φ̇ · [−d_pend·φ̇ − m·l·cos(φ)·a]
      = −d_pend·φ̇² − m·l·cos(φ)·φ̇·a
```

Der erste Term ist reine (kleine) Dissipation. Der zweite Term ist der über
die Wagenbeschleunigung `a` steuerbare Anteil.

### 2.3 Regelgesetz

Wahl: `a_cmd = K_ENERGY · (E − E_top) · sign(cos(φ)·φ̇)`. Einsetzen zeigt:

```
dE/dt ≈ −K_ENERGY·m·l·|cos(φ)·φ̇|·(E − E_top)   (Dissipationsterm vernachlässigt)
```

Das Vorzeichen von `dE/dt` ist damit garantiert entgegengesetzt zu
`(E − E_top)` — die Energie konvergiert monoton gegen `E_top`, unabhängig
vom genauen Wert von `K_ENERGY` (der nur die Konvergenzgeschwindigkeit
bestimmt). Dieses Regelgesetz ist robust gegenüber Modellfehlern, da es auf
der tatsächlich gemessenen Energie basiert, nicht auf einer Linearisierung.

### 2.4 Umrechnung auf τ (exakt, keine Näherung)

Aus (I)/(II) aufgelöst nach `a` (identisch zu `AP1_Validierung.md` §6.2):

```
a = [τ − d_cart·v + m·l·sin(φ)·φ̇² + m·g·sin(φ)·cos(φ)
     + (d_pend·cos(φ)/l)·φ̇] / (M + m·sin²φ)
```

Linear in `τ`, daher exakt nach `τ` für ein gewünschtes `a_cmd` auflösbar:

```
τ = a_cmd·(M + m·sin²φ) + d_cart·v − m·l·sin(φ)·φ̇²
    − m·g·sin(φ)·cos(φ) − (d_pend/l)·cos(φ)·φ̇
```

Anschließend wie bei allen Controllern auf `[−MAX_TAU, MAX_TAU]` geclampt.

## 3. Reglerklasse & Umschalt-Logik

```python
class SwingUpController(Controller):
    # Gleiche physikalische Konstanten wie LQRController (AP1_Validierung.md §6),
    # kein automatischer Sync — muss zu InvertedPendulumMB.mo passen.
    M = 5
    m = 0.5
    l = 0.5
    g = 9.81
    d_cart = 0.15
    d_pend = 0.15
    MAX_TAU = 10.0

    K_ENERGY = ...        # Tuning-Parameter, während Implementierung festlegen
    CAPTURE_THETA = math.radians(10)   # swingup -> lqr
    CAPTURE_VPHI = 2.0
    RELEASE_THETA = math.radians(25)   # lqr -> swingup (Hysterese)

    def __init__(self):
        self.lqr = LQRController()
        self.mode = "swingup"

    def compute(self, phi_fmu, vphi, s, v):
        theta = (phi_fmu % (2 * math.pi)) - math.pi

        if self.mode == "swingup" and abs(theta) < self.CAPTURE_THETA and abs(vphi) < self.CAPTURE_VPHI:
            self.mode = "lqr"
        elif self.mode == "lqr" and abs(theta) > self.RELEASE_THETA:
            self.mode = "swingup"

        if self.mode == "lqr":
            return self.lqr.compute(phi_fmu, vphi, s, v)

        # swing-up: Energie-Regelgesetz (Abschnitt 2.3/2.4)
        E = 0.5 * self.m * self.l**2 * vphi**2 + self.m * self.g * self.l * (1 - math.cos(phi_fmu))
        E_top = 2 * self.m * self.g * self.l
        sign = 1.0 if math.cos(phi_fmu) * vphi >= 0 else -1.0
        a_cmd = self.K_ENERGY * (E - E_top) * sign
        tau = (
            a_cmd * (self.M + self.m * math.sin(phi_fmu) ** 2)
            + self.d_cart * v
            - self.m * self.l * math.sin(phi_fmu) * vphi**2
            - self.m * self.g * math.sin(phi_fmu) * math.cos(phi_fmu)
            - (self.d_pend / self.l) * math.cos(phi_fmu) * vphi
        )
        return max(-self.MAX_TAU, min(self.MAX_TAU, tau))
```

`self.lqr` ist eine eigene `LQRController`-Instanz (Komposition, keine
Duplikation der Riccati-Lösung). `CAPTURE_THETA=10°` bleibt unter dem in der
AP3-Teil-1-Review bestätigten ~11°-Einzugsbereich; `RELEASE_THETA=25°` gibt
Hysterese-Puffer, damit kurze Störungen (z. B. ein Pfeiltasten-Stoß im
Auto-Modus) nicht sofort zurückschalten. Beide Schwellen sowie `K_ENERGY`
sind Tuning-Konstanten, während der Implementierung experimentell
festzulegen (analog zu `Q`/`R` beim LQR).

## 4. Integration in `run_game()`

Dritter Eintrag im bestehenden `controllers`-Dict:

```python
controllers = {"PD": SimpleController(), "LQR": LQRController(), "SwingUp": SwingUpController()}
```

Der bestehende `L`-Zyklus (`names[(names.index(controller_name) + 1) % len(names)]`)
funktioniert unverändert mit dem dritten Eintrag.

**Live-Submodus im Badge:** Am bestehenden `redraw()`-Aufrufpunkt in
`run_game()` wird ein Anzeige-String gebildet, der bei `SwingUpController`
den aktuellen `mode` anhängt (z. B. `AUTO [SwingUp: swinging]` /
`AUTO [SwingUp: balancing]`), bei den anderen Controllern unverändert nur
den Namen zeigt. Keine Änderung der `redraw()`-Signatur nötig — die
Zusammensetzung des Strings passiert vor dem Aufruf.

## 5. Testing

`tests/test_swingup_controller.py` (reine Funktionstests, kein FMU/Pygame
nötig):

- Energie-Formel: `E(φ=0, φ̇=0) = 0`, `E(φ=π, φ̇=0) = 2·m·g·l`.
- Vorzeichen-Regressionstest: bei `E < E_top` und `cos(φ)·φ̇ > 0` liefert
  `compute()` ein `τ`, dessen Vorzeichen dem erwarteten `a_cmd`-Vorzeichen
  entspricht (pinnt die Formel gegen Vorzeichenfehler, analog zum
  LQR-Gain-Pin-Test aus AP3 Teil 1).
- Umschalt-Automat: Zustand nahe `φ=π` mit kleinem `θ`/`φ̇` (innerhalb
  `CAPTURE_THETA`/`CAPTURE_VPHI`) → `mode` wechselt nach einem `compute()`-
  Aufruf zu `"lqr"`, und `τ` stimmt danach mit `LQRController().compute()`
  für denselben Zustand überein. Umgekehrt: aus `"lqr"` mit `|θ| >
  RELEASE_THETA` → zurück zu `"swingup"`.
- Clamping-Test: bei extremem Energiedefizit bleibt `τ` innerhalb
  `[-MAX_TAU, MAX_TAU]`.

**Interaktive Verifikation** (Mensch, echtes Display — wie bei allen
Pygame-/FMU-abhängigen Teilen seit AP2): diesmal **ohne** die
AP3-Teil-1-Krücke (temporärer `phi0`-Override) nötig, da der reale
Default-Start bei `67,5°` genau das Szenario ist, für das der Swing-up
gebaut ist. Zu beobachten: mehrere Schwingungen bis zum Erreichen der
aufrechten Lage, sichtbarer Phasenwechsel im Badge, danach stabiles
Balancieren. Zusätzlich testbar: während der Balance-Phase per Pfeiltaste
stören und beobachten, ob bei zu großer Auslenkung automatisch zurück in
die Swing-up-Phase gewechselt wird.

## 6. Nicht Teil dieses Designs

- Formaler Reglervergleich (Stabilität, Reaktionszeit, Robustheit) zwischen
  `SimpleController`, `LQRController` und `SwingUpController` — eigenes,
  späteres AP3-Teilstück, wie bereits im Teil-1-Design festgehalten.
- Pole Placement, nichtlinearer oder lernbasierter Regler (laut
  Projektanweisung optional) — nicht Teil dieses Designs.
- Feintuning von `K_ENERGY`/`CAPTURE_THETA`/`CAPTURE_VPHI`/`RELEASE_THETA`
  über die während der Implementierung festgelegten Startwerte hinaus (z. B.
  systematische Optimierung der Schwingzeit) — funktionale Korrektheit und
  Robustheit reichen für diesen Teilschritt, keine Performance-Optimierung.
