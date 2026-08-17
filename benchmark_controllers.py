"""Headless benchmark comparing the three AP3 controllers against the real
InvertedPendulumMB.fmu on stability, reaction time, and robustness.

See docs/superpowers/specs/2026-08-17-ap3-controller-comparison-design.md
for the full methodology.
"""

import math
import os
import shutil

import matplotlib
from fmpy import extract, read_model_description
from fmpy.fmi2 import FMU2Slave

from pendulum_game_controlled import LQRController, SimpleController, SwingUpController

# Headless backend: the benchmark writes PNGs and never opens a window.
# matplotlib.use() must run before pyplot is imported, hence the one
# deliberately out-of-order import below.
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# Success criterion shared across all scenarios: a controller "holds" the
# upright position from the moment |theta| stays below TOLERANCE_DEG for at
# least HOLD_DURATION seconds. TOLERANCE_DEG matches the game's own
# tight_bonus_zone (pendulum_game_controlled.py's compute_score_increment).
TOLERANCE_DEG = 5.0
HOLD_DURATION = 1.0

# Disturbance used by robustness(): a fixed-magnitude tau pulse held for
# ROBUSTNESS_KICK_STEPS game frames. Defined here (rather than inline in
# robustness()'s signature) so generate_report() can quote the real values
# instead of a hardcoded string that would silently drift on a retune.
ROBUSTNESS_KICK_TAU = 8.0
ROBUSTNESS_KICK_STEPS = 5

# Default envelope sweep grid: 2°, 4°, ... 90°. Note the sweep *starts* at
# ENVELOPE_SWEEP_START_DEG — a controller whose real threshold sits below
# that first tested point reports "no success in the swept range", which is
# not the same as having no stable region at all (see report §7).
ENVELOPE_SWEEP_START_DEG = 2
ENVELOPE_SWEEP_STOP_DEG = 92
ENVELOPE_SWEEP_STEP_DEG = 2


def held_from(t, theta, tolerance_rad, hold_duration):
    """Return the earliest time in `t` from which |theta| stays below
    `tolerance_rad` for at least `hold_duration` seconds, or None if that
    never happens within the given data."""
    entry_time = None
    for ti, th in zip(t, theta):
        if abs(th) < tolerance_rad:
            if entry_time is None:
                entry_time = ti
            elif ti - entry_time >= hold_duration:
                return entry_time
        else:
            entry_time = None
    return None


def find_capture_envelope(results_by_theta0):
    """results_by_theta0: dict {theta0_deg: bool success}, assumed to come
    from a monotonically increasing sweep. Return the largest theta0_deg
    for which every smaller tested theta0_deg also succeeded (i.e. the
    value just before the first failure), or None if the smallest tested
    theta0_deg already failed or the input is empty."""
    best = None
    for theta0_deg in sorted(results_by_theta0):
        if results_by_theta0[theta0_deg]:
            best = theta0_deg
        else:
            break
    return best


def simulate_run(fmu_path, controller, theta0_deg=None, vphi0=0.0, duration=20.0, on_frame=None):
    # Sub-stepped FMU co-simulation loop, same pattern as run_game() in
    # pendulum_game_controlled.py: explicit-Euler at the full 0.02s step
    # numerically injects energy into the lightly-damped pendulum, so tau
    # is held constant across SUBSTEPS smaller inner steps.
    dt = 0.02
    SUBSTEPS = 10
    MAX_TAU = 10.0
    substep_dt = dt / SUBSTEPS

    unzipdir = extract(fmu_path)
    desc = read_model_description(unzipdir)
    fmu = FMU2Slave(
        guid=desc.guid,
        unzipDirectory=unzipdir,
        modelIdentifier=desc.coSimulation.modelIdentifier,
    )
    fmu.instantiate()
    fmu.setupExperiment(startTime=0.0)

    def ref(name):
        for var in desc.modelVariables:
            if var.name == name:
                return var.valueReference
        raise Exception(f"'{name}' not found in FMU")

    if theta0_deg is not None:
        fmu.setReal([ref("phi0")], [math.pi + math.radians(theta0_deg)])
        fmu.setReal([ref("vphi0")], [vphi0])

    fmu.enterInitializationMode()
    fmu.exitInitializationMode()

    tau_ref = ref("tau")
    s_ref = ref("s")
    v_ref = ref("v")
    phi_ref = ref("phi")
    vphi_ref = ref("vphi")

    def wrapped_theta(phi):
        return (phi % (2 * math.pi)) - math.pi

    t = 0.0
    t_history = [0.0]
    theta_history = [wrapped_theta(fmu.getReal([phi_ref])[0])]

    try:
        while t < duration:
            phi = fmu.getReal([phi_ref])[0]
            vphi = fmu.getReal([vphi_ref])[0]
            s = fmu.getReal([s_ref])[0]
            v = fmu.getReal([v_ref])[0]
            theta = wrapped_theta(phi)

            tau = controller.compute(phi, vphi, s, v)
            if on_frame is not None:
                tau += on_frame(t, theta, vphi, s, v)
            tau = max(-MAX_TAU, min(MAX_TAU, tau))

            fmu.setReal([tau_ref], [tau])
            for _ in range(SUBSTEPS):
                t += substep_dt
                fmu.doStep(currentCommunicationPoint=t, communicationStepSize=substep_dt)

            t_history.append(t)
            theta_history.append(wrapped_theta(fmu.getReal([phi_ref])[0]))
    finally:
        fmu.terminate()
        fmu.freeInstance()
        shutil.rmtree(unzipdir)

    return t_history, theta_history


CONTROLLER_FACTORIES = {
    "PD": SimpleController,
    "LQR": LQRController,
    "SwingUp": SwingUpController,
}


class KickInjector:
    """on_frame callback for simulate_run: waits until the controller has
    held the upright position, then injects a fixed-magnitude tau pulse for
    `kick_steps` frames. Records when the kick started (`kick_time`, None
    if the controller never settled)."""

    def __init__(self, tolerance_rad, hold_duration, kick_tau, kick_steps):
        self.tolerance_rad = tolerance_rad
        self.hold_duration = hold_duration
        self.kick_tau = kick_tau
        self.kick_steps = kick_steps
        self.kick_time = None
        self._t_history = []
        self._theta_history = []
        self._kicked = False
        self._kick_frames_remaining = 0

    def offset_for(self, t, theta, vphi=None, s=None, v=None):
        self._t_history.append(t)
        self._theta_history.append(theta)

        if self._kick_frames_remaining > 0:
            self._kick_frames_remaining -= 1
            return self.kick_tau

        if not self._kicked:
            if held_from(self._t_history, self._theta_history, self.tolerance_rad, self.hold_duration) is not None:
                self._kicked = True
                self.kick_time = t
                self._kick_frames_remaining = self.kick_steps - 1
                return self.kick_tau

        return 0.0


def envelope_sweep(fmu_path, controller_factory, theta0_values_deg=None,
                    tolerance_deg=TOLERANCE_DEG, hold_duration=HOLD_DURATION,
                    duration=20.0, simulate_fn=simulate_run):
    if theta0_values_deg is None:
        theta0_values_deg = list(
            range(ENVELOPE_SWEEP_START_DEG, ENVELOPE_SWEEP_STOP_DEG, ENVELOPE_SWEEP_STEP_DEG)
        )
    tolerance_rad = math.radians(tolerance_deg)

    results_by_theta0 = {}
    consecutive_failures = 0
    for theta0_deg in theta0_values_deg:
        controller = controller_factory()
        t, theta = simulate_fn(fmu_path, controller, theta0_deg=theta0_deg, duration=duration)
        success = held_from(t, theta, tolerance_rad, hold_duration) is not None
        results_by_theta0[theta0_deg] = success
        consecutive_failures = 0 if success else consecutive_failures + 1
        if consecutive_failures >= 2:
            break

    return {
        "results_by_theta0": results_by_theta0,
        "envelope_deg": find_capture_envelope(results_by_theta0),
    }


def reaction_time(fmu_path, controller_factory, theta0_values_deg=(2.0, 10.0),
                   tolerance_deg=TOLERANCE_DEG, hold_duration=HOLD_DURATION,
                   duration=20.0, simulate_fn=simulate_run):
    tolerance_rad = math.radians(tolerance_deg)
    settling_times = {}
    trajectories = {}
    for theta0_deg in theta0_values_deg:
        controller = controller_factory()
        t, theta = simulate_fn(fmu_path, controller, theta0_deg=theta0_deg, duration=duration)
        settling_times[theta0_deg] = held_from(t, theta, tolerance_rad, hold_duration)
        trajectories[theta0_deg] = (t, theta)
    return {"settling_times": settling_times, "trajectories": trajectories}


def robustness(fmu_path, controller_factory, theta0_deg=2.0,
                tolerance_deg=TOLERANCE_DEG, hold_duration=HOLD_DURATION,
                kick_tau=ROBUSTNESS_KICK_TAU, kick_steps=ROBUSTNESS_KICK_STEPS,
                duration=20.0, simulate_fn=simulate_run):
    tolerance_rad = math.radians(tolerance_deg)
    injector = KickInjector(tolerance_rad, hold_duration, kick_tau, kick_steps)
    controller = controller_factory()
    t, theta = simulate_fn(
        fmu_path, controller, theta0_deg=theta0_deg, duration=duration, on_frame=injector.offset_for
    )

    if injector.kick_time is None:
        return {
            "kicked": False,
            "recovery_time": None,
            "kick_time": None,
            "peak_post_kick_deviation_deg": None,
            "trajectory": (t, theta),
        }

    post_kick_t = [ti for ti in t if ti >= injector.kick_time]
    post_kick_theta = [th for ti, th in zip(t, theta) if ti >= injector.kick_time]
    recovered_at = held_from(post_kick_t, post_kick_theta, tolerance_rad, hold_duration)
    recovery_time = None if recovered_at is None else recovered_at - injector.kick_time

    # recovery_time alone is misleading when the pulse never pushes the
    # pendulum out of the tolerance band: the controller then scores 0.00s
    # "by construction", not by being fast. The peak deviation over the same
    # post-kick slice says how much the pulse actually disturbed it.
    peak_post_kick_deviation_deg = max(
        (abs(math.degrees(th)) for th in post_kick_theta), default=None
    )

    return {
        "kicked": True,
        "recovery_time": recovery_time,
        "kick_time": injector.kick_time,
        "peak_post_kick_deviation_deg": peak_post_kick_deviation_deg,
        "trajectory": (t, theta),
    }


def swingup_capture(fmu_path, controller_factory, tolerance_deg=TOLERANCE_DEG,
                     hold_duration=HOLD_DURATION, duration=20.0, simulate_fn=simulate_run):
    tolerance_rad = math.radians(tolerance_deg)
    controller = controller_factory()
    t, theta = simulate_fn(fmu_path, controller, theta0_deg=None, duration=duration)
    capture_time = held_from(t, theta, tolerance_rad, hold_duration)
    return {"capture_time": capture_time, "trajectory": (t, theta)}


def run_all_scenarios(fmu_path):
    envelope = {
        name: envelope_sweep(fmu_path, factory)
        for name, factory in CONTROLLER_FACTORIES.items()
        if name != "SwingUp"
    }
    reaction = {
        name: reaction_time(fmu_path, factory)
        for name, factory in CONTROLLER_FACTORIES.items()
    }
    robust = {
        name: robustness(fmu_path, factory)
        for name, factory in CONTROLLER_FACTORIES.items()
    }
    swingup = swingup_capture(fmu_path, CONTROLLER_FACTORIES["SwingUp"])
    return {
        "envelope": envelope,
        "reaction_time": reaction,
        "robustness": robust,
        "swingup_capture": swingup,
    }


def _envelope_text(envelope_deg):
    """Shared wording for an envelope result, so §3's table cell and §7's
    quote of that cell can't drift apart."""
    return "kein Erfolg im gesweepten Bereich" if envelope_deg is None else f"{envelope_deg:.0f}°"


def generate_report(results):
    lines = ["# AP3 Reglervergleich", ""]

    lines.append("## 1. Kontext")
    lines.append("")
    lines.append(
        "Vergleich der drei AP3-Regler (PD/SimpleController, LQR, SwingUp) "
        "gegen die reale InvertedPendulumMB.fmu, gefordert durch die "
        "Projektanweisung (AP3: 'Vergleich der Regler hinsichtlich "
        "Stabilitaet, Reaktionszeit und Robustheit gegenueber "
        "Stoerungen'). Details zur Methodik in "
        "docs/superpowers/specs/2026-08-17-ap3-controller-comparison-design.md."
    )
    lines.append("")

    lines.append("## 2. Methodik")
    lines.append("")
    lines.append(
        f"Erfolgskriterium: |theta| < {TOLERANCE_DEG:.0f}° fuer mindestens "
        f"{HOLD_DURATION:.1f}s ununterbrochen (theta = Abweichung von der "
        "aufrechten Lage phi=pi). Stabilitaet wird als groesste "
        "erfolgreiche Anfangsauslenkung in einem Sweep von "
        f"{ENVELOPE_SWEEP_START_DEG}° bis "
        f"{ENVELOPE_SWEEP_STOP_DEG - ENVELOPE_SWEEP_STEP_DEG}° in "
        f"{ENVELOPE_SWEEP_STEP_DEG}°-Schritten gemessen (PD/LQR; "
        f"{ENVELOPE_SWEEP_START_DEG}° ist damit zugleich die kleinste "
        "getestete Auslenkung, nicht nur die Schrittweite), "
        "Reaktionszeit als Einschwingzeit bei festen "
        "Baseline-Auslenkungen (2°, 10°), Robustheit als Erholungszeit "
        "und maximale Auslenkung nach einem Kraft-Puls auf tau "
        f"(KICK_TAU={ROBUSTNESS_KICK_TAU} fuer "
        f"KICK_STEPS={ROBUSTNESS_KICK_STEPS} "
        "Frames) waehrend des eingeschwungenen Zustands, und die "
        "Swing-up-Faehigkeit als Einschwingzeit ab der realen "
        "Spiel-Anfangsbedingung (phi0=0.75*pi/2)."
    )
    lines.append("")

    lines.append("## 3. Stabilitaet (Einzugsbereich)")
    lines.append("")
    lines.append("| Regler | Einzugsbereich |")
    lines.append("|---|---|")
    for name, data in results["envelope"].items():
        lines.append(f"| {name} | {_envelope_text(data['envelope_deg'])} |")
    lines.append("| SwingUp | siehe Swing-up-Ergebnis unten |")
    lines.append("")

    lines.append("## 4. Reaktionszeit")
    lines.append("")
    lines.append("| Regler | 2° | 10° |")
    lines.append("|---|---|---|")
    for name, data in results["reaction_time"].items():
        row = [name]
        for theta0 in (2.0, 10.0):
            t = data["settling_times"].get(theta0)
            row.append("kein Einschwingen" if t is None else f"{t:.2f}s")
        lines.append("| " + " | ".join(row) + " |")
    lines.append("")
    lines.append(
        f"Hinweis zu '0.00s': Gemessen wird der fruehste Zeitpunkt, ab dem "
        f"|theta| ununterbrochen {HOLD_DURATION:.1f}s lang unter "
        f"{TOLERANCE_DEG:.0f}° bleibt. Die Baseline 2° liegt bereits "
        f"innerhalb dieser Toleranzzone - ein Regler, der sie nie "
        f"verlaesst, erhaelt damit per Definition des Erfolgskriteriums "
        f"0.00s, was *nicht* 'sofortige Reaktion' bedeutet. Dasselbe gilt "
        f"fuer die Erholungszeiten in Abschnitt 5: bleibt die Auslenkung "
        f"nach dem Kraft-Puls durchgehend unter {TOLERANCE_DEG:.0f}°, ist "
        f"die Erholungszeit ebenfalls 0.00s - wie stark der Puls "
        f"tatsaechlich gestoert hat, zeigt dort erst die Spalte 'Max. "
        f"Auslenkung nach Puls'."
    )
    lines.append("")

    lines.append("## 5. Robustheit (Kraft-Puls)")
    lines.append("")
    lines.append("| Regler | Erholungszeit | Max. Auslenkung nach Puls |")
    lines.append("|---|---|---|")
    for name, data in results["robustness"].items():
        if not data["kicked"]:
            value = "Regler hat vor dem Puls nicht eingeschwungen"
        elif data["recovery_time"] is None:
            value = "keine Erholung"
        else:
            value = f"{data['recovery_time']:.2f}s"
        peak = data.get("peak_post_kick_deviation_deg")
        peak_value = "kein Puls ausgeloest" if peak is None else f"{peak:.2f}°"
        lines.append(f"| {name} | {value} | {peak_value} |")
    lines.append("")

    lines.append("## 6. Swing-up ab realer Anfangsbedingung")
    lines.append("")
    capture_time = results["swingup_capture"]["capture_time"]
    value = "kein Capture" if capture_time is None else f"{capture_time:.2f}s"
    lines.append(f"SwingUp: {value}")
    lines.append("PD/LQR: N/A (strukturell nicht loesbar, siehe Design-Dokument §3.4)")
    lines.append("")

    lines.append("## 7. Diskussion")
    lines.append("")

    def _envelope_deg(name):
        data = results["envelope"].get(name)
        return None if data is None else data["envelope_deg"]

    pd_env_text = _envelope_text(_envelope_deg("PD"))
    lqr_env_text = _envelope_text(_envelope_deg("LQR"))
    peaks = {
        name: data.get("peak_post_kick_deviation_deg")
        for name, data in results["robustness"].items()
        if data.get("peak_post_kick_deviation_deg") is not None
    }
    peaks_text = (
        "Maximalauslenkung: "
        + ", ".join(f"{name} {value:.2f}°" for name, value in peaks.items())
        if peaks else "kein Regler war zum Puls-Zeitpunkt eingeschwungen"
    )

    lines.append(
        "**PD (SimpleController) ist um die aufrechte Lage nicht "
        "asymptotisch stabilisierend.** Das ist kein gradueller "
        "Unterschied zu LQR, sondern ein struktureller: eine "
        "Eigenwertanalyse der geschlossenen Regelschleife (A - B*K mit "
        "K_pd = [0, 0, K_PHI, K_VPHI] entsprechend "
        "SimpleController.compute) um phi=pi liefert einen Eigenwert mit "
        "positivem Realteil (~1.15 beim frueheren d_pend=0.15, ~1.28 beim "
        "aktuellen d_pend=0.01) - die aufrechte Lage bleibt unter PD "
        "instabil, der Regler ist dort also nicht 'nur schwach gedaempft' "
        "und hat auch keinen kleinen, aber positiven Einzugsbereich im "
        "Sinne asymptotischer Stabilitaet. Dass PD das Erfolgskriterium "
        "bei sehr kleinen Anfangsauslenkungen (empirisch bis ~1.5°) "
        "trotzdem erfuellt, liegt allein daran, dass die Divergenz von "
        f"einem so kleinen Start aus laenger als das "
        f"{HOLD_DURATION:.1f}s-Haltefenster braucht, um die "
        f"{TOLERANCE_DEG:.0f}°-Toleranzgrenze zu ueberschreiten. Der Sweep "
        f"in Abschnitt 3 *beginnt* bei {ENVELOPE_SWEEP_START_DEG}° "
        f"({ENVELOPE_SWEEP_START_DEG}° ist Startwert und Schrittweite "
        f"zugleich); '{pd_env_text}' fuer PD heisst deshalb, dass PDs "
        "reale Schwelle (~1.5-2°) unterhalb des ersten getesteten Punktes "
        "liegt - nicht, dass PD bei beliebig kleiner Auslenkung sofort "
        "versagt."
    )
    lines.append("")

    lines.append(
        f"**LQR erreicht demgegenueber {lqr_env_text} (Abschnitt 3).** Der "
        "LQR nutzt den vollen Zustand (s, v, theta, theta_dot) statt nur "
        "phi/vphi und ist um phi=pi tatsaechlich asymptotisch "
        f"stabilisierend, deshalb ein messbarer Einzugsbereich von "
        f"{lqr_env_text} gegenueber PDs Schwelle unterhalb von "
        f"{ENVELOPE_SWEEP_START_DEG}°. Beide sind aber auf eine "
        "Linearisierung um die aufrechte Lage angewiesen und koennen die "
        "reale Spiel-Anfangsbedingung (~112.5° von der aufrechten Lage) "
        "strukturell nicht erreichen (Abschnitt 6) - genau der in der "
        "Projektanweisung genannte Vergleichspunkt fuer den "
        "energiebasierten SwingUp-Regler (siehe AP1_Validierung.md §6 fuer "
        "die zugrundeliegende Physik)."
    )
    lines.append("")

    lines.append(
        "**Die SwingUp-Zeilen in den Abschnitten 4 und 5 sind kein "
        "eigenstaendiges Ergebnis.** SwingUpController schaltet per "
        "Hysterese (CAPTURE_THETA=10°, CAPTURE_VPHI=2.0) auf seinen "
        "internen LQRController um; beide Baselines (2°, 10°) liegen "
        "innerhalb dieses Fangbereichs, SwingUp delegiert dort also von "
        "Anfang an direkt an LQR. Dass seine Zahlen mit LQRs identisch "
        "sind, ist deshalb zu erwarten und weder Zufall noch Fehler - es "
        "sagt aber auch nichts ueber SwingUp selbst aus. Das "
        "unterscheidende Verhalten des SwingUp-Reglers zeigt sich erst in "
        "Abschnitt 6, im eigentlichen Swing-up ab der realen "
        "Anfangsbedingung."
    )
    lines.append("")

    lines.append(
        "**Zur Robustheit (Abschnitt 5): die Erholungszeiten sind nur "
        "zusammen mit der maximalen Auslenkung zu lesen.** Der Kraft-Puls "
        f"(KICK_TAU={ROBUSTNESS_KICK_TAU} fuer "
        f"{ROBUSTNESS_KICK_STEPS} Frames) lenkt das Pendel gegen die "
        f"eingeschwungenen Regler nur minimal aus ({peaks_text}) und "
        f"bleibt damit deutlich innerhalb der "
        f"{TOLERANCE_DEG:.0f}°-Toleranzzone. Eine "
        "Erholungszeit von 0.00s bedeutet hier also 'die Stoerung hat die "
        "Toleranzzone nie verlassen', nicht 'sofort ausgeregelt'; der "
        "Puls ist fuer diese Regler zu schwach, um als Stoerung im Sinne "
        "des Kriteriums zu zaehlen. Ein Regler, der vor dem Puls gar nicht "
        "erst eingeschwungen war (PD, siehe oben), wird als 'nicht "
        "eingeschwungen' statt mit einer Erholungszeit gefuehrt und ist "
        "entsprechend gesondert zu lesen. Ein aussagekraeftigerer "
        "Robustheitsvergleich braeuchte einen staerkeren Puls oder eine "
        "Magnitudenvariation - bewusst ausserhalb des Scopes dieses "
        "Benchmarks (Design-Dokument §7)."
    )
    lines.append("")

    return "\n".join(lines)


# SwingUp is drawn dashed on purpose: at the 2°/10° baselines it is inside
# its own CAPTURE_THETA hysteresis and delegates to its internal
# LQRController, so its trace is numerically identical to LQR's and would
# otherwise paint over it, making the LQR legend entry invisible.
PLOT_LINESTYLES = {"PD": "-", "LQR": "-", "SwingUp": "--"}


def _with_wrap_breaks(theta_deg):
    """theta is wrapped to (-180°, 180°], so a full rotation shows up as a
    ~360° jump between neighbouring samples. Blank those samples out (NaN)
    so matplotlib breaks the line there instead of drawing a vertical
    stripe across the whole plot, which reads as a spike rather than a
    wraparound."""
    out = list(theta_deg)
    for i in range(1, len(theta_deg)):
        if abs(theta_deg[i] - theta_deg[i - 1]) > 180.0:
            out[i] = float("nan")
    return out


def _plot_trace(name, t, theta_deg):
    plt.plot(
        t,
        _with_wrap_breaks(theta_deg),
        label=name,
        linestyle=PLOT_LINESTYLES.get(name, "-"),
        linewidth=2 if PLOT_LINESTYLES.get(name) == "--" else 1.5,
    )


def _note_offscale_traces(traces_deg, ylim):
    """Add a caption when a trace leaves the (deliberately zoomed) y-range,
    so a reader doesn't mistake a clipped divergence for missing data."""
    offscale = [n for n, values in traces_deg if values and (max(values) > ylim[1] or min(values) < ylim[0])]
    if offscale:
        plt.annotate(
            f"{', '.join(offscale)}: verlaesst den Achsenbereich (Divergenz bis +/-180°)",
            xy=(0.5, 0.02), xycoords="axes fraction", ha="center", fontsize=8, color="dimgray",
        )


def generate_plots(results, output_dir):
    os.makedirs(output_dir, exist_ok=True)

    names = list(results["envelope"].keys())
    values = [(results["envelope"][n]["envelope_deg"] or 0) for n in names]
    plt.figure()
    plt.bar(names, values)
    plt.ylabel("Einzugsbereich (Grad)")
    plt.title("Stabilitaet: Einzugsbereich je Regler")
    # A missing envelope draws as a zero-height bar, which would read as
    # "exactly 0°". Label it as what it is: below the smallest swept value.
    for name in names:
        if results["envelope"][name]["envelope_deg"] is None:
            plt.text(
                name, 0.2, f"kein Erfolg\nab {ENVELOPE_SWEEP_START_DEG}° aufwaerts",
                ha="center", va="bottom", fontsize=8, color="dimgray",
            )
    # SwingUp is not swept (it is not a balance-only controller); mark its
    # separate result here so the missing bar is explained in the plot
    # itself. Requested by the design document §5.2.
    capture_time = results["swingup_capture"]["capture_time"]
    swingup_note = (
        "SwingUp: nicht gesweept (kein reiner Balance-Regler) - "
        + ("kein Capture ab realer Anfangsbedingung" if capture_time is None
           else f"Capture ab realer Anfangsbedingung nach {capture_time:.2f}s")
    )
    plt.annotate(
        swingup_note, xy=(0.5, -0.16), xycoords="axes fraction",
        ha="center", fontsize=8, color="dimgray",
    )
    plt.subplots_adjust(bottom=0.22)
    plt.savefig(os.path.join(output_dir, "envelope_sweep.png"))
    plt.close()

    for theta0 in (2.0, 10.0):
        plt.figure()
        traces_deg = []
        for name, data in results["reaction_time"].items():
            t, theta = data["trajectories"][theta0]
            theta_deg = [math.degrees(th) for th in theta]
            _plot_trace(name, t, theta_deg)
            traces_deg.append((name, theta_deg))
        plt.axhline(5.0, color="gray", linestyle="--")
        plt.axhline(-5.0, color="gray", linestyle="--")
        # Zoom to the controllers that actually settle: PD diverges to
        # +/-180° and would otherwise squash LQR/SwingUp into a flat line.
        bound = 1.5 * max(TOLERANCE_DEG, theta0)
        plt.ylim(-bound, bound)
        _note_offscale_traces(traces_deg, (-bound, bound))
        plt.xlabel("t (s)")
        plt.ylabel("theta (deg)")
        plt.title(f"Reaktionszeit ab {theta0:.0f}°")
        plt.legend()
        plt.savefig(os.path.join(output_dir, f"reaction_time_{int(theta0)}deg.png"))
        plt.close()

    plt.figure()
    traces_deg = []
    kick_times = []
    for name, data in results["robustness"].items():
        t, theta = data["trajectory"]
        theta_deg = [math.degrees(th) for th in theta]
        _plot_trace(name, t, theta_deg)
        traces_deg.append((name, theta_deg))
        if data["kick_time"] is not None:
            plt.axvline(data["kick_time"], color="red", linestyle=":")
            kick_times.append(data["kick_time"])
    plt.axhline(5.0, color="gray", linestyle="--")
    plt.axhline(-5.0, color="gray", linestyle="--")
    # Same zoom rationale as above, plus a time window around the kick so
    # the actual pulse response (~1° for LQR/SwingUp) is visible at all.
    ylim = (-8.0, 8.0)
    plt.ylim(*ylim)
    if kick_times:
        plt.xlim(max(0.0, min(kick_times) - 1.0), max(kick_times) + 4.0)
    _note_offscale_traces(traces_deg, ylim)
    plt.xlabel("t (s)")
    plt.ylabel("theta (deg)")
    plt.title("Robustheit: Kraft-Puls-Reaktion")
    plt.legend()
    plt.savefig(os.path.join(output_dir, "robustness_kick.png"))
    plt.close()

    t, theta = results["swingup_capture"]["trajectory"]
    plt.figure()
    plt.plot(t, [math.degrees(th) for th in theta])
    capture_time = results["swingup_capture"]["capture_time"]
    if capture_time is not None:
        plt.axvline(capture_time, color="red", linestyle=":")
    plt.xlabel("t (s)")
    plt.ylabel("theta (deg)")
    plt.title("Swing-up ab realer Anfangsbedingung")
    plt.savefig(os.path.join(output_dir, "swingup_capture.png"))
    plt.close()


if __name__ == "__main__":
    fmu_path = os.path.abspath("InvertedPendulumMB.fmu")
    if not os.path.exists(fmu_path):
        raise SystemExit(
            "InvertedPendulumMB.fmu not found in project root. "
            "Copy the (gitignored) FMU into place before running this benchmark."
        )

    results = run_all_scenarios(fmu_path)

    report = generate_report(results)
    with open("AP3_Reglervergleich.md", "w", encoding="utf-8") as f:
        f.write(report)

    generate_plots(results, "benchmark_plots")

    print("Wrote AP3_Reglervergleich.md and benchmark_plots/")
